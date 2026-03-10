import sys
import time

import pigpio

# GPIO pins (BCM numbering)
TRIG = 24
ECHO = 25

# HC-SR04 guard rails
MIN_DISTANCE_CM = 2
MAX_DISTANCE_CM = 400
LOCK_DISTANCE_CM = 1000.0
LOCK_TOLERANCE_CM = 120.0

# Pulse/noise handling
ECHO_RISE_TIMEOUT_S = 0.03
ECHO_FALL_TIMEOUT_S = 0.025
ECHO_GLITCH_US = 350
STUCK_HIGH_RECOVER_S = 0.01
PRINT_INTERVAL_S = 1.2

# Speed profiles for driving conditions
SPEED_PROFILES = {
    "slow":   {"min_interval": 0.24, "samples": 6,  "min_valid": 4},
    "normal": {"min_interval": 0.20, "samples": 8,  "min_valid": 5},
    "fast":   {"min_interval": 0.14, "samples": 10, "min_valid": 6},
}

# Smoothing / spike handling
MAX_STEP_CM = 30.0      # reject sudden unrealistic jump from last stable value
ALPHA = 0.35            # EMA smoothing factor


def parse_profile() -> dict:
    profile = "normal"
    for arg in sys.argv:
        if arg.startswith("--speed="):
            profile = arg.split("=", 1)[1].strip().lower()
    if profile not in SPEED_PROFILES:
        print(f"[warn] unknown speed profile '{profile}', using 'normal'")
        profile = "normal"
    return SPEED_PROFILES[profile]


def setup(pi: pigpio.pi) -> None:
    pi.set_mode(TRIG, pigpio.OUTPUT)
    pi.set_mode(ECHO, pigpio.INPUT)
    pi.set_pull_up_down(ECHO, pigpio.PUD_DOWN)
    pi.set_glitch_filter(ECHO, ECHO_GLITCH_US)
    pi.write(TRIG, 0)
    time.sleep(0.08)


_last_trigger_time = 0.0


def get_distance_cm(pi: pigpio.pi, min_interval_s: float, debug: bool = False) -> float | None:
    global _last_trigger_time

    now = time.perf_counter()
    if now - _last_trigger_time < min_interval_s:
        time.sleep(min_interval_s - (now - _last_trigger_time))

    if pi.read(ECHO) == 1:
        t_h = time.perf_counter()
        while pi.read(ECHO) == 1 and (time.perf_counter() - t_h) < STUCK_HIGH_RECOVER_S:
            pass
        if pi.read(ECHO) == 1:
            if debug:
                print("[debug] ECHO stuck high before trigger; skipping sample")
            return None

    pi.gpio_trigger(TRIG, 10, 1)
    _last_trigger_time = time.perf_counter()
    t0 = _last_trigger_time

    while pi.read(ECHO) == 0:
        if time.perf_counter() - t0 > ECHO_RISE_TIMEOUT_S:
            if debug:
                waited = (time.perf_counter() - t0) * 1000
                print(f"[debug] timeout waiting ECHO rise after {waited:.2f} ms")
            return None

    start = time.perf_counter()

    while pi.read(ECHO) == 1:
        if time.perf_counter() - start > ECHO_FALL_TIMEOUT_S:
            if debug:
                waited = (time.perf_counter() - start) * 1000
                print(f"[debug] timeout waiting ECHO fall after {waited:.2f} ms (invalid long pulse)")
            return None

    duration = time.perf_counter() - start
    distance = (duration * 34300) / 2

    if debug:
        print(f"[debug] pulse {duration*1e6:.0f} us -> {distance:.1f} cm")

    if abs(distance - LOCK_DISTANCE_CM) <= LOCK_TOLERANCE_CM:
        if debug:
            print(f"[debug] rejected lock-like reading near 1000 cm: {distance:.1f} cm")
        return None

    if distance < MIN_DISTANCE_CM or distance > MAX_DISTANCE_CM:
        return None

    return distance


def main() -> None:
    pi = pigpio.pi()
    if not pi.connected:
        print("pigpio daemon not running or not reachable.", file=sys.stderr)
        sys.exit(1)

    profile = parse_profile()
    min_interval_s = profile["min_interval"]
    samples_per_report = profile["samples"]
    min_valid = profile["min_valid"]
    debug = "--debug" in sys.argv

    print(
        f"Profile: interval={min_interval_s:.2f}s, samples={samples_per_report}, "
        f"min_valid={min_valid}"
    )

    last_stable: float | None = None

    try:
        setup(pi)
        report_timer = time.perf_counter()

        while True:
            samples: list[float] = []
            invalid_count = 0

            for _ in range(samples_per_report):
                dist = get_distance_cm(pi, min_interval_s=min_interval_s, debug=debug)
                if dist is not None:
                    samples.append(dist)
                else:
                    invalid_count += 1

            if len(samples) >= min_valid:
                samples.sort()
                median = samples[len(samples) // 2]

                # Reject unrealistic jumps then smooth output.
                if last_stable is not None and abs(median - last_stable) > MAX_STEP_CM:
                    if debug:
                        print(
                            f"[debug] jump rejected: median={median:.1f} cm "
                            f"last={last_stable:.1f} cm"
                        )
                    output = last_stable
                else:
                    if last_stable is None:
                        output = median
                    else:
                        output = (ALPHA * median) + ((1.0 - ALPHA) * last_stable)

                last_stable = output
                print(
                    f"Distance: {output:.1f} cm "
                    f"(valid {len(samples)}/{samples_per_report}, invalid {invalid_count})"
                )
            else:
                print(f"Skipped set: {invalid_count}/{samples_per_report} invalid")

            elapsed = time.perf_counter() - report_timer
            if elapsed < PRINT_INTERVAL_S:
                time.sleep(PRINT_INTERVAL_S - elapsed)
            report_timer = time.perf_counter()

    except KeyboardInterrupt:
        pass
    finally:
        pi.write(TRIG, 0)
        pi.set_glitch_filter(ECHO, 0)
        pi.stop()


if __name__ == "__main__":
    main()
