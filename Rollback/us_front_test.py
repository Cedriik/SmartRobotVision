import sys
import time

import pigpio

# GPIO pins (BCM numbering)
TRIG = 18
ECHO = 19

# HC-SR04 guard rails
MIN_DISTANCE_CM = 2
MAX_DISTANCE_CM = 400
LOCK_DISTANCE_CM = 1000.0
LOCK_TOLERANCE_CM = 120.0

# Timing
ECHO_RISE_TIMEOUT_S = 0.03
ECHO_FALL_TIMEOUT_S = 0.025       # hard cap: ~400 cm round-trip is ~23.3 ms
MIN_INTERVAL_S = 0.20             # slower trigger interval to reduce ringing/crosstalk
PRINT_INTERVAL_S = 1.2
SAMPLES_PER_REPORT = 8
ECHO_GLITCH_US = 350              # stronger digital filtering against narrow spikes
STUCK_HIGH_RECOVER_S = 0.01       # wait up to 10 ms for ECHO to drop before skipping


def setup(pi: pigpio.pi) -> None:
    pi.set_mode(TRIG, pigpio.OUTPUT)
    pi.set_mode(ECHO, pigpio.INPUT)
    pi.set_pull_up_down(ECHO, pigpio.PUD_DOWN)
    pi.set_glitch_filter(ECHO, ECHO_GLITCH_US)
    pi.write(TRIG, 0)
    time.sleep(0.08)


_last_trigger_time = 0.0


def get_distance_cm(pi: pigpio.pi, debug: bool = False) -> float | None:
    global _last_trigger_time

    now = time.perf_counter()
    if now - _last_trigger_time < MIN_INTERVAL_S:
        time.sleep(MIN_INTERVAL_S - (now - _last_trigger_time))

    # Recover from line stuck high before triggering.
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

    try:
        setup(pi)
        debug = "--debug" in sys.argv
        report_timer = time.perf_counter()

        while True:
            samples: list[float] = []
            invalid_count = 0

            for _ in range(SAMPLES_PER_REPORT):
                dist = get_distance_cm(pi, debug=debug)
                if dist is not None:
                    samples.append(dist)
                else:
                    invalid_count += 1

            if invalid_count >= (SAMPLES_PER_REPORT - 2):
                print(f"Skipped set: {invalid_count}/{SAMPLES_PER_REPORT} invalid")
            elif samples:
                samples.sort()
                median = samples[len(samples) // 2]
                print(
                    f"Distance: {median:.1f} cm "
                    f"(valid {len(samples)}/{SAMPLES_PER_REPORT}, invalid {invalid_count})"
                )
            else:
                print("Out of range (no valid samples)")

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
