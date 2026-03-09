import sys
import time

import pigpio

# GPIO pins (BCM numbering)
TRIG = 18
ECHO = 19

# HC-SR04 characteristics and guard rails
MIN_DISTANCE_CM = 2             # datasheet lower limit
MAX_DISTANCE_CM = 400           # practical upper limit
ECHO_RISE_TIMEOUT_S = 0.02      # wait up to 20 ms for echo to go high
ECHO_FALL_TIMEOUT_S = 0.065     # wait up to 65 ms for echo to drop
MIN_INTERVAL_S = 0.08           # minimum gap between trigger pulses (slower to reduce ringing)
PRINT_INTERVAL_S = 1.0          # overall reporting interval
SAMPLES_PER_REPORT = 10         # how many pulses to gather before printing


def setup(pi: pigpio.pi) -> None:
    """Configure GPIO for the ultrasonic sensor."""
    pi.set_mode(TRIG, pigpio.OUTPUT)
    pi.set_mode(ECHO, pigpio.INPUT)
    pi.set_pull_up_down(ECHO, pigpio.PUD_DOWN)  # keep echo low when idle
    pi.write(TRIG, 0)
    time.sleep(0.05)


_last_trigger_time = 0.0


def get_distance_cm(pi: pigpio.pi, debug: bool = False) -> float | None:
    """Return distance in cm or None when out of range/timeout.

    When debug is True, prints which stage times out and the raw pulse width.
    """
    global _last_trigger_time
    now = time.perf_counter()
    if now - _last_trigger_time < MIN_INTERVAL_S:
        time.sleep(MIN_INTERVAL_S - (now - _last_trigger_time))

    # If ECHO is already high before we trigger, that's an error/stuck-high case.
    if pi.read(ECHO) == 1:
        if debug:
            print("[debug] ECHO already high before trigger; skipping measurement")
        return None

    pi.gpio_trigger(TRIG, 10, 1)  # 10 µs pulse
    _last_trigger_time = time.perf_counter()
    t0 = _last_trigger_time

    # Wait for echo to rise
    while pi.read(ECHO) == 0:
        if time.perf_counter() - t0 > ECHO_RISE_TIMEOUT_S:
            if debug:
                waited = (time.perf_counter() - t0) * 1000
                print(f"[debug] timeout waiting for ECHO rise after {waited:.2f} ms")
            return None

    start = time.perf_counter()

    # Wait for echo to fall
    while pi.read(ECHO) == 1:
        if time.perf_counter() - start > ECHO_FALL_TIMEOUT_S:
            if debug:
                waited = (time.perf_counter() - start) * 1000
                print(f"[debug] timeout waiting for ECHO fall after {waited:.2f} ms")
            return None

    duration = time.perf_counter() - start
    distance = (duration * 34300) / 2  # speed of sound 34300 cm/s

    if debug:
        print(f"[debug] pulse {duration*1e6:.0f} µs -> {distance:.1f} cm")

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

            for i in range(SAMPLES_PER_REPORT):
                dist = get_distance_cm(pi, debug=debug)
                if dist is not None:
                    samples.append(dist)
                else:
                    invalid_count += 1
                # store last raw duration even when None for visibility
                # (already printed in debug mode)
                time.sleep(MIN_INTERVAL_S)

            if invalid_count >= 7:
                print(f"Skipped set: {invalid_count}/{SAMPLES_PER_REPORT} samples invalid")
            elif samples:
                # median is robust against occasional bad pulses
                samples.sort()
                median = samples[len(samples) // 2]
                print(f"Distance: {median:.1f} cm (valid {len(samples)}/{SAMPLES_PER_REPORT}, invalid {invalid_count})")
            else:
                print("Out of range (no valid samples)")

            # ensure ~500 ms between printed lines
            elapsed = time.perf_counter() - report_timer
            if elapsed < PRINT_INTERVAL_S:
                time.sleep(PRINT_INTERVAL_S - elapsed)
            report_timer = time.perf_counter()
    except KeyboardInterrupt:
        pass
    finally:
        pi.write(TRIG, 0)
        pi.stop()


if __name__ == "__main__":
    main()
