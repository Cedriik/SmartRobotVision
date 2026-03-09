import sys
import time

import pigpio

# --- Motor configuration ---
USE_PWM = False          # Set False to drive EN pins HIGH (no speed control)
PWM_FREQ = 1000          # Hz for PWM
DEFAULT_SPEED = 70       # 0-100 starting duty cycle
PWM_DUTY = DEFAULT_SPEED
DUTY_MAX = 255           # pigpio uses 0-255 duty by default

IN1 = 17
IN2 = 27
IN3 = 22
IN4 = 25
ENA = 13  # BCM13 (pin 33)
ENB = 12  # BCM12 (pin 32)

# --- Ultrasonic configuration (borrowed from ultrasonic_front.py) ---
TRIG = 18
ECHO = 19
MIN_DISTANCE_CM = 2
STOP_THRESHOLD_CM = 20     # stop motors at or below this distance
MAX_DISTANCE_CM = 400
ECHO_RISE_TIMEOUT_S = 0.02
ECHO_FALL_TIMEOUT_S = 0.065
MIN_INTERVAL_S = 0.08
PRINT_INTERVAL_S = 1.0
SAMPLES_PER_REPORT = 10
INVALID_SKIP_THRESHOLD = 7

# pigpio instance (shared with ultrasonic)
pi = None

# --- Motor helpers ---


def set_speed(duty):
    """Change PWM duty cycle on both enables."""
    global PWM_DUTY
    PWM_DUTY = duty
    if USE_PWM:
        dc = max(0, min(100, duty))
        pig_duty = int(dc * DUTY_MAX / 100)
        pi.set_PWM_dutycycle(ENA, pig_duty)
        pi.set_PWM_dutycycle(ENB, pig_duty)


def stop():
    pi.write(IN1, 0)
    pi.write(IN2, 0)
    pi.write(IN3, 0)
    pi.write(IN4, 0)
    if USE_PWM:
        set_speed(0)


def forward():
    set_speed(DEFAULT_SPEED)
    pi.write(IN1, 0)
    pi.write(IN2, 1)
    pi.write(IN3, 0)
    pi.write(IN4, 1)


def backward():
    set_speed(DEFAULT_SPEED)
    pi.write(IN1, 1)
    pi.write(IN2, 0)
    pi.write(IN3, 1)
    pi.write(IN4, 0)


def right():
    set_speed(DEFAULT_SPEED)
    pi.write(IN1, 0)
    pi.write(IN2, 1)
    pi.write(IN3, 1)
    pi.write(IN4, 0)


def left():
    set_speed(DEFAULT_SPEED)
    pi.write(IN1, 1)
    pi.write(IN2, 0)
    pi.write(IN3, 0)
    pi.write(IN4, 1)


# --- Ultrasonic helpers ---

_last_trigger_time = 0.0


def setup_ultrasonic(pi):
    """Configure GPIO for the ultrasonic sensor."""
    pi.set_mode(TRIG, pigpio.OUTPUT)
    pi.set_mode(ECHO, pigpio.INPUT)
    pi.set_pull_up_down(ECHO, pigpio.PUD_DOWN)
    pi.write(TRIG, 0)
    time.sleep(0.05)


def single_distance_cm(pi, debug=False):
    """Return one distance reading or None on timeout/out-of-range."""
    global _last_trigger_time
    now = time.perf_counter()
    if now - _last_trigger_time < MIN_INTERVAL_S:
        time.sleep(MIN_INTERVAL_S - (now - _last_trigger_time))

    if pi.read(ECHO) == 1:
        if debug:
            print("[debug] ECHO already high before trigger; skipping measurement")
        return None

    pi.gpio_trigger(TRIG, 10, 1)  # 10 microsecond pulse
    _last_trigger_time = time.perf_counter()
    t0 = _last_trigger_time

    while pi.read(ECHO) == 0:
        if time.perf_counter() - t0 > ECHO_RISE_TIMEOUT_S:
            if debug:
                waited = (time.perf_counter() - t0) * 1000
                print(f"[debug] timeout waiting for ECHO rise after {waited:.2f} ms")
            return None

    start = time.perf_counter()

    while pi.read(ECHO) == 1:
        if time.perf_counter() - start > ECHO_FALL_TIMEOUT_S:
            if debug:
                waited = (time.perf_counter() - start) * 1000
                print(f"[debug] timeout waiting for ECHO fall after {waited:.2f} ms")
            return None

    duration = time.perf_counter() - start
    distance = (duration * 34300) / 2  # speed of sound 34300 cm/s

    if debug:
        print(f"[debug] pulse {duration * 1e6:.0f} us -> {distance:.1f} cm")

    if distance < MIN_DISTANCE_CM or distance > MAX_DISTANCE_CM:
        return None
    return distance


def sampled_distance(pi, debug=False):
    """Take multiple readings, return (median_distance|None, invalid_count, valid_count)."""
    samples = []
    invalid_count = 0

    for _ in range(SAMPLES_PER_REPORT):
        dist = single_distance_cm(pi, debug=debug)
        if dist is not None:
            samples.append(dist)
        else:
            invalid_count += 1
        time.sleep(MIN_INTERVAL_S)

    if invalid_count >= INVALID_SKIP_THRESHOLD:
        return None, invalid_count, len(samples)

    if not samples:
        return None, invalid_count, 0

    samples.sort()
    median = samples[len(samples) // 2]
    return median, invalid_count, len(samples)


def move_with_ultrasonic_guard(move_fn, duration_s, pi, debug=False):
    """Drive using move_fn but stop if obstacle is within STOP_THRESHOLD_CM."""
    move_fn()
    end_time = time.time() + duration_s
    while time.time() < end_time:
        loop_start = time.time()
        dist, invalid, valid = sampled_distance(pi, debug=debug)

        if invalid >= INVALID_SKIP_THRESHOLD:
            print(f"[ultra] skipped batch: {invalid}/{SAMPLES_PER_REPORT} invalid")
        elif dist is not None:
            print(f"[ultra] {dist:.1f} cm (valid {valid}/{SAMPLES_PER_REPORT}, invalid {invalid})")
            if dist <= STOP_THRESHOLD_CM:
                print(f"[ultra] obstacle within {STOP_THRESHOLD_CM} cm -> stopping motors")
                stop()
                return False
        else:
            print("[ultra] out of range (no valid samples)")

        elapsed = time.time() - loop_start
        if elapsed < PRINT_INTERVAL_S:
            time.sleep(PRINT_INTERVAL_S - elapsed)

    stop()
    return True


def cleanup(pi_ultra=None):
    stop()
    if pi_ultra is not None:
        pi_ultra.stop()


if __name__ == "__main__":
    debug_flag = "--debug" in sys.argv
    pi_ultra = pigpio.pi()
    if not pi_ultra.connected:
        print("pigpio daemon not running or not reachable.", file=sys.stderr)
        sys.exit(1)

    pi = pi_ultra

    # configure motor pins
    for pin in (IN1, IN2, IN3, IN4, ENA, ENB):
        pi.set_mode(pin, pigpio.OUTPUT)
        pi.write(pin, 0)

    if USE_PWM:
        pi.set_PWM_frequency(ENA, PWM_FREQ)
        pi.set_PWM_frequency(ENB, PWM_FREQ)
        set_speed(DEFAULT_SPEED)
    else:
        pi.write(ENA, 1)
        pi.write(ENB, 1)

    try:
        setup_ultrasonic(pi_ultra)

        print("Forward with ultrasonic guard")
        move_with_ultrasonic_guard(forward, duration_s=6, pi=pi_ultra, debug=debug_flag)

        print("Backward with ultrasonic guard")
        move_with_ultrasonic_guard(backward, duration_s=4, pi=pi_ultra, debug=debug_flag)

        print("Right turn with ultrasonic guard")
        move_with_ultrasonic_guard(right, duration_s=3, pi=pi_ultra, debug=debug_flag)

        print("Left turn with ultrasonic guard")
        move_with_ultrasonic_guard(left, duration_s=3, pi=pi_ultra, debug=debug_flag)

    except KeyboardInterrupt:
        print("\nStopped by user")
    finally:
        cleanup(pi_ultra)
