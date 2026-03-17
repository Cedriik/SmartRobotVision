# NOTE: Rotation timings should be calibrated for your robot and surface.

import sys
import time

import pigpio

# Motor pins (BCM) - L298N: OUT1/OUT2 = left, OUT3/OUT4 = right
IN1 = 17
IN2 = 27
IN3 = 22
IN4 = 23
ENA = 13
ENB = 12
PWM_HZ = 1000
PWM_DUTY = 70  # percent

# Ultrasonic pins (BCM)
TRIG = 18
ECHO = 19

# Thresholds (cm)
STOP_CM = 10.0
CLEAR_CM = 15.0

# Ultrasonic timing/noise controls
MIN_INTERVAL_S = 0.06
ECHO_RISE_TIMEOUT_S = 0.03
ECHO_FALL_TIMEOUT_S = 0.025
ECHO_GLITCH_US = 350
STUCK_HIGH_RECOVER_S = 0.01
LOCK_DISTANCE_CM = 1000.0
LOCK_TOLERANCE_CM = 120.0


def setup_motors(pi: pigpio.pi) -> None:
    pi.set_mode(IN1, pigpio.OUTPUT)
    pi.set_mode(IN2, pigpio.OUTPUT)
    pi.set_mode(IN3, pigpio.OUTPUT)
    pi.set_mode(IN4, pigpio.OUTPUT)
    pi.set_mode(ENA, pigpio.OUTPUT)
    pi.set_mode(ENB, pigpio.OUTPUT)

    pi.set_PWM_frequency(ENA, PWM_HZ)
    pi.set_PWM_frequency(ENB, PWM_HZ)
    pi.set_PWM_range(ENA, 100)
    pi.set_PWM_range(ENB, 100)
    pi.set_PWM_dutycycle(ENA, PWM_DUTY)
    pi.set_PWM_dutycycle(ENB, PWM_DUTY)


def stop(pi: pigpio.pi) -> None:
    pi.write(IN1, 0)
    pi.write(IN2, 0)
    pi.write(IN3, 0)
    pi.write(IN4, 0)


def forward(pi: pigpio.pi) -> None:
    pi.write(IN1, 0)
    pi.write(IN2, 1)
    pi.write(IN3, 0)
    pi.write(IN4, 1)


def backward(pi: pigpio.pi) -> None:
    pi.write(IN1, 1)
    pi.write(IN2, 0)
    pi.write(IN3, 1)
    pi.write(IN4, 0)


def right(pi: pigpio.pi) -> None:
    pi.write(IN1, 0)
    pi.write(IN2, 1)
    pi.write(IN3, 1)
    pi.write(IN4, 0)


def left(pi: pigpio.pi) -> None:
    pi.write(IN1, 1)
    pi.write(IN2, 0)
    pi.write(IN3, 0)
    pi.write(IN4, 1)


def setup_ultrasonic(pi: pigpio.pi) -> None:
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

    if pi.read(ECHO) == 1:
        t_h = time.perf_counter()
        while pi.read(ECHO) == 1 and (time.perf_counter() - t_h) < STUCK_HIGH_RECOVER_S:
            pass
        if pi.read(ECHO) == 1:
            if debug:
                print("[debug] ECHO stuck high pre-trigger")
            return None

    pi.gpio_trigger(TRIG, 10, 1)
    _last_trigger_time = time.perf_counter()
    t0 = _last_trigger_time

    while pi.read(ECHO) == 0:
        if time.perf_counter() - t0 > ECHO_RISE_TIMEOUT_S:
            if debug:
                print("[debug] timeout waiting ECHO rise")
            return None

    start = time.perf_counter()
    while pi.read(ECHO) == 1:
        if time.perf_counter() - start > ECHO_FALL_TIMEOUT_S:
            if debug:
                print("[debug] timeout waiting ECHO fall (long pulse)")
            return None

    duration = time.perf_counter() - start
    distance = (duration * 34300) / 2

    if abs(distance - LOCK_DISTANCE_CM) <= LOCK_TOLERANCE_CM:
        return None
    if distance < 2 or distance > 400:
        return None
    return distance


def drive_turn(direction: str, duration_s: float, pi: pigpio.pi) -> None:
    if direction == "left":
        left(pi)
    elif direction == "right":
        right(pi)
    else:
        raise ValueError(f"Unknown turn direction: {direction}")

    start = time.perf_counter()
    while time.perf_counter() - start < duration_s:
        time.sleep(0.02)
    stop(pi)


def execute_forward_with_us(pi: pigpio.pi, debug: bool) -> None:
    forward(pi)
    while True:
        dist = get_distance_cm(pi, debug=debug)
        if dist is not None and dist < STOP_CM:
            stop(pi)
            if debug:
                print(f"[debug] blocked at {dist:.1f} cm -> stop forward")
            break
        time.sleep(0.02)


def execute_step(direction: str, duration_s: float, pi: pigpio.pi, debug: bool) -> None:
    direction = direction.strip().lower()
    print(f"{direction} for {duration_s:.1f}s")

    if direction in ("forward", "straight"):
        execute_forward_with_us(pi, debug)
        return

    if direction == "backward":
        backward(pi)
    elif direction == "left":
        left(pi)
    elif direction == "right":
        right(pi)
    elif direction == "stop":
        stop(pi)
    else:
        raise ValueError(f"Unknown direction: {direction}")

    start = time.perf_counter()
    while time.perf_counter() - start < duration_s:
        time.sleep(0.02)
    stop(pi)


def main() -> None:
    debug = "--debug" in sys.argv
    pi = pigpio.pi()
    if not pi.connected:
        print("pigpio daemon not running or not reachable.", file=sys.stderr)
        sys.exit(1)

    setup_motors(pi)
    setup_ultrasonic(pi)

    path = [
        ("straight", 1.0),
        ("right", 1.0),
        ("forward", 1.0),
        ("right", 1.0),
        ("forward", 1.0),
        ("right", 1.0),
        ("forward", 1.0),
        ("left", 1.0),
        ("forward", 1.0),
        ("left", 1.0),
        ("forward", 1.0),
        ("left", 1.0),
        ("forward", 1.0),
    ]

    try:
        for name, dur in path:
            execute_step(name, dur, pi, debug=debug)
            time.sleep(0.2)
    except KeyboardInterrupt:
        print("\nStopped by user")
    finally:
        stop(pi)
        pi.write(TRIG, 0)
        pi.set_glitch_filter(ECHO, 0)
        pi.stop()


if __name__ == "__main__":
    main()
