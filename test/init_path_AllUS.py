import sys
import time
from statistics import mean

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
FRONT_TRIG = 18
FRONT_ECHO = 19
LEFT_TRIG = 24
LEFT_ECHO = 25
RIGHT_TRIG = 4
RIGHT_ECHO = 26

# Distance thresholds (cm)
FRONT_STOP_CM = 10.0
SIDE_CLEAR_CM = 20.0

# Ultrasonic timing/noise controls
MIN_INTERVAL_S = 0.06
ECHO_RISE_TIMEOUT_S = 0.03
ECHO_FALL_TIMEOUT_S = 0.025
ECHO_GLITCH_US = 350
STUCK_HIGH_RECOVER_S = 0.01
LOCK_DISTANCE_CM = 1000.0
LOCK_TOLERANCE_CM = 120.0
MIN_DISTANCE_CM = 2.0
MAX_DISTANCE_CM = 400.0

# Averaging / motion controls
SIDE_SAMPLE_SECONDS = 2.0
TURN_SECONDS = 1.0
RESUME_PAUSE_S = 0.20
TEST_FORWARD_SECONDS = 2.0

SENSORS = {
    "front": {"trig": FRONT_TRIG, "echo": FRONT_ECHO, "last_trigger": 0.0},
    "left": {"trig": LEFT_TRIG, "echo": LEFT_ECHO, "last_trigger": 0.0},
    "right": {"trig": RIGHT_TRIG, "echo": RIGHT_ECHO, "last_trigger": 0.0},
}


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
    for sensor in SENSORS.values():
        pi.set_mode(sensor["trig"], pigpio.OUTPUT)
        pi.set_mode(sensor["echo"], pigpio.INPUT)
        pi.set_pull_up_down(sensor["echo"], pigpio.PUD_DOWN)
        pi.set_glitch_filter(sensor["echo"], ECHO_GLITCH_US)
        pi.write(sensor["trig"], 0)
    time.sleep(0.08)


def cleanup_ultrasonic(pi: pigpio.pi) -> None:
    for sensor in SENSORS.values():
        pi.write(sensor["trig"], 0)
        pi.set_glitch_filter(sensor["echo"], 0)


def get_distance_cm(pi: pigpio.pi, sensor_name: str, debug: bool = False) -> float | None:
    sensor = SENSORS[sensor_name]
    now = time.perf_counter()
    since_last = now - sensor["last_trigger"]
    if since_last < MIN_INTERVAL_S:
        time.sleep(MIN_INTERVAL_S - since_last)

    echo = sensor["echo"]
    trig = sensor["trig"]

    if pi.read(echo) == 1:
        t_h = time.perf_counter()
        while pi.read(echo) == 1 and (time.perf_counter() - t_h) < STUCK_HIGH_RECOVER_S:
            pass
        if pi.read(echo) == 1:
            if debug:
                print(f"[debug] {sensor_name} ECHO stuck high pre-trigger")
            return None

    pi.gpio_trigger(trig, 10, 1)
    sensor["last_trigger"] = time.perf_counter()
    t0 = sensor["last_trigger"]

    while pi.read(echo) == 0:
        if time.perf_counter() - t0 > ECHO_RISE_TIMEOUT_S:
            if debug:
                print(f"[debug] {sensor_name} timeout waiting ECHO rise")
            return None

    start = time.perf_counter()
    while pi.read(echo) == 1:
        if time.perf_counter() - start > ECHO_FALL_TIMEOUT_S:
            if debug:
                print(f"[debug] {sensor_name} timeout waiting ECHO fall")
            return None

    duration = time.perf_counter() - start
    distance = (duration * 34300) / 2

    if abs(distance - LOCK_DISTANCE_CM) <= LOCK_TOLERANCE_CM:
        return None
    if distance < MIN_DISTANCE_CM or distance > MAX_DISTANCE_CM:
        return None
    return distance


def average_distance(pi: pigpio.pi, sensor_name: str, sample_seconds: float, debug: bool = False) -> float | None:
    samples: list[float] = []
    start = time.perf_counter()

    while time.perf_counter() - start < sample_seconds:
        dist = get_distance_cm(pi, sensor_name, debug=debug)
        if dist is not None:
            samples.append(dist)
        time.sleep(0.02)

    if not samples:
        return None
    return mean(samples)


def report_side_status(left_avg: float | None, right_avg: float | None) -> None:
    left_text = "invalid" if left_avg is None else f"{left_avg:.1f} cm"
    right_text = "invalid" if right_avg is None else f"{right_avg:.1f} cm"
    print(f"Side verification -> left: {left_text}, right: {right_text}")


def choose_turn_direction(left_avg: float | None, right_avg: float | None) -> str | None:
    left_clear = left_avg is not None and left_avg >= SIDE_CLEAR_CM
    right_clear = right_avg is not None and right_avg >= SIDE_CLEAR_CM

    if left_clear and right_clear:
        if left_avg >= right_avg:
            return "left"
        return "right"
    if left_clear:
        return "left"
    if right_clear:
        return "right"
    return None


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


def verify_and_turn(pi: pigpio.pi, debug: bool) -> bool:
    left_avg = average_distance(pi, "left", SIDE_SAMPLE_SECONDS, debug=debug)
    right_avg = average_distance(pi, "right", SIDE_SAMPLE_SECONDS, debug=debug)
    report_side_status(left_avg, right_avg)

    direction = choose_turn_direction(left_avg, right_avg)
    if direction is None:
        print("Both sides obstructed or invalid. Robot remains stopped.")
        return False

    print(f"Turning {direction} based on side clearance")
    drive_turn(direction, TURN_SECONDS, pi)
    return True


def execute_forward_with_all_us(duration_s: float, pi: pigpio.pi, debug: bool) -> None:
    forward(pi)
    start = time.perf_counter()

    while time.perf_counter() - start < duration_s:
        dist = get_distance_cm(pi, "front", debug=debug)
        if dist is not None and dist < FRONT_STOP_CM:
            stop(pi)
            print(f"Front blocked at {dist:.1f} cm")
            turned = verify_and_turn(pi, debug=debug)
            if not turned:
                return
            time.sleep(RESUME_PAUSE_S)
            forward(pi)
        time.sleep(0.02)

    stop(pi)


def execute_step(direction: str, duration_s: float, pi: pigpio.pi, debug: bool) -> None:
    direction = direction.strip().lower()
    print(f"{direction} for {duration_s:.1f}s")

    if direction in ("forward", "straight"):
        execute_forward_with_all_us(duration_s, pi, debug)
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

    # Predetermined path disabled for now to avoid conflicts while validating
    # the all-ultrasonic turn-decision logic.
    #
    # path = [
    #     ("straight", 1.0),
    #     ("right", 1.0),
    #     ("forward", 1.0),
    #     ("right", 1.0),
    #     ("forward", 1.0),
    #     ("right", 1.0),
    #     ("forward", 1.0),
    #     ("left", 1.0),
    #     ("forward", 1.0),
    #     ("left", 1.0),
    #     ("forward", 1.0),
    #     ("left", 1.0),
    #     ("forward", 1.0),
    # ]
    #
    # for name, dur in path:
    #     execute_step(name, dur, pi, debug=debug)
    #     time.sleep(0.2)

    print(
        "init_path_AllUS.py ready. Predetermined path is commented out in main(). "
        f"Running guarded forward test for {TEST_FORWARD_SECONDS:.1f}s."
    )

    try:
        execute_forward_with_all_us(TEST_FORWARD_SECONDS, pi, debug)
    except KeyboardInterrupt:
        print("\nStopped by user")
    finally:
        stop(pi)
        cleanup_ultrasonic(pi)
        pi.stop()


if __name__ == "__main__":
    main()
