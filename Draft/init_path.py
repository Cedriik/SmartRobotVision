import sys
import time
import math
from statistics import median

import pigpio

# Motor pins (BCM) - L298N: OUT1/OUT2 = left, OUT3/OUT4 = right
IN1 = 17
IN2 = 27
IN3 = 22
IN4 = 23
ENA = 13
ENB = 12
PWM_HZ = 1000

# PWM tuning
DEFAULT_PWM_DUTY = 60
BASE_CRUISE_PWM_DUTY = 42
BASE_TIGHT_PWM_DUTY = 30
BASE_CRAWL_PWM_DUTY = 24
TURN_PWM_DUTY = 80
MIN_DRIVE_PWM_DUTY = 20
MAX_DRIVE_PWM_DUTY = 60
LEFT_MOTOR_TRIM_PWM = 3.0
RIGHT_MOTOR_TRIM_PWM = 0.0

# Ultrasonic pins (BCM)
FRONT_TRIG = 18
FRONT_ECHO = 19
LEFT_TRIG = 24
LEFT_ECHO = 25
RIGHT_TRIG = 4
RIGHT_ECHO = 26

# Distance thresholds (cm)
FRONT_STOP_CM = 10.0
FRONT_SLOW_CM = 18.0
TURN_OPENING_CLEAR_CM = 10.0
LEFT_REFERENCE_CM = 8.6
RIGHT_REFERENCE_CM = 7.8
PASSAGE_TIGHT_SIDE_CM = 5.0
SIDE_HARD_LIMIT_CM = 2.5
SIDE_PID_ACTIVE_MAX_CM = 20.0

# Ultrasonic timing/noise controls
MIN_INTERVAL_S = 0.03
ECHO_RISE_TIMEOUT_S = 0.03
ECHO_FALL_TIMEOUT_S = 0.025
ECHO_GLITCH_US = 350
STUCK_HIGH_RECOVER_S = 0.01
LOCK_DISTANCE_CM = 1000.0
LOCK_TOLERANCE_CM = 120.0
MIN_DISTANCE_CM = 2.0
MAX_DISTANCE_CM = 400.0

# Averaging / motion controls
SIDE_SAMPLE_SECONDS = 1.0
TURN_SECONDS = 1.0
RESUME_PAUSE_S = 0.20
PRE_TURN_VERIFY_SECONDS = 0.8
POST_TURN_STABILIZE_SECONDS = 0.8

# Side-centering PID controls
PID_LOOP_DT_S = 0.025
PID_FILTER_ALPHA = 0.30
PID_KP = 2.5
PID_KI = 0.10
PID_KD = 0.12
PID_DEADBAND_CM = 0.25
PID_INTEGRAL_LIMIT = 10.0
PID_MAX_LEFT_ADJUST = 10.0
PID_MAX_RIGHT_ADJUST = 24.0
RIGHT_TURN_GAIN_BOOST = 1.12
RIGHT_REFERENCE_WEIGHT = 1.45
LEFT_REFERENCE_WEIGHT = 0.90
POSITIVE_ERROR_EXP_GAIN = 0.26
NEGATIVE_ERROR_EXP_GAIN = 0.06
MAX_ERROR_EXP_SCALE = 3.0
POSITIVE_INTEGRAL_GAIN = 2.6
RIGHT_WALL_KP_MULTIPLIER = 1.25
RIGHT_WALL_RIGHT_BOOST = 1.45
RIGHT_WALL_MIN_ADJUST_PWM = 7.0
LONG_RANGE_FRONT_CM = 60.0
LONG_RANGE_KP_MULTIPLIER = 1.80
LONG_RANGE_KD_MULTIPLIER = 1.40
LONG_RANGE_RIGHT_BOOST = 1.35
LONG_RANGE_MIN_ADJUST_PWM = 6.0
FRONT_COARSE_VERIFY_CM = 14.0
COARSE_RECOVERY_ENTER_ERROR = 8.0
COARSE_RECOVERY_EXIT_ERROR = 3.5
COARSE_RECOVERY_ENTER_DELTA = 4.0
COARSE_RECOVERY_EXIT_DELTA = 2.0
COARSE_KP_MULTIPLIER = 1.45
COARSE_KI_MULTIPLIER = 1.60
COARSE_KD_MULTIPLIER = 1.30
COARSE_RIGHT_BOOST = 1.25
COARSE_MIN_ADJUST_PWM = 8.0
PID_DEBUG_PRINT_INTERVAL_S = 0.25

# Path definition
PATH_STEPS = [
    ("straight", None),
    ("right", 1.0),
    ("forward", None),
    ("right", 1.0),
    ("forward", None),
    ("right", 1.0),
    ("forward", None),
    ("left", 1.0),
    ("forward", None),
    ("left", 1.0),
    ("forward", None),
    ("left", 1.0),
    ("forward", None),
]

SENSORS = {
    "front": {"trig": FRONT_TRIG, "echo": FRONT_ECHO, "last_trigger": 0.0},
    "left": {"trig": LEFT_TRIG, "echo": LEFT_ECHO, "last_trigger": 0.0},
    "right": {"trig": RIGHT_TRIG, "echo": RIGHT_ECHO, "last_trigger": 0.0},
}


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


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
    pi.set_PWM_dutycycle(ENA, 0)
    pi.set_PWM_dutycycle(ENB, 0)


def apply_drive(pi: pigpio.pi, left_speed: float, right_speed: float) -> None:
    left_speed = clamp(left_speed, -100.0, 100.0)
    right_speed = clamp(right_speed, -100.0, 100.0)

    if left_speed > 0:
        pi.write(IN1, 0)
        pi.write(IN2, 1)
    elif left_speed < 0:
        pi.write(IN1, 1)
        pi.write(IN2, 0)
    else:
        pi.write(IN1, 0)
        pi.write(IN2, 0)

    if right_speed > 0:
        pi.write(IN3, 0)
        pi.write(IN4, 1)
    elif right_speed < 0:
        pi.write(IN3, 1)
        pi.write(IN4, 0)
    else:
        pi.write(IN3, 0)
        pi.write(IN4, 0)

    pi.set_PWM_dutycycle(ENA, int(abs(left_speed)))
    pi.set_PWM_dutycycle(ENB, int(abs(right_speed)))


def stop(pi: pigpio.pi) -> None:
    apply_drive(pi, 0.0, 0.0)


def forward(pi: pigpio.pi, duty: float = DEFAULT_PWM_DUTY) -> None:
    apply_drive(pi, duty, duty)


def backward(pi: pigpio.pi, duty: float = DEFAULT_PWM_DUTY) -> None:
    apply_drive(pi, -duty, -duty)


def right(pi: pigpio.pi, duty: float = TURN_PWM_DUTY) -> None:
    apply_drive(pi, duty, -duty)


def left(pi: pigpio.pi, duty: float = TURN_PWM_DUTY) -> None:
    apply_drive(pi, -duty, duty)


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
        time.sleep(0.01)

    if not samples:
        return None
    return median(samples)


def report_side_status(left_avg: float | None, right_avg: float | None) -> None:
    left_text = "invalid" if left_avg is None else f"{left_avg:.1f} cm"
    right_text = "invalid" if right_avg is None else f"{right_avg:.1f} cm"
    print(f"Side verification -> left: {left_text}, right: {right_text}")


def report_single_sensor(label: str, avg: float | None) -> None:
    text = "invalid" if avg is None else f"{avg:.1f} cm"
    print(f"{label} -> {text}")


def format_distance(value: float | None) -> str:
    if value is None:
        return "invalid"
    return f"{value:.1f} cm"


def choose_turn_direction(left_avg: float | None, right_avg: float | None) -> str | None:
    left_clear = left_avg is not None and left_avg >= TURN_OPENING_CLEAR_CM
    right_clear = right_avg is not None and right_avg >= TURN_OPENING_CLEAR_CM

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
        left(pi, TURN_PWM_DUTY)
    elif direction == "right":
        right(pi, TURN_PWM_DUTY)
    else:
        raise ValueError(f"Unknown turn direction: {direction}")

    start = time.perf_counter()
    while time.perf_counter() - start < duration_s:
        time.sleep(0.02)
    stop(pi)


def verify_side_before_turn(direction: str, pi: pigpio.pi, debug: bool) -> None:
    side_avg = average_distance(pi, direction, PRE_TURN_VERIFY_SECONDS, debug=debug)
    report_single_sensor(f"Pre-turn {direction} check", side_avg)
    if side_avg is not None and side_avg < TURN_OPENING_CLEAR_CM:
        print(
            f"Warning: {direction} side is below {TURN_OPENING_CLEAR_CM:.1f} cm, "
            "but continuing because the path is predetermined."
        )


def stabilize_after_turn(pi: pigpio.pi, debug: bool) -> None:
    front_avg = average_distance(pi, "front", POST_TURN_STABILIZE_SECONDS, debug=debug)
    report_single_sensor("Post-turn front check", front_avg)


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


def smooth_distance(previous: float | None, current: float | None) -> float | None:
    if current is None:
        return previous
    if previous is None:
        return current
    return (PID_FILTER_ALPHA * current) + ((1.0 - PID_FILTER_ALPHA) * previous)


def apply_deadband(error: float) -> float:
    if abs(error) <= PID_DEADBAND_CM:
        return 0.0
    return error


def pid_visible_distance(distance_cm: float | None) -> float | None:
    if distance_cm is None:
        return None
    if distance_cm > SIDE_PID_ACTIVE_MAX_CM:
        return None
    return distance_cm


def scale_balance_error(error: float) -> float:
    if error == 0.0:
        return 0.0
    gain = POSITIVE_ERROR_EXP_GAIN if error > 0.0 else NEGATIVE_ERROR_EXP_GAIN
    scale = min(math.exp(abs(error) * gain), MAX_ERROR_EXP_SCALE)
    return error * scale


def compute_passage_error(left_cm: float | None, right_cm: float | None) -> tuple[float | None, str]:
    left_pid = pid_visible_distance(left_cm)
    right_pid = pid_visible_distance(right_cm)

    if left_pid is not None and right_pid is not None:
        raw_error = (
            (right_pid - RIGHT_REFERENCE_CM) * RIGHT_REFERENCE_WEIGHT
            - (left_pid - LEFT_REFERENCE_CM) * LEFT_REFERENCE_WEIGHT
        )
        return scale_balance_error(apply_deadband(raw_error)), "center"
    if right_pid is not None:
        raw_error = (right_pid - RIGHT_REFERENCE_CM) * RIGHT_REFERENCE_WEIGHT
        return scale_balance_error(apply_deadband(raw_error)), "right-wall"
    if left_pid is not None:
        raw_error = (LEFT_REFERENCE_CM - left_pid) * LEFT_REFERENCE_WEIGHT
        return scale_balance_error(apply_deadband(raw_error)), "left-wall"
    return 0.0, "trim-only"


def compute_visible_side_delta(left_cm: float | None, right_cm: float | None) -> float:
    left_pid = pid_visible_distance(left_cm)
    right_pid = pid_visible_distance(right_cm)
    if left_pid is None or right_pid is None:
        return 0.0
    return right_pid - left_pid


def choose_forward_base_duty(
    front_cm: float | None,
    left_cm: float | None,
    right_cm: float | None,
) -> float:
    side_values = [dist for dist in (left_cm, right_cm) if dist is not None]
    nearest_side = min(side_values) if side_values else None
    base_duty = BASE_CRUISE_PWM_DUTY

    if nearest_side is not None:
        if nearest_side <= PASSAGE_TIGHT_SIDE_CM + 0.5:
            base_duty = BASE_CRAWL_PWM_DUTY
        elif nearest_side <= PASSAGE_TIGHT_SIDE_CM + 2.0:
            base_duty = BASE_TIGHT_PWM_DUTY
        elif nearest_side <= 12.0:
            base_duty = (BASE_CRUISE_PWM_DUTY + BASE_TIGHT_PWM_DUTY) / 2

    if front_cm is not None and front_cm < FRONT_SLOW_CM:
        base_duty = min(base_duty, BASE_TIGHT_PWM_DUTY)

    return base_duty


def get_pid_profile(
    front_cm: float | None,
    mode: str,
    coarse_recovery: bool,
) -> tuple[float, float, float, float, float]:
    kp = PID_KP
    ki = PID_KI
    kd = PID_KD
    right_boost = RIGHT_TURN_GAIN_BOOST
    min_adjust = 0.0

    if mode == "right-wall":
        kp *= RIGHT_WALL_KP_MULTIPLIER
        right_boost *= RIGHT_WALL_RIGHT_BOOST
        min_adjust = max(min_adjust, RIGHT_WALL_MIN_ADJUST_PWM)

    if front_cm is not None and front_cm >= LONG_RANGE_FRONT_CM:
        kp *= LONG_RANGE_KP_MULTIPLIER
        kd *= LONG_RANGE_KD_MULTIPLIER
        right_boost *= LONG_RANGE_RIGHT_BOOST
        min_adjust = max(min_adjust, LONG_RANGE_MIN_ADJUST_PWM)

    if coarse_recovery:
        kp *= COARSE_KP_MULTIPLIER
        ki *= COARSE_KI_MULTIPLIER
        kd *= COARSE_KD_MULTIPLIER
        right_boost *= COARSE_RIGHT_BOOST
        min_adjust = max(min_adjust, COARSE_MIN_ADJUST_PWM)

    return kp, ki, kd, right_boost, min_adjust


def run_forward_pid(pi: pigpio.pi, duration_s: float | None, debug: bool) -> str:
    front_filtered: float | None = None
    left_filtered: float | None = None
    right_filtered: float | None = None

    integral = 0.0
    previous_error = 0.0
    coarse_recovery = False
    loop_started_at = time.perf_counter()
    motion_started_at = loop_started_at
    last_debug_at = 0.0

    while True:
        if duration_s is not None and (time.perf_counter() - motion_started_at) >= duration_s:
            stop(pi)
            return "completed"

        front_filtered = smooth_distance(front_filtered, get_distance_cm(pi, "front", debug=debug))
        left_filtered = smooth_distance(left_filtered, get_distance_cm(pi, "left", debug=debug))
        right_filtered = smooth_distance(right_filtered, get_distance_cm(pi, "right", debug=debug))

        if front_filtered is not None and front_filtered < FRONT_STOP_CM:
            stop(pi)
            print(f"Front blocked at {front_filtered:.1f} cm")
            return "front_blocked"

        side_values = [dist for dist in (left_filtered, right_filtered) if dist is not None]
        if side_values and min(side_values) < SIDE_HARD_LIMIT_CM:
            stop(pi)
            print(
                "Side clearance dropped below "
                f"{SIDE_HARD_LIMIT_CM:.1f} cm "
                f"(left={format_distance(left_filtered)}, right={format_distance(right_filtered)})."
            )
            return "side_limit"

        error, mode = compute_passage_error(left_filtered, right_filtered)
        if error is None:
            stop(pi)
            print("Lost both side-wall readings. Robot stopped for safety.")
            return "side_lost"

        side_delta = compute_visible_side_delta(left_filtered, right_filtered)
        front_allows_coarse = front_filtered is None or front_filtered >= FRONT_COARSE_VERIFY_CM
        if coarse_recovery:
            if (
                abs(error) <= COARSE_RECOVERY_EXIT_ERROR
                and abs(side_delta) <= COARSE_RECOVERY_EXIT_DELTA
            ) or not front_allows_coarse:
                coarse_recovery = False
        elif front_allows_coarse and (
            abs(error) >= COARSE_RECOVERY_ENTER_ERROR
            or abs(side_delta) >= COARSE_RECOVERY_ENTER_DELTA
        ):
            coarse_recovery = True

        now = time.perf_counter()
        dt = max(now - loop_started_at, 0.001)
        loop_started_at = now

        if mode == "trim-only":
            integral = 0.0
            previous_error = 0.0
        elif error == 0.0 or (previous_error != 0.0 and (error > 0.0) != (previous_error > 0.0)):
            integral = 0.0
        integral_input = error * (POSITIVE_INTEGRAL_GAIN if error > 0.0 else 1.0)
        integral = clamp(integral + (integral_input * dt), -PID_INTEGRAL_LIMIT, PID_INTEGRAL_LIMIT)
        derivative = 0.0 if mode == "trim-only" else (error - previous_error) / dt
        kp, ki, kd, right_boost, min_adjust = get_pid_profile(front_filtered, mode, coarse_recovery)
        control_signal = 0.0 if mode == "trim-only" else (
            (kp * error) + (ki * integral) + (kd * derivative)
        )
        raw_adjust = 0.0 if mode == "trim-only" else clamp(
            control_signal,
            -PID_MAX_LEFT_ADJUST,
            PID_MAX_RIGHT_ADJUST,
        )
        adjust = raw_adjust
        if adjust > 0:
            adjust = clamp(adjust * right_boost, -PID_MAX_LEFT_ADJUST, PID_MAX_RIGHT_ADJUST)
        if min_adjust > 0.0 and error != 0.0 and abs(adjust) < min_adjust:
            adjust = min_adjust if adjust >= 0.0 else -min_adjust

        base_duty = choose_forward_base_duty(front_filtered, left_filtered, right_filtered)
        base_left_duty = base_duty + LEFT_MOTOR_TRIM_PWM
        base_right_duty = base_duty + RIGHT_MOTOR_TRIM_PWM
        left_duty = clamp(base_left_duty + adjust, MIN_DRIVE_PWM_DUTY, MAX_DRIVE_PWM_DUTY)
        right_duty = clamp(base_right_duty - adjust, MIN_DRIVE_PWM_DUTY, MAX_DRIVE_PWM_DUTY)
        apply_drive(pi, left_duty, right_duty)
        previous_error = error

        if debug and (now - last_debug_at) >= PID_DEBUG_PRINT_INTERVAL_S:
            print(
                "[pid] "
                f"mode={mode}{'+coarse' if coarse_recovery else ''} "
                f"front={format_distance(front_filtered)} "
                f"left={format_distance(left_filtered)} "
                f"right={format_distance(right_filtered)} "
                f"err={error:.2f} delta={side_delta:.2f} raw={control_signal:.1f} adj={adjust:.1f} "
                f"base=({base_left_duty:.0f},{base_right_duty:.0f}) "
                f"pwm=({left_duty:.0f},{right_duty:.0f})"
            )
            last_debug_at = now

        elapsed = time.perf_counter() - now
        sleep_for = PID_LOOP_DT_S - elapsed
        if sleep_for > 0:
            time.sleep(sleep_for)


def execute_forward_with_all_us(duration_s: float, pi: pigpio.pi, debug: bool) -> None:
    run_forward_pid(pi, duration_s, debug=debug)


def execute_forward_until_blocked(pi: pigpio.pi, debug: bool) -> None:
    run_forward_pid(pi, None, debug=debug)


def execute_step(direction: str, duration_s: float | None, pi: pigpio.pi, debug: bool) -> None:
    direction = direction.strip().lower()

    if direction in ("forward", "straight"):
        if duration_s is None:
            print(f"{direction} with side PID until front ultrasonic stop")
            execute_forward_until_blocked(pi, debug)
        else:
            print(f"{direction} with side PID for {duration_s:.1f}s")
            execute_forward_with_all_us(duration_s, pi, debug)
        return

    if duration_s is None:
        raise ValueError(f"Timed direction requires a duration: {direction}")

    print(f"{direction} for {duration_s:.1f}s")

    if direction == "backward":
        backward(pi, BASE_TIGHT_PWM_DUTY)
    elif direction == "left":
        left(pi, TURN_PWM_DUTY)
    elif direction == "right":
        right(pi, TURN_PWM_DUTY)
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

    print("init_path.py ready. Predetermined path is active with side-PID forward control.")

    try:
        for name, dur in PATH_STEPS:
            execute_step(name, dur, pi, debug=debug)
            time.sleep(RESUME_PAUSE_S)
    except KeyboardInterrupt:
        print("\nStopped by user")
    finally:
        stop(pi)
        cleanup_ultrasonic(pi)
        pi.stop()


if __name__ == "__main__":
    main()
