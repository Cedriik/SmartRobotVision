from __future__ import annotations

import math
import os
import signal
import sys
import threading
import time
from statistics import median

import cv2
import numpy as np
import pigpio
from flask import Flask, Response, render_template_string

app = Flask(__name__)

# Motor pins (BCM) - L298N
IN1 = 17
IN2 = 27
IN3 = 22
IN4 = 23
ENA = 13
ENB = 12
PWM_HZ = 1000

# Servo
SERVO_PIN = 16
SERVO_HOME_US = 1500
SERVO_ALERT_US = 2150
SERVO_SETTLE_S = 0.25

# PWM tuning
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
FRONT_EMERGENCY_STOP_CM = 6.0
FRONT_SLOW_CM = 18.0
TURN_OPENING_CLEAR_CM = 10.0
LEFT_REFERENCE_CM = 8.6
RIGHT_REFERENCE_CM = 7.8
PASSAGE_TIGHT_SIDE_CM = 5.0
SIDE_HARD_LIMIT_CM = 2.5
SIDE_PID_ACTIVE_MAX_CM = 20.0

# Ultrasonic timing / noise controls
MIN_INTERVAL_S = 0.03
ECHO_RISE_TIMEOUT_S = 0.03
ECHO_FALL_TIMEOUT_S = 0.025
ECHO_GLITCH_US = 350
STUCK_HIGH_RECOVER_S = 0.01
LOCK_DISTANCE_CM = 1000.0
LOCK_TOLERANCE_CM = 120.0
MIN_DISTANCE_CM = 2.0
MAX_DISTANCE_CM = 400.0

# Motion timing
SIDE_SAMPLE_SECONDS = 0.8
TURN_SECONDS = 1.0
POST_TURN_STABILIZE_SECONDS = 0.6
PID_LOOP_DT_S = 0.025
PID_FILTER_ALPHA = 0.30
RESUME_PAUSE_S = 0.15

# Side-centering PID controls
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

# Vision
CAMERA_INDEX = 0
FRAME_SLEEP_S = 0.02
COLOR_LOCK_FRAMES = 8
BOX_HOLD_FRAMES = 3
CAMERA_BLOCK_CONFIRM_FRAMES = 3

COLORS = {
    "Yellow": [(np.array([18, 120, 120]), np.array([38, 255, 255]))],
    "Green": [(np.array([55, 120, 60]), np.array([85, 255, 255]))],
}

COLOR_BOXES = {
    "Yellow": (0, 255, 255),
    "Green": (0, 255, 0),
}

COLOR_MIN_AREA = {
    "Yellow": 300,
    "Green": 250,
}

SENSORS = {
    "front": {"trig": FRONT_TRIG, "echo": FRONT_ECHO, "last_trigger": 0.0},
    "left": {"trig": LEFT_TRIG, "echo": LEFT_ECHO, "last_trigger": 0.0},
    "right": {"trig": RIGHT_TRIG, "echo": RIGHT_ECHO, "last_trigger": 0.0},
}

HTML = """
<html>
    <head><title>Robot Vision Integrated</title>
    <style>
        body { background: #000; color: #0f0; font-family: monospace; text-align: center; margin: 0; }
        .container { display: flex; flex-direction: column; align-items: center; justify-content: center; height: 100vh; }
        img { border: 2px solid #222; max-width: 90%; box-shadow: 0 0 20px #040; }
        .hud { margin-bottom: 5px; font-size: 11px; letter-spacing: 1px; }
    </style></head>
    <body>
        <div class="container">
            <div class="hud">SYSTEM: ROBOT // CAMERA + ULTRASONICS + PID</div>
            <img src="/video_feed">
            <br><a href="/stop_server" style="color:#600; text-decoration:none; margin-top:10px; display:block;">[ EMERGENCY SHUTDOWN ]</a>
        </div>
    </body>
</html>
"""

state_lock = threading.Lock()
shutdown_event = threading.Event()
camera_ready_event = threading.Event()

latest_jpeg: bytes | None = None
latest_frame_ts = 0.0
latest_locked_color = "None"
yellow_pause_latched = False
camera_blocked_confirmed = False
camera_blocked_frames = 0
motion_status = "standby"
status_detail = "Initializing"
sensor_snapshot = {"front": "invalid", "left": "invalid", "right": "invalid"}

detection_memory = {
    name: {"box": None, "frames": 0, "consecutive": 0, "area": 0.0}
    for name in COLORS
}


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def set_motion_status(status: str, detail: str | None = None) -> None:
    global motion_status, status_detail
    with state_lock:
        changed = status != motion_status or (detail is not None and detail != status_detail)
        motion_status = status
        if detail is not None:
            status_detail = detail
        detail_text = status_detail
    if changed:
        print(f"[motion] {status}: {detail_text}")


def set_sensor_snapshot(front: float | None, left: float | None, right: float | None) -> None:
    with state_lock:
        sensor_snapshot["front"] = format_distance(front)
        sensor_snapshot["left"] = format_distance(left)
        sensor_snapshot["right"] = format_distance(right)


def set_yellow_pause(active: bool) -> None:
    global yellow_pause_latched
    changed = False
    with state_lock:
        if yellow_pause_latched != active:
            yellow_pause_latched = active
            changed = True
    if changed:
        if active:
            print("[vision] Yellow locked. Robot paused until Green is detected.")
            set_motion_status("yellow-pause", "Yellow detected. Waiting for Green.")
        else:
            print("[vision] Green locked. Yellow pause cleared.")
            set_motion_status("resume-ready", "Green detected. Resuming robot.")


def is_yellow_paused() -> bool:
    with state_lock:
        return yellow_pause_latched


def set_locked_color(name: str) -> None:
    global latest_locked_color
    with state_lock:
        latest_locked_color = name


def setup_motors(pi: pigpio.pi) -> None:
    for pin in (IN1, IN2, IN3, IN4, ENA, ENB):
        pi.set_mode(pin, pigpio.OUTPUT)
    pi.set_PWM_frequency(ENA, PWM_HZ)
    pi.set_PWM_frequency(ENB, PWM_HZ)
    pi.set_PWM_range(ENA, 100)
    pi.set_PWM_range(ENB, 100)
    pi.set_PWM_dutycycle(ENA, 0)
    pi.set_PWM_dutycycle(ENB, 0)


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


def setup_servo(pi: pigpio.pi) -> None:
    pi.set_mode(SERVO_PIN, pigpio.OUTPUT)
    set_servo_home(pi)


def set_servo_home(pi: pigpio.pi) -> None:
    pi.set_servo_pulsewidth(SERVO_PIN, SERVO_HOME_US)
    time.sleep(SERVO_SETTLE_S)


def set_servo_alert(pi: pigpio.pi) -> None:
    pi.set_servo_pulsewidth(SERVO_PIN, SERVO_ALERT_US)
    time.sleep(SERVO_SETTLE_S)


def release_servo(pi: pigpio.pi) -> None:
    pi.set_servo_pulsewidth(SERVO_PIN, 0)


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


def stop_motors(pi: pigpio.pi) -> None:
    apply_drive(pi, 0.0, 0.0)


def right(pi: pigpio.pi, duty: float = TURN_PWM_DUTY) -> None:
    apply_drive(pi, duty, -duty)


def left(pi: pigpio.pi, duty: float = TURN_PWM_DUTY) -> None:
    apply_drive(pi, -duty, duty)


def format_distance(value: float | None) -> str:
    if value is None:
        return "invalid"
    return f"{value:.1f} cm"


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
    distance = (duration * 34300.0) / 2.0

    if abs(distance - LOCK_DISTANCE_CM) <= LOCK_TOLERANCE_CM:
        return None
    if distance < MIN_DISTANCE_CM or distance > MAX_DISTANCE_CM:
        return None
    return distance


def average_distance(pi: pigpio.pi, sensor_name: str, sample_seconds: float, debug: bool = False) -> float | None:
    samples: list[float] = []
    start = time.perf_counter()
    while time.perf_counter() - start < sample_seconds and not shutdown_event.is_set():
        if is_yellow_paused():
            break
        dist = get_distance_cm(pi, sensor_name, debug=debug)
        if dist is not None:
            samples.append(dist)
        time.sleep(0.01)
    if not samples:
        return None
    return median(samples)


def report_side_status(left_avg: float | None, right_avg: float | None) -> None:
    print(f"Side verification -> left: {format_distance(left_avg)}, right: {format_distance(right_avg)}")


def report_single_sensor(label: str, avg: float | None) -> None:
    print(f"{label} -> {format_distance(avg)}")


def choose_turn_direction(left_avg: float | None, right_avg: float | None) -> str | None:
    left_clear = left_avg is not None and left_avg >= TURN_OPENING_CLEAR_CM
    right_clear = right_avg is not None and right_avg >= TURN_OPENING_CLEAR_CM
    if left_clear and right_clear:
        return "left" if left_avg >= right_avg else "right"
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
    while time.perf_counter() - start < duration_s and not shutdown_event.is_set():
        if is_yellow_paused():
            stop_motors(pi)
            return
        time.sleep(0.02)
    stop_motors(pi)


def verify_and_turn(pi: pigpio.pi, debug: bool) -> bool:
    left_avg = average_distance(pi, "left", SIDE_SAMPLE_SECONDS, debug=debug)
    right_avg = average_distance(pi, "right", SIDE_SAMPLE_SECONDS, debug=debug)
    report_side_status(left_avg, right_avg)
    direction = choose_turn_direction(left_avg, right_avg)
    if direction is None:
        print("Both sides obstructed or invalid. Robot remains stopped.")
        return False
    print(f"Turning {direction} based on side clearance")
    set_motion_status("turning", f"Turning {direction} based on side ultrasonic clearance.")
    drive_turn(direction, TURN_SECONDS, pi)
    front_avg = average_distance(pi, "front", POST_TURN_STABILIZE_SECONDS, debug=debug)
    report_single_sensor("Post-turn front check", front_avg)
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
    if distance_cm is None or distance_cm > SIDE_PID_ACTIVE_MAX_CM:
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


def choose_forward_base_duty(front_cm: float | None, left_cm: float | None, right_cm: float | None) -> float:
    side_values = [dist for dist in (left_cm, right_cm) if dist is not None]
    nearest_side = min(side_values) if side_values else None
    base_duty = BASE_CRUISE_PWM_DUTY
    if nearest_side is not None:
        if nearest_side <= PASSAGE_TIGHT_SIDE_CM + 0.5:
            base_duty = BASE_CRAWL_PWM_DUTY
        elif nearest_side <= PASSAGE_TIGHT_SIDE_CM + 2.0:
            base_duty = BASE_TIGHT_PWM_DUTY
        elif nearest_side <= 12.0:
            base_duty = (BASE_CRUISE_PWM_DUTY + BASE_TIGHT_PWM_DUTY) / 2.0
    if front_cm is not None and front_cm < FRONT_SLOW_CM:
        base_duty = min(base_duty, BASE_TIGHT_PWM_DUTY)
    return base_duty


def get_pid_profile(front_cm: float | None, mode: str, coarse_recovery: bool) -> tuple[float, float, float, float, float]:
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


def is_center_blocked(frame: np.ndarray) -> tuple[bool, tuple[int, int, int, int]]:
    try:
        h, w = frame.shape[:2]
        s_h, e_h = h // 4, (3 * h) // 4
        s_w, e_w = w // 4, (3 * w) // 4
        roi = frame[s_h:e_h, s_w:e_w]
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        score = cv2.Laplacian(gray, cv2.CV_64F).var()
        return score < 25, (s_w, s_h, e_w, e_h)
    except Exception:
        return False, (0, 0, 0, 0)


def update_color_memory(frame: np.ndarray) -> list[tuple[str, tuple[int, int, int, int], float, bool]]:
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    visual_items: list[tuple[str, tuple[int, int, int, int], float, bool]] = []
    locked_color = "None"
    locked_area = 0.0

    for color_name, ranges in COLORS.items():
        mask = None
        for low, high in ranges:
            current = cv2.inRange(hsv, low, high)
            mask = current if mask is None else cv2.bitwise_or(mask, current)
        mask = cv2.medianBlur(mask, 3)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        best_box = None
        best_area = 0.0
        for contour in contours:
            area = cv2.contourArea(contour)
            if area < COLOR_MIN_AREA[color_name] or area > 50000:
                continue
            x, y, w, h = cv2.boundingRect(contour)
            if h <= 0:
                continue
            aspect_ratio = float(w) / float(h)
            if 0.35 < aspect_ratio < 3.0 and area > best_area:
                best_area = area
                best_box = (x, y, w, h)

        mem = detection_memory[color_name]
        if best_box is not None:
            mem["box"] = best_box
            mem["frames"] = BOX_HOLD_FRAMES
            mem["consecutive"] += 1
            mem["area"] = best_area
        else:
            mem["consecutive"] = 0
            mem["area"] = 0.0
            if mem["frames"] > 0:
                mem["frames"] -= 1
            else:
                mem["box"] = None

        is_locked = mem["consecutive"] >= COLOR_LOCK_FRAMES
        if is_locked and mem["area"] >= locked_area:
            locked_color = color_name
            locked_area = mem["area"]

        if mem["box"] is not None and mem["frames"] > 0:
            visual_items.append((color_name, mem["box"], mem["area"], is_locked))

    set_locked_color(locked_color)
    return visual_items


def annotate_frame(frame: np.ndarray, roi: tuple[int, int, int, int], visual_items: list[tuple[str, tuple[int, int, int, int], float, bool]]) -> np.ndarray:
    with state_lock:
        paused = yellow_pause_latched
        locked_color = latest_locked_color
        blocked = camera_blocked_confirmed
        status = motion_status
        detail = status_detail
        front_text = sensor_snapshot["front"]
        left_text = sensor_snapshot["left"]
        right_text = sensor_snapshot["right"]

    x1, y1, x2, y2 = roi
    roi_color = (0, 0, 255) if blocked else (80, 0, 0)
    roi_thickness = 3 if blocked else 1
    cv2.rectangle(frame, (x1, y1), (x2, y2), roi_color, roi_thickness)

    for color_name, box, area, is_locked in visual_items:
        bx, by, bw, bh = box
        color = COLOR_BOXES[color_name]
        label = f"{color_name} {'[LOCKED]' if is_locked else '...'} {int(area)}"
        cv2.rectangle(frame, (bx, by), (bx + bw, by + bh), color, 2 if is_locked else 1)
        cv2.putText(frame, label, (bx, max(18, by - 6)), cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 2)

    lines = [
        f"STATE: {status}",
        f"DETAIL: {detail}",
        f"LOCKED COLOR: {locked_color}",
        f"YELLOW PAUSE: {'ON' if paused else 'OFF'}",
        f"ULTRA F:{front_text} L:{left_text} R:{right_text}",
    ]
    for idx, line in enumerate(lines):
        cv2.putText(
            frame,
            line,
            (10, 24 + (idx * 24)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (255, 255, 255),
            2,
        )
    return frame


def camera_worker(pi: pigpio.pi) -> None:
    global latest_jpeg, latest_frame_ts, camera_blocked_confirmed, camera_blocked_frames

    cap = cv2.VideoCapture(CAMERA_INDEX)
    if not cap.isOpened():
        set_motion_status("camera-fault", "Camera failed to open.")
        print("Camera failed to open.", file=sys.stderr)
        shutdown_event.set()
        return

    camera_ready_event.set()
    set_motion_status("camera-ready", "Camera stream active.")

    try:
        while not shutdown_event.is_set():
            ok, frame = cap.read()
            if not ok or frame is None:
                time.sleep(FRAME_SLEEP_S)
                continue

            blocked_now, roi = is_center_blocked(frame)
            visual_items = update_color_memory(frame)

            if blocked_now:
                camera_blocked_frames += 1
            else:
                camera_blocked_frames = 0
            camera_blocked_confirmed = camera_blocked_frames >= CAMERA_BLOCK_CONFIRM_FRAMES

            locked = latest_locked_color
            if locked == "Yellow":
                if not is_yellow_paused():
                    set_yellow_pause(True)
                    set_servo_alert(pi)
            elif locked == "Green" and is_yellow_paused():
                set_yellow_pause(False)
                set_servo_home(pi)

            annotated = annotate_frame(frame, roi, visual_items)
            ok, buffer = cv2.imencode(".jpg", annotated)
            if ok:
                with state_lock:
                    latest_jpeg = buffer.tobytes()
                    latest_frame_ts = time.time()

            time.sleep(FRAME_SLEEP_S)
    finally:
        cap.release()


def frame_generator():
    while not shutdown_event.is_set():
        with state_lock:
            frame = latest_jpeg
        if frame is None:
            time.sleep(0.05)
            continue
        yield (b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + frame + b"\r\n")
        time.sleep(0.03)


def obstacle_confirmed_by_camera(front_cm: float | None) -> bool:
    with state_lock:
        blocked = camera_blocked_confirmed
    return blocked and front_cm is not None and front_cm <= FRONT_STOP_CM


def run_navigation(pi: pigpio.pi, debug: bool) -> None:
    front_filtered: float | None = None
    left_filtered: float | None = None
    right_filtered: float | None = None

    integral = 0.0
    previous_error = 0.0
    coarse_recovery = False
    loop_started_at = time.perf_counter()
    last_debug_at = 0.0
    yellow_pause_active = False

    while not shutdown_event.is_set():
        if is_yellow_paused():
            if not yellow_pause_active:
                stop_motors(pi)
                integral = 0.0
                previous_error = 0.0
                coarse_recovery = False
                yellow_pause_active = True
            time.sleep(0.05)
            continue

        if yellow_pause_active:
            yellow_pause_active = False
            set_motion_status("resuming", "Green detected. Returning to straight PID.")
            time.sleep(RESUME_PAUSE_S)

        front_filtered = smooth_distance(front_filtered, get_distance_cm(pi, "front", debug=debug))
        left_filtered = smooth_distance(left_filtered, get_distance_cm(pi, "left", debug=debug))
        right_filtered = smooth_distance(right_filtered, get_distance_cm(pi, "right", debug=debug))
        set_sensor_snapshot(front_filtered, left_filtered, right_filtered)

        if front_filtered is not None and front_filtered < FRONT_EMERGENCY_STOP_CM:
            stop_motors(pi)
            set_motion_status("emergency-stop", f"Front ultrasonic too close at {front_filtered:.1f} cm.")
            time.sleep(0.1)
            continue

        if obstacle_confirmed_by_camera(front_filtered):
            stop_motors(pi)
            set_motion_status(
                "obstacle-stop",
                f"Camera obstruction confirmed by front ultrasonic at {front_filtered:.1f} cm.",
            )
            turned = verify_and_turn(pi, debug=debug)
            integral = 0.0
            previous_error = 0.0
            coarse_recovery = False
            if not turned:
                time.sleep(0.2)
            continue

        side_values = [dist for dist in (left_filtered, right_filtered) if dist is not None]
        if side_values and min(side_values) < SIDE_HARD_LIMIT_CM:
            stop_motors(pi)
            set_motion_status(
                "side-limit",
                f"Side clearance too low (left={format_distance(left_filtered)}, right={format_distance(right_filtered)}).",
            )
            time.sleep(0.1)
            continue

        error, mode = compute_passage_error(left_filtered, right_filtered)
        if error is None:
            stop_motors(pi)
            set_motion_status("side-lost", "Lost both side-wall readings.")
            time.sleep(0.1)
            continue

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
        control_signal = 0.0 if mode == "trim-only" else ((kp * error) + (ki * integral) + (kd * derivative))
        raw_adjust = 0.0 if mode == "trim-only" else clamp(control_signal, -PID_MAX_LEFT_ADJUST, PID_MAX_RIGHT_ADJUST)
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
        set_motion_status("running", "Straight PID active.")
        previous_error = error

        if debug and (now - last_debug_at) >= PID_DEBUG_PRINT_INTERVAL_S:
            print(
                "[pid] "
                f"mode={mode}{'+coarse' if coarse_recovery else ''} "
                f"front={format_distance(front_filtered)} "
                f"left={format_distance(left_filtered)} "
                f"right={format_distance(right_filtered)} "
                f"err={error:.2f} delta={side_delta:.2f} raw={control_signal:.1f} adj={adjust:.1f} "
                f"pwm=({left_duty:.0f},{right_duty:.0f})"
            )
            last_debug_at = now

        elapsed = time.perf_counter() - now
        sleep_for = PID_LOOP_DT_S - elapsed
        if sleep_for > 0:
            time.sleep(sleep_for)


@app.route("/")
def index():
    return render_template_string(HTML)


@app.route("/video_feed")
def video_feed():
    return Response(frame_generator(), mimetype="multipart/x-mixed-replace; boundary=frame")


@app.route("/stop_server")
def stop_server():
    shutdown_event.set()
    os.kill(os.getpid(), signal.SIGINT)
    return "Shutdown complete."


def main() -> None:
    debug = "--debug" in sys.argv
    pi = pigpio.pi()
    if not pi.connected:
        print("pigpio daemon not running or not reachable.", file=sys.stderr)
        sys.exit(1)

    setup_motors(pi)
    setup_ultrasonic(pi)
    setup_servo(pi)
    stop_motors(pi)
    set_motion_status("boot", "Robot controller starting.")

    camera_thread = threading.Thread(target=camera_worker, args=(pi,), daemon=True)
    nav_thread = threading.Thread(target=run_navigation, args=(pi, debug), daemon=True)

    camera_thread.start()
    if not camera_ready_event.wait(timeout=5.0):
        shutdown_event.set()
        raise RuntimeError("Camera thread did not become ready.")
    nav_thread.start()

    print("robot.py ready. Web stream, yellow/green color latches, and ultrasonic PID are active.")

    try:
        app.run(host="0.0.0.0", port=5000, threaded=True)
    except KeyboardInterrupt:
        print("\nStopped by user")
    finally:
        shutdown_event.set()
        stop_motors(pi)
        cleanup_ultrasonic(pi)
        set_servo_home(pi)
        release_servo(pi)
        pi.stop()


if __name__ == "__main__":
    main()
