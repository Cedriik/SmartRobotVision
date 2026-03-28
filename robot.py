# Checkpoint snapshot: init_path_camera_cp1.py
# Source: initial_path_camera.py
# Created: 2026-03-23
# Note: Saved before further turn-trigger logic changes.

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
from flask import Flask, Response, jsonify, request

app = Flask(__name__)

IN1, IN2, IN3, IN4, ENA, ENB = 17, 27, 22, 23, 13, 12
PWM_HZ = 1000
FRONT_TRIG, FRONT_ECHO = 18, 19
LEFT_TRIG, LEFT_ECHO = 24, 25
RIGHT_TRIG, RIGHT_ECHO = 4, 26
SERVO_PIN = 16
SERVO_HOME_US = 500
SERVO_ALERT_US = 1300
SERVO_REVERSE_US = SERVO_HOME_US
SERVO_SETTLE_S = 0.25

BASE_CRUISE_PWM_DUTY = 56
BASE_TIGHT_PWM_DUTY = 40
BASE_CRAWL_PWM_DUTY = 32
TURN_PWM_DUTY = 100
TURN_MINI_PWM_DUTY = 95
MIN_DRIVE_PWM_DUTY = 20
MAX_DRIVE_PWM_DUTY = 65
LEFT_MOTOR_TRIM_PWM = 3.0
RIGHT_MOTOR_TRIM_PWM = 0.0

FRONT_STOP_CM = 8.0
FRONT_SLOW_CM = 23.0
TURN_OPENING_CLEAR_CM = 10.0
LEFT_REFERENCE_CM = 8.6
RIGHT_REFERENCE_CM = 7.8
PASSAGE_TIGHT_SIDE_CM = 5.0
SIDE_HARD_LIMIT_CM = 2.5
SIDE_PID_ACTIVE_MAX_CM = 15.0
TURN_ALIGN_FRONT_CLEAR_CM = 15.0
TURN_ALIGN_BOTH_ERROR_CM = 1.2
TURN_ALIGN_SINGLE_TOL_CM = 2.0

MIN_INTERVAL_S = 0.03
ECHO_RISE_TIMEOUT_S = 0.03
ECHO_FALL_TIMEOUT_S = 0.025
ECHO_GLITCH_US = 350
STUCK_HIGH_RECOVER_S = 0.01
LOCK_DISTANCE_CM = 1000.0
LOCK_TOLERANCE_CM = 120.0
MIN_DISTANCE_CM = 2.0
MAX_DISTANCE_CM = 400.0

SIDE_SAMPLE_SECONDS = 0.8
PRE_TURN_VERIFY_SECONDS = 0.6
POST_TURN_STABILIZE_SECONDS = 0.5
TURN_SECONDS = 0.6
TURN_COARSE_RATIO = 0.78
TURN_MINI_SETTLE_S = 0.08
TURN_MINI_FIRST_PULSE_S = 0.24
TURN_MINI_DOUBLE_PULSE_S = 0.14
TURN_MINI_MAX_TOTAL_SECONDS = 3.2
TURN_FRONT_CHECK_SECONDS = 0.12
TURN_PRE_MINI_VERIFY_SECONDS = 1.2
RESUME_PAUSE_S = 0.20
RESUME_STRAIGHT_PWM = 70.0
RESUME_STRAIGHT_SECONDS = 2.0

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
COARSE_RECOVERY_ENTER_ERROR = 4.0
COARSE_RECOVERY_EXIT_ERROR = 2.0
COARSE_RECOVERY_ENTER_DELTA = 4.0
COARSE_RECOVERY_EXIT_DELTA = 2.0
COARSE_KP_MULTIPLIER = 1.45
COARSE_KI_MULTIPLIER = 1.60
COARSE_KD_MULTIPLIER = 1.30
COARSE_RIGHT_BOOST = 1.25
COARSE_MIN_ADJUST_PWM = 8.0
PID_DEBUG_PRINT_INTERVAL_S = 0.25

CAMERA_INDEX = 0
STREAM_PORT = 5001
LINE_ROI_HEIGHT = 0.42
CORRIDOR_THRESHOLD = 60
CORRIDOR_MIN_AREA = 2200
CORRIDOR_FILL_TARGET = 0.30
LINE_BLACK_THRESHOLD = 85
LINE_MIN_CONTOUR_AREA = 450
LINE_EDGE_CANNY_LOW = 70
LINE_EDGE_CANNY_HIGH = 180
LINE_HOUGH_THRESHOLD = 28
LINE_HOUGH_MIN_LENGTH = 32
LINE_HOUGH_MAX_GAP = 22
LINE_SINGLE_EDGE_HALF_WIDTH_PX = 90.0
LINE_OFFSET_KP = 11.0
LINE_HEADING_KD = 6.0
LINE_MAX_ADJUST = 18.0
TURN_CAMERA_MIN_CONFIDENCE = 0.35
TURN_CAMERA_OFFSET_OK = 0.18
TURN_CAMERA_HEADING_OK = 0.22
TURN_CAMERA_ALIGN_GAIN = 0.60
TURN_CAMERA_SIDE_OPEN_THRESHOLD = 0.12
TURN_CAMERA_SIDE_DIFF_THRESHOLD = 0.08
SIDE_CAMERA_ROI_TOP = 0.34
SIDE_CAMERA_ROI_BOTTOM = 0.82
SIDE_CAMERA_ROI_WIDTH = 0.24
TURN_CAMERA_CORRIDOR_ANGLE_OK_DEG = 10.0
TURN_CAMERA_VANISH_OFFSET_OK = 0.20
FEATURE_MAX_CORNERS = 80
FEATURE_QUALITY_LEVEL = 0.01
FEATURE_MIN_DISTANCE = 7
FEATURE_STABLE_MOTION_PX = 1.8
FEATURE_ROTATION_OK_DEG = 4.0
FEATURE_CONFIDENCE_MIN = 0.25
COLOR_LOCK_FRAMES = 8
YELLOW_LOCK_FRAMES = 1
COLOR_RELEASE_FRAMES = 3
COLOR_BRIGHTNESS_TOLERANCE = 0.20
COLOR_TRIGGER_COOLDOWN_S = 3.0
YELLOW_TRIGGER_COOLDOWN_S = 10.0
BOX_HOLD_FRAMES = 3
COLORS = {
    "Yellow": [
        (np.array([20, 120, 120]), np.array([35, 255, 255])),
    ],
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

PATH_TURN_SECONDS = 0.6
PATH_BACKWARD_SECONDS = 1.0
DEFAULT_PATH_SEQUENCE = [
    "straight",
    "right",
    "forward",
    "right",
    "forward",
    "right",
    "forward",
    "left",
    "forward",
    "left",
    "forward",
    "left",
    "forward",
]
PATH_SEQUENCE = list(DEFAULT_PATH_SEQUENCE)
STEP_DURATION_BY_DIRECTION = {
    "left": PATH_TURN_SECONDS,
    "right": PATH_TURN_SECONDS,
    "backward": PATH_BACKWARD_SECONDS,
}
SENSORS = {"front": {"trig": FRONT_TRIG, "echo": FRONT_ECHO, "last_trigger": 0.0}, "left": {"trig": LEFT_TRIG, "echo": LEFT_ECHO, "last_trigger": 0.0}, "right": {"trig": RIGHT_TRIG, "echo": RIGHT_ECHO, "last_trigger": 0.0}}

state_lock = threading.Lock()
config_lock = threading.Lock()
shutdown_event = threading.Event()
camera_ready_event = threading.Event()
run_enabled_event = threading.Event()
restart_requested_event = threading.Event()
latest_jpeg = None
latest_locked_color = "None"
yellow_pause_latched = False
servo_is_alert = False
vision_detail_text = "no-color-lock"
resume_boost_until = 0.0
line_state = {
    "found": False,
    "mode": "none",
    "offset": 0.0,
    "heading": 0.0,
    "confidence": 0.0,
    "left_open": 0.0,
    "right_open": 0.0,
    "left_blocked": False,
    "right_blocked": False,
    "corridor_angle_deg": 0.0,
    "vanishing_offset": 0.0,
    "geometry_confidence": 0.0,
    "feature_motion_px": 0.0,
    "feature_rotation_deg": 0.0,
    "feature_confidence": 0.0,
    "feature_stable": True,
}
sensor_snapshot = {"front": "invalid", "left": "invalid", "right": "invalid"}
status_text = "idle-waiting-start"
detection_memory = {
    name: {"box": None, "frames": 0, "consecutive": 0, "misses": 0, "area": 0.0}
    for name in COLORS
}
color_cooldowns = {name: 0.0 for name in COLORS}


def clamp(value, low, high):
    return max(low, min(high, value))


def format_distance(value):
    return "invalid" if value is None else f"{value:.1f} cm"


def set_status(text):
    global status_text
    with state_lock:
        changed = text != status_text
        status_text = text
    if changed:
        print(text)


def set_sensor_snapshot(front, left, right):
    with state_lock:
        sensor_snapshot["front"] = format_distance(front)
        sensor_snapshot["left"] = format_distance(left)
        sensor_snapshot["right"] = format_distance(right)


def set_locked_color(name):
    global latest_locked_color
    with state_lock:
        latest_locked_color = name


def set_vision_detail(text):
    global vision_detail_text
    with state_lock:
        vision_detail_text = text


def set_yellow_pause(active):
    global yellow_pause_latched
    changed = False
    with state_lock:
        if yellow_pause_latched != active:
            yellow_pause_latched = active
            changed = True
    if changed:
        if active:
            set_status("[vision] yellow locked - waiting for green")
        else:
            set_status("[vision] green locked - yellow pause cleared")


def is_yellow_paused():
    with state_lock:
        return yellow_pause_latched


def robot_paused():
    return not run_enabled_event.is_set()


def get_path_steps():
    with config_lock:
        return [(direction, STEP_DURATION_BY_DIRECTION.get(direction)) for direction in PATH_SEQUENCE]


def get_control_state():
    with config_lock, state_lock:
        return {
            "paused": robot_paused(),
            "turn_seconds": PATH_TURN_SECONDS,
            "path_sequence": list(PATH_SEQUENCE),
            "status": status_text,
        }


def update_turn_seconds(seconds):
    global PATH_TURN_SECONDS
    with config_lock:
        PATH_TURN_SECONDS = seconds
        STEP_DURATION_BY_DIRECTION["left"] = seconds
        STEP_DURATION_BY_DIRECTION["right"] = seconds


def append_path_step(step_name):
    with config_lock:
        PATH_SEQUENCE.append(step_name)


def clear_path_sequence():
    with config_lock:
        PATH_SEQUENCE.clear()


def reset_default_path_sequence():
    with config_lock:
        PATH_SEQUENCE[:] = DEFAULT_PATH_SEQUENCE


def edit_requires_pause():
    if not robot_paused():
        return Response("Pause robot before editing web controls.", status=409)
    return None


def mark_path_edited(message):
    restart_requested_event.set()
    set_status(message)


def motion_restart_requested():
    return restart_requested_event.is_set()


def wait_until_run_enabled(pi):
    global resume_boost_until
    was_paused = False
    while not shutdown_event.is_set() and (not run_enabled_event.is_set() or is_yellow_paused()):
        was_paused = True
        stop(pi)
        if is_yellow_paused():
            set_status("[nav] paused by yellow - waiting for green")
        else:
            set_status("[nav] paused - press START")
        time.sleep(0.05)
    if not shutdown_event.is_set() and was_paused:
        resume_boost_until = time.perf_counter() + RESUME_STRAIGHT_SECONDS
        set_status(f"[nav] smooth resume - holding PWM {RESUME_STRAIGHT_PWM:.0f}")
    return not shutdown_event.is_set()


def setup_motors(pi):
    for pin in (IN1, IN2, IN3, IN4, ENA, ENB):
        pi.set_mode(pin, pigpio.OUTPUT)
    pi.set_PWM_frequency(ENA, PWM_HZ)
    pi.set_PWM_frequency(ENB, PWM_HZ)
    pi.set_PWM_range(ENA, 100)
    pi.set_PWM_range(ENB, 100)
    pi.set_PWM_dutycycle(ENA, 0)
    pi.set_PWM_dutycycle(ENB, 0)


def setup_ultrasonic(pi):
    for sensor in SENSORS.values():
        pi.set_mode(sensor["trig"], pigpio.OUTPUT)
        pi.set_mode(sensor["echo"], pigpio.INPUT)
        pi.set_pull_up_down(sensor["echo"], pigpio.PUD_DOWN)
        pi.set_glitch_filter(sensor["echo"], ECHO_GLITCH_US)
        pi.write(sensor["trig"], 0)
    time.sleep(0.08)


def cleanup_ultrasonic(pi):
    for sensor in SENSORS.values():
        pi.write(sensor["trig"], 0)
        pi.set_glitch_filter(sensor["echo"], 0)


def setup_servo(pi):
    pi.set_mode(SERVO_PIN, pigpio.OUTPUT)
    set_servo_home(pi)


def set_servo_home(pi):
    global servo_is_alert
    pi.set_servo_pulsewidth(SERVO_PIN, SERVO_HOME_US)
    servo_is_alert = False
    time.sleep(SERVO_SETTLE_S)


def set_servo_alert(pi):
    global servo_is_alert
    pi.set_servo_pulsewidth(SERVO_PIN, SERVO_ALERT_US)
    servo_is_alert = True
    time.sleep(SERVO_SETTLE_S)


def set_servo_reverse(pi):
    global servo_is_alert
    pi.set_servo_pulsewidth(SERVO_PIN, SERVO_REVERSE_US)
    servo_is_alert = True
    time.sleep(SERVO_SETTLE_S)


def release_servo(pi):
    pi.set_servo_pulsewidth(SERVO_PIN, 0)


def apply_drive(pi, left_speed, right_speed):
    left_speed = clamp(left_speed, -100.0, 100.0)
    right_speed = clamp(right_speed, -100.0, 100.0)
    if left_speed > 0:
        pi.write(IN1, 0); pi.write(IN2, 1)
    elif left_speed < 0:
        pi.write(IN1, 1); pi.write(IN2, 0)
    else:
        pi.write(IN1, 0); pi.write(IN2, 0)
    if right_speed > 0:
        pi.write(IN3, 0); pi.write(IN4, 1)
    elif right_speed < 0:
        pi.write(IN3, 1); pi.write(IN4, 0)
    else:
        pi.write(IN3, 0); pi.write(IN4, 0)
    pi.set_PWM_dutycycle(ENA, int(abs(left_speed)))
    pi.set_PWM_dutycycle(ENB, int(abs(right_speed)))


def stop(pi):
    apply_drive(pi, 0.0, 0.0)


def backward(pi, duty=BASE_TIGHT_PWM_DUTY):
    apply_drive(pi, -duty, -duty)


def right(pi, duty=TURN_PWM_DUTY):
    apply_drive(pi, duty, -duty)


def left(pi, duty=TURN_PWM_DUTY):
    apply_drive(pi, -duty, duty)


def get_distance_cm(pi, sensor_name, debug=False):
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
    if abs(distance - LOCK_DISTANCE_CM) <= LOCK_TOLERANCE_CM or distance < MIN_DISTANCE_CM or distance > MAX_DISTANCE_CM:
        return None
    return distance


def average_distance(pi, sensor_name, sample_seconds, debug=False):
    samples = []
    start = time.perf_counter()
    while time.perf_counter() - start < sample_seconds and not shutdown_event.is_set():
        dist = get_distance_cm(pi, sensor_name, debug=debug)
        if dist is not None:
            samples.append(dist)
        time.sleep(0.01)
    return None if not samples else median(samples)


def smooth_distance(previous, current):
    if current is None:
        return previous
    if previous is None:
        return current
    return (PID_FILTER_ALPHA * current) + ((1.0 - PID_FILTER_ALPHA) * previous)


def apply_deadband(error):
    return 0.0 if abs(error) <= PID_DEADBAND_CM else error


def pid_visible_distance(distance_cm):
    return None if distance_cm is None or distance_cm > SIDE_PID_ACTIVE_MAX_CM else distance_cm


def scale_balance_error(error):
    if error == 0.0:
        return 0.0
    gain = POSITIVE_ERROR_EXP_GAIN if error > 0.0 else NEGATIVE_ERROR_EXP_GAIN
    return error * min(math.exp(abs(error) * gain), MAX_ERROR_EXP_SCALE)


def compute_passage_error(left_cm, right_cm):
    left_pid = pid_visible_distance(left_cm)
    right_pid = pid_visible_distance(right_cm)
    if left_pid is not None and right_pid is not None:
        raw = ((right_pid - RIGHT_REFERENCE_CM) * RIGHT_REFERENCE_WEIGHT) - ((left_pid - LEFT_REFERENCE_CM) * LEFT_REFERENCE_WEIGHT)
        return scale_balance_error(apply_deadband(raw)), "center"
    if right_pid is not None:
        raw = (right_pid - RIGHT_REFERENCE_CM) * RIGHT_REFERENCE_WEIGHT
        return scale_balance_error(apply_deadband(raw)), "right-wall"
    if left_pid is not None:
        raw = (LEFT_REFERENCE_CM - left_pid) * LEFT_REFERENCE_WEIGHT
        return scale_balance_error(apply_deadband(raw)), "left-wall"
    return 0.0, "trim-only"


def compute_visible_side_delta(left_cm, right_cm):
    left_pid = pid_visible_distance(left_cm)
    right_pid = pid_visible_distance(right_cm)
    if left_pid is None or right_pid is None:
        return 0.0
    return right_pid - left_pid


def choose_forward_base_duty(front_cm, left_cm, right_cm):
    side_values = [dist for dist in (left_cm, right_cm) if dist is not None]
    nearest_side = min(side_values) if side_values else None
    base = BASE_CRUISE_PWM_DUTY
    if nearest_side is not None:
        if nearest_side <= PASSAGE_TIGHT_SIDE_CM + 0.5:
            base = BASE_CRAWL_PWM_DUTY
        elif nearest_side <= PASSAGE_TIGHT_SIDE_CM + 2.0:
            base = BASE_TIGHT_PWM_DUTY
        elif nearest_side <= 12.0:
            base = (BASE_CRUISE_PWM_DUTY + BASE_TIGHT_PWM_DUTY) / 2.0
    if front_cm is not None and front_cm < FRONT_SLOW_CM:
        base = min(base, BASE_TIGHT_PWM_DUTY)
    return base


def get_pid_profile(front_cm, mode, coarse_recovery):
    kp, ki, kd = PID_KP, PID_KI, PID_KD
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


def line_bottom_x(x1, y1, x2, y2, target_y):
    dy = y2 - y1
    if dy == 0:
        return None
    t = (target_y - y1) / dy
    return x1 + (t * (x2 - x1))


def analyze_corridor_candidate(mask, mode, roi_width, roi_height):
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contour, best_area = None, 0.0
    for candidate in contours:
        area = cv2.contourArea(candidate)
        if area > best_area and area >= CORRIDOR_MIN_AREA:
            contour, best_area = candidate, area
    if contour is None:
        return None

    moments = cv2.moments(contour)
    if moments["m00"] <= 0:
        return None

    cx = int(moments["m10"] / moments["m00"])
    cy = int(moments["m01"] / moments["m00"])
    heading = 0.0
    if len(contour) >= 2:
        vx, vy, _, _ = cv2.fitLine(contour, cv2.DIST_L2, 0, 0.01, 0.01)
        heading = clamp(float(vx) / max(abs(float(vy)), 0.35), -1.0, 1.0)

    confidence = clamp(best_area / (roi_width * max(roi_height, 1) * CORRIDOR_FILL_TARGET), 0.0, 1.0)
    return {
        "found": True,
        "mode": mode,
        "offset": clamp((cx - (roi_width / 2.0)) / (roi_width / 2.0), -1.0, 1.0),
        "heading": heading,
        "confidence": confidence,
        "contour": contour,
        "cx": cx,
        "cy": cy,
    }


def compute_mask_open_ratio(mask, x1, y1, x2, y2):
    roi = mask[y1:y2, x1:x2]
    if roi.size == 0:
        return 0.0
    return float(cv2.countNonZero(roi)) / float(roi.size)


def compute_hough_geometry(blur_roi, overlay):
    height, width = blur_roi.shape[:2]
    edges = cv2.Canny(blur_roi, LINE_EDGE_CANNY_LOW, LINE_EDGE_CANNY_HIGH)
    lines = cv2.HoughLinesP(
        edges,
        1,
        np.pi / 180.0,
        LINE_HOUGH_THRESHOLD,
        minLineLength=LINE_HOUGH_MIN_LENGTH,
        maxLineGap=LINE_HOUGH_MAX_GAP,
    )

    left_edges = []
    right_edges = []
    left_top_x = []
    right_top_x = []
    weighted_angles = []
    weighted_lengths = []

    if lines is not None:
        for line in lines:
            x1, y1, x2, y2 = line[0]
            dx = x2 - x1
            dy = y2 - y1
            length = math.hypot(dx, dy)
            if length < LINE_HOUGH_MIN_LENGTH or abs(dy) < 8:
                continue

            bottom_x = line_bottom_x(x1, y1, x2, y2, height - 1.0)
            top_x = line_bottom_x(x1, y1, x2, y2, 0.0)
            if bottom_x is None or top_x is None:
                continue

            angle_deg = math.degrees(math.atan2(dx, dy))
            if abs(angle_deg) > 65.0:
                continue

            weighted_angles.append(angle_deg)
            weighted_lengths.append(length)
            mid_x = (x1 + x2) / 2.0
            color = (255, 0, 0) if mid_x < (width / 2.0) else (0, 0, 255)
            cv2.line(overlay, (x1, y1), (x2, y2), color, 1)
            if mid_x < (width / 2.0):
                left_edges.append(bottom_x)
                left_top_x.append(top_x)
            else:
                right_edges.append(bottom_x)
                right_top_x.append(top_x)

    corridor_angle_deg = 0.0
    if weighted_lengths:
        corridor_angle_deg = float(np.average(weighted_angles, weights=weighted_lengths))

    vanishing_offset = 0.0
    if left_top_x and right_top_x:
        top_center = (median(left_top_x) + median(right_top_x)) / 2.0
        vanishing_offset = clamp((top_center - (width / 2.0)) / (width / 2.0), -1.0, 1.0)

    lane_center = None
    lane_mode = "none"
    if left_edges and right_edges:
        lane_center = (median(left_edges) + median(right_edges)) / 2.0
        lane_mode = "double-edge"
    elif left_edges:
        lane_center = median(left_edges) + LINE_SINGLE_EDGE_HALF_WIDTH_PX
        lane_mode = "single-left"
    elif right_edges:
        lane_center = median(right_edges) - LINE_SINGLE_EDGE_HALF_WIDTH_PX
        lane_mode = "single-right"

    if lane_center is not None:
        lane_center = clamp(lane_center, 0.0, float(width - 1))
        cv2.line(overlay, (int(lane_center), 0), (int(lane_center), height - 1), (0, 255, 255), 2)

    geometry_confidence = clamp((len(weighted_angles) / 8.0), 0.0, 1.0)
    return {
        "lane_center": lane_center,
        "lane_mode": lane_mode,
        "corridor_angle_deg": corridor_angle_deg,
        "vanishing_offset": vanishing_offset,
        "geometry_confidence": geometry_confidence,
    }, overlay


def compute_feature_metrics(previous_gray, current_gray):
    if previous_gray is None or current_gray is None:
        return {
            "feature_motion_px": 0.0,
            "feature_rotation_deg": 0.0,
            "feature_confidence": 0.0,
            "feature_stable": True,
        }

    prev_points = cv2.goodFeaturesToTrack(
        previous_gray,
        maxCorners=FEATURE_MAX_CORNERS,
        qualityLevel=FEATURE_QUALITY_LEVEL,
        minDistance=FEATURE_MIN_DISTANCE,
    )
    if prev_points is None or len(prev_points) < 6:
        return {
            "feature_motion_px": 0.0,
            "feature_rotation_deg": 0.0,
            "feature_confidence": 0.0,
            "feature_stable": True,
        }

    next_points, status, _ = cv2.calcOpticalFlowPyrLK(previous_gray, current_gray, prev_points, None)
    if next_points is None or status is None:
        return {
            "feature_motion_px": 0.0,
            "feature_rotation_deg": 0.0,
            "feature_confidence": 0.0,
            "feature_stable": True,
        }

    good_old = prev_points[status.flatten() == 1]
    good_new = next_points[status.flatten() == 1]
    if len(good_old) < 6 or len(good_new) < 6:
        return {
            "feature_motion_px": 0.0,
            "feature_rotation_deg": 0.0,
            "feature_confidence": 0.0,
            "feature_stable": True,
        }

    motion = good_new - good_old
    mean_motion = float(np.mean(np.linalg.norm(motion, axis=1)))
    transform, _ = cv2.estimateAffinePartial2D(good_old, good_new)
    rotation_deg = 0.0
    if transform is not None:
        rotation_deg = math.degrees(math.atan2(transform[1, 0], transform[0, 0]))

    confidence = clamp(len(good_old) / 24.0, 0.0, 1.0)
    stable = mean_motion <= FEATURE_STABLE_MOTION_PX and abs(rotation_deg) <= FEATURE_ROTATION_OK_DEG
    return {
        "feature_motion_px": mean_motion,
        "feature_rotation_deg": rotation_deg,
        "feature_confidence": confidence,
        "feature_stable": stable,
    }


def analyze_line(frame):
    h, w = frame.shape[:2]
    roi_y = int(h * (1.0 - LINE_ROI_HEIGHT))
    roi = frame[roi_y:, :]
    gray_full = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    blur_full = cv2.GaussianBlur(gray_full, (5, 5), 0)
    blur = blur_full[roi_y:, :]
    kernel = np.ones((5, 5), np.uint8)
    _, corridor_dark_mask_full = cv2.threshold(blur_full, CORRIDOR_THRESHOLD, 255, cv2.THRESH_BINARY_INV)
    _, corridor_bright_mask_full = cv2.threshold(blur_full, CORRIDOR_THRESHOLD, 255, cv2.THRESH_BINARY)
    corridor_dark_mask = corridor_dark_mask_full[roi_y:, :]
    corridor_bright_mask = corridor_bright_mask_full[roi_y:, :]
    corridor_dark_mask = cv2.morphologyEx(corridor_dark_mask, cv2.MORPH_OPEN, kernel)
    corridor_dark_mask = cv2.morphologyEx(corridor_dark_mask, cv2.MORPH_CLOSE, kernel)
    corridor_bright_mask = cv2.morphologyEx(corridor_bright_mask, cv2.MORPH_OPEN, kernel)
    corridor_bright_mask = cv2.morphologyEx(corridor_bright_mask, cv2.MORPH_CLOSE, kernel)
    corridor_dark_mask_full = cv2.morphologyEx(corridor_dark_mask_full, cv2.MORPH_OPEN, kernel)
    corridor_dark_mask_full = cv2.morphologyEx(corridor_dark_mask_full, cv2.MORPH_CLOSE, kernel)
    corridor_bright_mask_full = cv2.morphologyEx(corridor_bright_mask_full, cv2.MORPH_OPEN, kernel)
    corridor_bright_mask_full = cv2.morphologyEx(corridor_bright_mask_full, cv2.MORPH_CLOSE, kernel)

    geometry_result, _ = compute_hough_geometry(blur, cv2.cvtColor(np.zeros_like(blur), cv2.COLOR_GRAY2BGR))

    candidates = [
        analyze_corridor_candidate(corridor_dark_mask, "corridor-dark", w, roi.shape[0]),
        analyze_corridor_candidate(corridor_bright_mask, "corridor-bright", w, roi.shape[0]),
    ]
    candidates = [candidate for candidate in candidates if candidate is not None]
    chosen_mask_full = corridor_dark_mask_full
    if candidates:
        best = max(candidates, key=lambda item: item["confidence"])
        chosen_mask = corridor_dark_mask if best["mode"] == "corridor-dark" else corridor_bright_mask
        chosen_mask_full = corridor_dark_mask_full if best["mode"] == "corridor-dark" else corridor_bright_mask_full
        overlay = cv2.cvtColor(chosen_mask, cv2.COLOR_GRAY2BGR)
        cv2.drawContours(overlay, [best["contour"]], -1, (0, 255, 0), 2)
        cv2.circle(overlay, (best["cx"], best["cy"]), 6, (0, 255, 255), -1)
        cv2.putText(
            overlay,
            best["mode"],
            (10, max(20, roi.shape[0] - 14)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (255, 255, 255),
            2,
        )
        side_top = int(h * SIDE_CAMERA_ROI_TOP)
        side_bottom = int(h * SIDE_CAMERA_ROI_BOTTOM)
        side_width = int(w * SIDE_CAMERA_ROI_WIDTH)
        left_open = compute_mask_open_ratio(chosen_mask_full, 0, side_top, side_width, side_bottom)
        right_open = compute_mask_open_ratio(chosen_mask_full, w - side_width, side_top, w, side_bottom)
        return {
            "found": True,
            "mode": best["mode"],
            "offset": best["offset"],
            "heading": best["heading"],
            "confidence": best["confidence"],
            "left_open": left_open,
            "right_open": right_open,
            "left_blocked": left_open < TURN_CAMERA_SIDE_OPEN_THRESHOLD,
            "right_blocked": right_open < TURN_CAMERA_SIDE_OPEN_THRESHOLD,
            "corridor_angle_deg": geometry_result["corridor_angle_deg"],
            "vanishing_offset": geometry_result["vanishing_offset"],
            "geometry_confidence": geometry_result["geometry_confidence"],
        }, overlay

    chosen_mask_full = corridor_dark_mask_full if cv2.countNonZero(corridor_dark_mask_full[roi_y:, :]) >= cv2.countNonZero(corridor_bright_mask_full[roi_y:, :]) else corridor_bright_mask_full
    side_top = int(h * SIDE_CAMERA_ROI_TOP)
    side_bottom = int(h * SIDE_CAMERA_ROI_BOTTOM)
    side_width = int(w * SIDE_CAMERA_ROI_WIDTH)
    left_open = compute_mask_open_ratio(chosen_mask_full, 0, side_top, side_width, side_bottom)
    right_open = compute_mask_open_ratio(chosen_mask_full, w - side_width, side_top, w, side_bottom)

    _, dark_mask = cv2.threshold(blur, LINE_BLACK_THRESHOLD, 255, cv2.THRESH_BINARY_INV)
    dark_mask = cv2.morphologyEx(dark_mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    overlay = cv2.cvtColor(dark_mask, cv2.COLOR_GRAY2BGR)
    result = {
        "found": False,
        "mode": "none",
        "offset": 0.0,
        "heading": 0.0,
        "confidence": 0.0,
        "left_open": left_open,
        "right_open": right_open,
        "left_blocked": left_open < TURN_CAMERA_SIDE_OPEN_THRESHOLD,
        "right_blocked": right_open < TURN_CAMERA_SIDE_OPEN_THRESHOLD,
        "corridor_angle_deg": geometry_result["corridor_angle_deg"],
        "vanishing_offset": geometry_result["vanishing_offset"],
        "geometry_confidence": geometry_result["geometry_confidence"],
        "feature_motion_px": 0.0,
        "feature_rotation_deg": 0.0,
        "feature_confidence": 0.0,
        "feature_stable": True,
    }
    contours, _ = cv2.findContours(dark_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contour, best_area = None, 0.0
    for candidate in contours:
        area = cv2.contourArea(candidate)
        if area > best_area and area >= LINE_MIN_CONTOUR_AREA:
            contour, best_area = candidate, area
    if contour is not None:
        m = cv2.moments(contour)
        if m["m00"] > 0:
            cx = int(m["m10"] / m["m00"])
            cy = int(m["m01"] / m["m00"])
            cv2.drawContours(overlay, [contour], -1, (0, 255, 0), 2)
            cv2.circle(overlay, (cx, cy), 6, (0, 255, 255), -1)
            heading = 0.0
            if len(contour) >= 2:
                vx, vy, _, _ = cv2.fitLine(contour, cv2.DIST_L2, 0, 0.01, 0.01)
                heading = clamp(float(vx) / max(abs(float(vy)), 0.35), -1.0, 1.0)
            result.update({"found": True, "mode": "line-contour", "offset": clamp((cx - (w / 2.0)) / (w / 2.0), -1.0, 1.0), "heading": heading, "confidence": clamp(best_area / (w * max(roi.shape[0], 1) * 0.20), 0.0, 1.0)})
            return result, overlay
    overlay = cv2.cvtColor(cv2.Canny(blur, LINE_EDGE_CANNY_LOW, LINE_EDGE_CANNY_HIGH), cv2.COLOR_GRAY2BGR)
    geometry_result, overlay = compute_hough_geometry(blur, overlay)
    lane_center = geometry_result["lane_center"]
    mode = geometry_result["lane_mode"]
    if lane_center is not None:
        result.update({"found": True, "mode": mode, "offset": clamp((lane_center - (w / 2.0)) / (w / 2.0), -1.0, 1.0), "heading": 0.0, "confidence": 0.55 if mode == "double-edge" else 0.35})
    result.update({
        "corridor_angle_deg": geometry_result["corridor_angle_deg"],
        "vanishing_offset": geometry_result["vanishing_offset"],
        "geometry_confidence": geometry_result["geometry_confidence"],
    })
    return result, overlay


def color_brightness_tolerance(color_name):
    return COLOR_BRIGHTNESS_TOLERANCE


def color_trigger_cooldown(color_name):
    return YELLOW_TRIGGER_COOLDOWN_S if color_name == "Yellow" else COLOR_TRIGGER_COOLDOWN_S


def color_lock_frames(color_name):
    return YELLOW_LOCK_FRAMES if color_name == "Yellow" else COLOR_LOCK_FRAMES


def adjusted_color_range(color_name, low, high):
    low_adj = low.copy()
    high_adj = high.copy()
    delta = int(255 * color_brightness_tolerance(color_name))
    low_adj[2] = max(0, int(low_adj[2]) - delta)
    high_adj[2] = min(255, int(high_adj[2]) + delta)
    return low_adj, high_adj


def update_color_memory(frame):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    visual_items = []
    locked_color = "None"
    locked_area = 0.0
    now = time.perf_counter()
    cooldown_text = []

    for color_name, ranges in COLORS.items():
        mask = None
        for low, high in ranges:
            low_adj, high_adj = adjusted_color_range(color_name, low, high)
            current = cv2.inRange(hsv, low_adj, high_adj)
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
            mem["misses"] = 0
            mem["area"] = best_area
        else:
            mem["misses"] += 1
            if mem["misses"] >= COLOR_RELEASE_FRAMES:
                mem["consecutive"] = 0
                mem["area"] = 0.0
            if mem["frames"] > 0:
                mem["frames"] -= 1
            else:
                mem["box"] = None

        is_locked = mem["consecutive"] >= color_lock_frames(color_name)
        trigger_enabled = run_enabled_event.is_set() or color_name != "Yellow"
        cooldown_remaining = max(0.0, color_cooldowns[color_name] - now)
        if is_locked and trigger_enabled and cooldown_remaining <= 0.0 and mem["area"] >= locked_area:
            locked_color = color_name
            locked_area = mem["area"]
        elif is_locked and trigger_enabled and cooldown_remaining > 0.0:
            cooldown_text.append(f"{color_name}:{cooldown_remaining:.1f}s")

        if mem["box"] is not None and mem["frames"] > 0:
            visual_items.append((color_name, mem["box"], mem["area"], is_locked, cooldown_remaining))

    if locked_color != "None":
        hold_seconds = color_trigger_cooldown(locked_color)
        color_cooldowns[locked_color] = now + hold_seconds
    set_locked_color(locked_color)
    if locked_color != "None":
        set_vision_detail(f"trigger={locked_color} hold={hold_seconds:.1f}s")
    elif cooldown_text:
        set_vision_detail(f"cooldown {' '.join(cooldown_text)}")
    else:
        set_vision_detail("color=None")
    return visual_items


def camera_worker(pi):
    global latest_jpeg
    cap = cv2.VideoCapture(CAMERA_INDEX)
    if not cap.isOpened():
        set_status("[camera] failed to open")
        shutdown_event.set()
        return
    camera_ready_event.set()
    set_status("[camera] ready")
    previous_feature_gray = None
    try:
        while not shutdown_event.is_set():
            ok, frame = cap.read()
            if not ok or frame is None:
                time.sleep(0.03)
                continue
            result, roi_overlay = analyze_line(frame)
            visual_items = update_color_memory(frame)
            h, w = frame.shape[:2]
            roi_y = int(h * (1.0 - LINE_ROI_HEIGHT))
            current_feature_gray = cv2.cvtColor(frame[roi_y:, :], cv2.COLOR_BGR2GRAY)
            result.update(compute_feature_metrics(previous_feature_gray, current_feature_gray))
            previous_feature_gray = current_feature_gray
            locked = latest_locked_color
            if locked == "Yellow":
                if not is_yellow_paused():
                    set_yellow_pause(True)
                if not servo_is_alert:
                    set_servo_alert(pi)
            elif locked == "Green" and is_yellow_paused():
                set_yellow_pause(False)
                if servo_is_alert:
                    set_servo_home(pi)
            with state_lock:
                line_state.update(result)
                current_status = status_text
                front_text = sensor_snapshot["front"]
                left_text = sensor_snapshot["left"]
                right_text = sensor_snapshot["right"]
                detail_text = vision_detail_text
            overlay_resized = cv2.resize(roi_overlay, (w, h - roi_y))
            frame[roi_y:, :] = cv2.addWeighted(frame[roi_y:, :], 0.55, overlay_resized, 0.45, 0.0)
            cv2.rectangle(frame, (0, roi_y), (w - 1, h - 1), (255, 255, 0), 2)
            cv2.line(frame, (w // 2, roi_y), (w // 2, h - 1), (255, 255, 255), 1)
            for color_name, box, area, is_locked, cooldown_remaining in visual_items:
                bx, by, bw, bh = box
                color = COLOR_BOXES[color_name]
                cooldown_label = f" cd={cooldown_remaining:.1f}s" if cooldown_remaining > 0.0 else ""
                label = f"{color_name} {'[LOCKED]' if is_locked else '...'} {int(area)}{cooldown_label}"
                cv2.rectangle(frame, (bx, by), (bx + bw, by + bh), color, 2 if is_locked else 1)
                cv2.putText(frame, label, (bx, max(18, by - 6)), cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 2)
            texts = [
                current_status,
                f"COLOR {detail_text}",
                f"LINE {result['mode']} found={result['found']} conf={float(result['confidence']):.2f}",
                f"OFFSET={float(result['offset']):+.2f} HEADING={float(result['heading']):+.2f}",
                f"ANGLE={float(result['corridor_angle_deg']):+.1f} VP={float(result['vanishing_offset']):+.2f}",
                f"FEAT M:{float(result['feature_motion_px']):.2f} R:{float(result['feature_rotation_deg']):+.1f} {'S' if result['feature_stable'] else 'U'}",
                f"CAM SIDE L:{float(result['left_open']):.2f}{'B' if result['left_blocked'] else ''} R:{float(result['right_open']):.2f}{'B' if result['right_blocked'] else ''}",
                f"ULTRA F:{front_text} L:{left_text} R:{right_text}",
            ]
            for idx, text in enumerate(texts):
                cv2.putText(frame, text, (10, 24 + (idx * 24)), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            ok, buffer = cv2.imencode('.jpg', frame)
            if ok:
                with state_lock:
                    latest_jpeg = buffer.tobytes()
            time.sleep(0.02)
    finally:
        cap.release()


def get_line_correction():
    with state_lock:
        found = bool(line_state["found"])
        mode = str(line_state["mode"])
        offset = float(line_state["offset"])
        heading = float(line_state["heading"])
        confidence = float(line_state["confidence"])
    if not found:
        return 0.0, "camera-none"
    correction = ((offset * LINE_OFFSET_KP) + (heading * LINE_HEADING_KD)) * max(confidence, 0.25)
    return clamp(correction, -LINE_MAX_ADJUST, LINE_MAX_ADJUST), f"camera-{mode}"


def get_camera_turn_alignment():
    with state_lock:
        found = bool(line_state["found"])
        mode = str(line_state["mode"])
        offset = float(line_state["offset"])
        heading = float(line_state["heading"])
        confidence = float(line_state["confidence"])
        left_open = float(line_state["left_open"])
        right_open = float(line_state["right_open"])
        left_blocked = bool(line_state["left_blocked"])
        right_blocked = bool(line_state["right_blocked"])
        corridor_angle_deg = float(line_state["corridor_angle_deg"])
        vanishing_offset = float(line_state["vanishing_offset"])
        geometry_confidence = float(line_state["geometry_confidence"])
        feature_motion_px = float(line_state["feature_motion_px"])
        feature_rotation_deg = float(line_state["feature_rotation_deg"])
        feature_confidence = float(line_state["feature_confidence"])
        feature_stable = bool(line_state["feature_stable"])

    if not found or confidence < TURN_CAMERA_MIN_CONFIDENCE:
        return {
            "usable": False,
            "aligned": False,
            "direction": None,
            "reason": f"camera-weak mode={mode} conf={confidence:.2f}",
        }

    turn_signal = (
        offset
        + (heading * TURN_CAMERA_ALIGN_GAIN)
        + vanishing_offset
        + (corridor_angle_deg / 45.0)
    )
    side_bias = right_open - left_open
    geometry_aligned = (
        geometry_confidence < TURN_CAMERA_MIN_CONFIDENCE
        or (
            abs(corridor_angle_deg) <= TURN_CAMERA_CORRIDOR_ANGLE_OK_DEG
            and abs(vanishing_offset) <= TURN_CAMERA_VANISH_OFFSET_OK
        )
    )
    feature_ok = (
        feature_confidence < FEATURE_CONFIDENCE_MIN
        or (
            feature_stable
            and abs(feature_rotation_deg) <= FEATURE_ROTATION_OK_DEG
            and feature_motion_px <= FEATURE_STABLE_MOTION_PX
        )
    )
    if (
        abs(offset) <= TURN_CAMERA_OFFSET_OK
        and abs(heading) <= TURN_CAMERA_HEADING_OK
        and not (left_blocked ^ right_blocked)
        and abs(side_bias) <= TURN_CAMERA_SIDE_DIFF_THRESHOLD
        and geometry_aligned
        and feature_ok
    ):
        return {
            "usable": True,
            "aligned": True,
            "direction": None,
            "reason": (
                f"camera-aligned mode={mode} off={offset:+.2f} "
                f"head={heading:+.2f} side={side_bias:+.2f} "
                f"ang={corridor_angle_deg:+.1f} vp={vanishing_offset:+.2f} "
                f"feat={feature_rotation_deg:+.1f}/{feature_motion_px:.2f} conf={confidence:.2f}"
            ),
        }

    if not feature_ok:
        return {
            "usable": True,
            "aligned": False,
            "direction": "right" if turn_signal > 0 else "left",
            "reason": (
                f"camera-feature-unstable mode={mode} rot={feature_rotation_deg:+.1f} "
                f"motion={feature_motion_px:.2f} conf={feature_confidence:.2f}"
            ),
        }

    if left_blocked and not right_blocked:
        return {
            "usable": True,
            "aligned": False,
            "direction": "right",
            "reason": (
                f"camera-left-angle-blocked mode={mode} "
                f"L={left_open:.2f} R={right_open:.2f}"
            ),
        }

    if right_blocked and not left_blocked:
        return {
            "usable": True,
            "aligned": False,
            "direction": "left",
            "reason": (
                f"camera-right-angle-blocked mode={mode} "
                f"L={left_open:.2f} R={right_open:.2f}"
            ),
        }

    if abs(side_bias) > TURN_CAMERA_SIDE_DIFF_THRESHOLD:
        turn_signal += side_bias

    return {
        "usable": True,
        "aligned": False,
        "direction": "right" if turn_signal > 0 else "left",
        "reason": (
            f"camera-correct mode={mode} off={offset:+.2f} "
            f"head={heading:+.2f} side={side_bias:+.2f} "
            f"ang={corridor_angle_deg:+.1f} vp={vanishing_offset:+.2f} "
            f"feat={feature_rotation_deg:+.1f}/{feature_motion_px:.2f} conf={confidence:.2f}"
        ),
    }


def perform_turn(direction, duration_s, pi, duty):
    if direction == "left":
        left(pi, duty)
    elif direction == "right":
        right(pi, duty)
    else:
        raise ValueError(f"Unknown turn direction: {direction}")
    remaining = duration_s
    while remaining > 0 and not shutdown_event.is_set():
        if motion_restart_requested():
            stop(pi)
            return "restart"
        if not wait_until_run_enabled(pi):
            return "paused"
        if direction == "left":
            left(pi, duty)
        else:
            right(pi, duty)
        slice_started = time.perf_counter()
        time.sleep(0.02)
        remaining -= time.perf_counter() - slice_started
    stop(pi)
    return "ok"


def choose_micro_direction(intended_direction, front_cm, left_cm, right_cm):
    if front_cm is not None and front_cm < TURN_ALIGN_FRONT_CLEAR_CM:
        return False, intended_direction, "front-not-clear"

    camera_alignment = get_camera_turn_alignment()
    if camera_alignment["usable"]:
        if camera_alignment["aligned"]:
            return True, None, camera_alignment["reason"]
        return False, camera_alignment["direction"], camera_alignment["reason"]

    error, mode = compute_passage_error(left_cm, right_cm)
    if mode == "center" and error is not None:
        if abs(error) <= TURN_ALIGN_BOTH_ERROR_CM:
            return True, None, "both-sides-aligned"
        return False, ("right" if error > 0 else "left"), f"both-sides-correct err={error:.2f}"
    if mode == "right-wall" and right_cm is not None:
        delta = right_cm - RIGHT_REFERENCE_CM
        if abs(delta) <= TURN_ALIGN_SINGLE_TOL_CM:
            return True, None, "right-wall-aligned"
        return False, ("right" if delta > 0 else "left"), f"right-wall-correct delta={delta:.2f}"
    if mode == "left-wall" and left_cm is not None:
        delta = left_cm - LEFT_REFERENCE_CM
        if abs(delta) <= TURN_ALIGN_SINGLE_TOL_CM:
            return True, None, "left-wall-aligned"
        return False, ("left" if delta > 0 else "right"), f"left-wall-correct delta={delta:.2f}"
    if front_cm is None or front_cm >= TURN_ALIGN_FRONT_CLEAR_CM:
        return True, None, "front-clear-no-side"
    return False, intended_direction, "fallback-turn"


def front_clear_for_turn(pi, debug):
    front_avg = average_distance(pi, "front", TURN_FRONT_CHECK_SECONDS, debug=debug)
    clear = front_avg is None or front_avg >= TURN_ALIGN_FRONT_CLEAR_CM
    return clear, front_avg


def verify_turn_before_mini(direction, pi, debug):
    if not wait_until_run_enabled(pi):
        return True

    print(
        f"Verifying post-turn distances for {TURN_PRE_MINI_VERIFY_SECONDS:.1f}s "
        "before mini-pulse correction"
    )
    front_avg = average_distance(pi, "front", TURN_PRE_MINI_VERIFY_SECONDS, debug=debug)
    left_avg = average_distance(pi, "left", TURN_PRE_MINI_VERIFY_SECONDS * 0.7, debug=debug)
    right_avg = average_distance(pi, "right", TURN_PRE_MINI_VERIFY_SECONDS * 0.7, debug=debug)
    aligned, pulse_direction, reason = choose_micro_direction(direction, front_avg, left_avg, right_avg)
    print(
        "[pre-mini] "
        f"front={format_distance(front_avg)} "
        f"left={format_distance(left_avg)} "
        f"right={format_distance(right_avg)} -> {reason}"
    )
    if aligned or pulse_direction is None:
        return True
    return False


def mini_adjust_after_turn(direction, pi, debug):
    started_at = time.perf_counter()
    step = 0
    while (time.perf_counter() - started_at) < TURN_MINI_MAX_TOTAL_SECONDS and not shutdown_event.is_set():
        step += 1
        if motion_restart_requested():
            return "restart"
        if not wait_until_run_enabled(pi):
            return "paused"
        front_avg = average_distance(pi, "front", 0.15, debug=debug)
        left_avg = average_distance(pi, "left", 0.12, debug=debug)
        right_avg = average_distance(pi, "right", 0.12, debug=debug)
        aligned, pulse_direction, reason = choose_micro_direction(direction, front_avg, left_avg, right_avg)
        print(f"[mini-turn {step}] front={format_distance(front_avg)} left={format_distance(left_avg)} right={format_distance(right_avg)} -> {reason}")
        if aligned or pulse_direction is None:
            return "aligned"
        clear_now, front_check = front_clear_for_turn(pi, debug)
        if not clear_now:
            print(f"[mini-turn {step}] front still not clear ({format_distance(front_check)}), pulsing {direction} with stronger turn power")
            pulse_direction = direction
        pulse_seconds = TURN_MINI_FIRST_PULSE_S if step == 1 else TURN_MINI_DOUBLE_PULSE_S
        result = perform_turn(pulse_direction, pulse_seconds, pi, TURN_MINI_PWM_DUTY)
        if result != "ok":
            return result
        time.sleep(TURN_MINI_SETTLE_S)
    print(f"[mini-turn] correction window ended after {time.perf_counter() - started_at:.2f}s")
    return "timeout"


def verify_side_before_turn(direction, pi, debug):
    side_avg = average_distance(pi, direction, PRE_TURN_VERIFY_SECONDS, debug=debug)
    print(f"Pre-turn {direction} check -> {format_distance(side_avg)}")


def stabilize_after_turn(pi, debug):
    front_avg = average_distance(pi, "front", POST_TURN_STABILIZE_SECONDS, debug=debug)
    print(f"Post-turn front check -> {format_distance(front_avg)}")


def execute_turn_with_micro(direction, duration_s, pi, debug):
    if not wait_until_run_enabled(pi):
        return "paused"
    verify_side_before_turn(direction, pi, debug)
    clear_now, front_check = front_clear_for_turn(pi, debug)
    if not clear_now:
        print(f"Pre-turn front not clear ({format_distance(front_check)}). Executing direct timed turn.")
    print(f"{direction} timed turn for {duration_s:.2f}s")
    result = perform_turn(direction, duration_s, pi, TURN_PWM_DUTY)
    if result != "ok":
        return result
    time.sleep(TURN_MINI_SETTLE_S)
    stabilize_after_turn(pi, debug)
    return "ok"


def run_forward_with_camera(duration_s, pi, debug):
    global resume_boost_until
    front_filtered = left_filtered = right_filtered = None
    integral = 0.0
    previous_error = 0.0
    coarse_recovery = False
    loop_started_at = motion_started_at = time.perf_counter()
    last_debug_at = 0.0
    while not shutdown_event.is_set():
        if motion_restart_requested():
            stop(pi)
            return "restart"
        if not wait_until_run_enabled(pi):
            return "paused"
        if duration_s is not None and (time.perf_counter() - motion_started_at) >= duration_s:
            stop(pi)
            return "completed"
        front_filtered = smooth_distance(front_filtered, get_distance_cm(pi, "front", debug=debug))
        left_filtered = smooth_distance(left_filtered, get_distance_cm(pi, "left", debug=debug))
        right_filtered = smooth_distance(right_filtered, get_distance_cm(pi, "right", debug=debug))
        set_sensor_snapshot(front_filtered, left_filtered, right_filtered)
        if front_filtered is not None and front_filtered < FRONT_STOP_CM:
            stop(pi)
            print(f"Front blocked at {front_filtered:.1f} cm")
            return "front_blocked"
        side_values = [dist for dist in (left_filtered, right_filtered) if dist is not None]
        if side_values and min(side_values) < SIDE_HARD_LIMIT_CM:
            stop(pi)
            print(f"Side clearance dropped below {SIDE_HARD_LIMIT_CM:.1f} cm (left={format_distance(left_filtered)}, right={format_distance(right_filtered)})")
            return "side_limit"
        error, mode = compute_passage_error(left_filtered, right_filtered)
        side_delta = compute_visible_side_delta(left_filtered, right_filtered)
        front_allows_coarse = front_filtered is None or front_filtered >= FRONT_COARSE_VERIFY_CM
        if coarse_recovery:
            if (abs(error) <= COARSE_RECOVERY_EXIT_ERROR and abs(side_delta) <= COARSE_RECOVERY_EXIT_DELTA) or not front_allows_coarse:
                coarse_recovery = False
        elif front_allows_coarse and (abs(error) >= COARSE_RECOVERY_ENTER_ERROR or abs(side_delta) >= COARSE_RECOVERY_ENTER_DELTA):
            coarse_recovery = True
        now = time.perf_counter()
        dt = max(now - loop_started_at, 0.001)
        loop_started_at = now
        if now < resume_boost_until:
            integral = 0.0
            previous_error = 0.0
            left_duty = clamp(RESUME_STRAIGHT_PWM + LEFT_MOTOR_TRIM_PWM, MIN_DRIVE_PWM_DUTY, MAX_DRIVE_PWM_DUTY)
            right_duty = clamp(RESUME_STRAIGHT_PWM + RIGHT_MOTOR_TRIM_PWM, MIN_DRIVE_PWM_DUTY, MAX_DRIVE_PWM_DUTY)
            apply_drive(pi, left_duty, right_duty)
            if debug and (now - last_debug_at) >= PID_DEBUG_PRINT_INTERVAL_S:
                print(f"[resume-smooth] front={format_distance(front_filtered)} left={format_distance(left_filtered)} right={format_distance(right_filtered)} pwm=({left_duty:.0f},{right_duty:.0f})")
                last_debug_at = now
            sleep_for = PID_LOOP_DT_S - (time.perf_counter() - now)
            if sleep_for > 0:
                time.sleep(sleep_for)
            continue
        if mode == "trim-only":
            integral = previous_error = 0.0
        elif error == 0.0 or (previous_error != 0.0 and (error > 0.0) != (previous_error > 0.0)):
            integral = 0.0
        integral_input = error * (POSITIVE_INTEGRAL_GAIN if error > 0.0 else 1.0)
        integral = clamp(integral + (integral_input * dt), -PID_INTEGRAL_LIMIT, PID_INTEGRAL_LIMIT)
        derivative = 0.0 if mode == "trim-only" else (error - previous_error) / dt
        kp, ki, kd, right_boost, min_adjust = get_pid_profile(front_filtered, mode, coarse_recovery)
        ultra_control = 0.0 if mode == "trim-only" else ((kp * error) + (ki * integral) + (kd * derivative))
        ultra_adjust = 0.0 if mode == "trim-only" else clamp(ultra_control, -PID_MAX_LEFT_ADJUST, PID_MAX_RIGHT_ADJUST)
        if ultra_adjust > 0:
            ultra_adjust = clamp(ultra_adjust * right_boost, -PID_MAX_LEFT_ADJUST, PID_MAX_RIGHT_ADJUST)
        if min_adjust > 0.0 and error != 0.0 and abs(ultra_adjust) < min_adjust:
            ultra_adjust = min_adjust if ultra_adjust >= 0.0 else -min_adjust
        camera_adjust = 0.0
        camera_mode = "camera-disabled"
        combined_adjust = ultra_adjust
        base_duty = choose_forward_base_duty(front_filtered, left_filtered, right_filtered)
        left_duty = clamp(base_duty + LEFT_MOTOR_TRIM_PWM + combined_adjust, MIN_DRIVE_PWM_DUTY, MAX_DRIVE_PWM_DUTY)
        right_duty = clamp(base_duty + RIGHT_MOTOR_TRIM_PWM - combined_adjust, MIN_DRIVE_PWM_DUTY, MAX_DRIVE_PWM_DUTY)
        apply_drive(pi, left_duty, right_duty)
        previous_error = error
        if debug and (now - last_debug_at) >= PID_DEBUG_PRINT_INTERVAL_S:
            print(f"[pid-ultra] mode={mode}{'+coarse' if coarse_recovery else ''} {camera_mode} front={format_distance(front_filtered)} left={format_distance(left_filtered)} right={format_distance(right_filtered)} err={error:.2f} delta={side_delta:.2f} ultra={ultra_adjust:.1f} adj={combined_adjust:.1f} pwm=({left_duty:.0f},{right_duty:.0f})")
            last_debug_at = now
        sleep_for = PID_LOOP_DT_S - (time.perf_counter() - now)
        if sleep_for > 0:
            time.sleep(sleep_for)
    stop(pi)
    return "stopped"


def execute_step(direction, duration_s, pi, debug):
    direction = direction.strip().lower()
    if direction == "center":
        direction = "straight"
    if not wait_until_run_enabled(pi):
        return "paused"
    if direction in ("forward", "straight"):
        print(f"{direction} with ultrasonic side PID" + (" until front stop" if duration_s is None else f" for {duration_s:.1f}s"))
        return run_forward_with_camera(duration_s, pi, debug=debug)
    if duration_s is None:
        raise ValueError(f"Timed direction requires a duration: {direction}")
    if direction in ("left", "right"):
        return execute_turn_with_micro(direction, duration_s, pi, debug=debug)
    print(f"{direction} for {duration_s:.1f}s")
    if direction == "backward":
        backward(pi, BASE_TIGHT_PWM_DUTY)
    elif direction == "stop":
        stop(pi)
    else:
        raise ValueError(f"Unknown direction: {direction}")
    start = time.perf_counter()
    while time.perf_counter() - start < duration_s and not shutdown_event.is_set():
        if motion_restart_requested():
            stop(pi)
            return "restart"
        time.sleep(0.02)
    stop(pi)
    return "ok"


def navigation_worker(pi, debug):
    current_step_index = 0
    try:
        while not shutdown_event.is_set():
            if motion_restart_requested():
                current_step_index = 0
                restart_requested_event.clear()
                set_status("[nav] restart requested - path reset to step 0")

            path_steps = get_path_steps()
            if current_step_index >= len(path_steps):
                run_enabled_event.clear()
                set_status("[nav] path completed")
                time.sleep(0.05)
                continue

            if not wait_until_run_enabled(pi):
                break

            if motion_restart_requested():
                continue

            name, dur = path_steps[current_step_index]
            set_status(f"[nav] step={current_step_index + 1}/{len(path_steps)} {name} duration={dur}")
            result = execute_step(name, dur, pi, debug)
            if result == "restart":
                current_step_index = 0
                restart_requested_event.clear()
                continue
            if result == "paused":
                continue

            current_step_index += 1
            time.sleep(RESUME_PAUSE_S)
    finally:
        stop(pi)


def frame_generator():
    while not shutdown_event.is_set():
        with state_lock:
            frame = latest_jpeg
        if frame is None:
            time.sleep(0.05)
            continue
        yield (b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
        time.sleep(0.03)


@app.route('/')
def index():
    return '''
<html>
<body style="background:#000;color:#0f0;font-family:monospace;text-align:center">
<h3>INITIAL PATH CAMERA</h3>
<div id="ui_status" style="margin:8px 0;color:#8f8">Ready</div>
<div id="pause_state" style="margin:6px 0;color:#ccc">Checking control state...</div>
<div style="margin:12px 0">
  <button onclick="sendCmd('/start')" style="color:#0f0;background:#111;border:1px solid #0f0;padding:8px 16px;margin:0 12px">START</button>
  <button onclick="sendCmd('/pause')" style="color:#ff0;background:#111;border:1px solid #ff0;padding:8px 16px;margin:0 12px">PAUSE</button>
  <button onclick="sendCmd('/restart')" style="color:#0ff;background:#111;border:1px solid #0ff;padding:8px 16px;margin:0 12px">RESTART</button>
  <a href="/stop_server" style="color:#f66;margin:0 12px">SHUTDOWN</a>
</div>
<div style="margin:14px auto;padding:12px;max-width:900px;border:1px solid #333;background:#0a0a0a">
  <div style="margin-bottom:8px;color:#9ff">Paused-only editor</div>
  <div style="margin-bottom:10px">
    <label for="turn_seconds">Turn seconds</label>
    <input id="turn_seconds" class="pause-only" type="number" min="0.1" max="10" step="0.05" style="width:90px;margin:0 8px;background:#111;color:#0f0;border:1px solid #355;padding:6px">
    <button class="pause-only" onclick="setTurnSeconds()" style="color:#9ff;background:#111;border:1px solid #0ff;padding:6px 12px">APPLY TURN</button>
  </div>
  <div style="margin-bottom:10px">
    <button class="pause-only" onclick="appendPathStep('straight')" style="color:#0f0;background:#111;border:1px solid #0f0;padding:6px 12px;margin:0 6px">STRAIGHT</button>
    <button class="pause-only" onclick="appendPathStep('left')" style="color:#ff0;background:#111;border:1px solid #ff0;padding:6px 12px;margin:0 6px">LEFT</button>
    <button class="pause-only" onclick="appendPathStep('right')" style="color:#0ff;background:#111;border:1px solid #0ff;padding:6px 12px;margin:0 6px">RIGHT</button>
    <button class="pause-only" onclick="appendPathStep('center')" style="color:#f9f;background:#111;border:1px solid #f9f;padding:6px 12px;margin:0 6px">CENTER</button>
  </div>
  <div style="margin-bottom:10px">
    <button class="pause-only" onclick="clearPath()" style="color:#f66;background:#111;border:1px solid #f66;padding:6px 12px;margin:0 6px">CLEAR PATH</button>
    <button class="pause-only" onclick="resetDefaultPath()" style="color:#fff;background:#111;border:1px solid #888;padding:6px 12px;margin:0 6px">RESET DEFAULT</button>
  </div>
  <div id="path_view" style="color:#ddd;word-break:break-word">Path: loading...</div>
</div>
<img src="/video_feed" style="max-width:90%;border:2px solid #222">
<script>
async function sendCmd(path) {
  const el = document.getElementById("ui_status");
  el.textContent = "Sending command...";
  try {
    const res = await fetch(path, {method: "POST"});
    const text = await res.text();
    el.textContent = text || "OK";
  } catch (err) {
    el.textContent = "Command failed";
  }
  await loadState();
}

function setPauseOnlyEnabled(paused) {
  const controls = document.querySelectorAll(".pause-only");
  controls.forEach((node) => {
    node.disabled = !paused;
    node.style.opacity = paused ? "1" : "0.45";
    node.style.cursor = paused ? "pointer" : "not-allowed";
  });
  document.getElementById("pause_state").textContent = paused
    ? "Robot paused. Route and turn timing can be edited."
    : "Robot running. Pause first to edit route and turn timing.";
}

async function loadState() {
  try {
    const res = await fetch("/control_state");
    const data = await res.json();
    setPauseOnlyEnabled(data.paused);
    document.getElementById("turn_seconds").value = Number(data.turn_seconds).toFixed(2);
    document.getElementById("path_view").textContent = "Path: " + (data.path_sequence.length ? data.path_sequence.join(" -> ") : "(empty)");
  } catch (err) {
    document.getElementById("ui_status").textContent = "Failed to load control state";
  }
}

async function postJson(path, payload) {
  const el = document.getElementById("ui_status");
  el.textContent = "Applying edit...";
  try {
    const res = await fetch(path, {
      method: "POST",
      headers: {"Content-Type": "application/json"},
      body: JSON.stringify(payload || {})
    });
    const text = await res.text();
    el.textContent = text || "OK";
  } catch (err) {
    el.textContent = "Edit failed";
  }
  await loadState();
}

async function setTurnSeconds() {
  const value = Number(document.getElementById("turn_seconds").value);
  await postJson("/set_turn_seconds", {seconds: value});
}

async function appendPathStep(step) {
  await postJson("/path/add", {step: step});
}

async function clearPath() {
  await postJson("/path/clear", {});
}

async function resetDefaultPath() {
  await postJson("/path/reset_default", {});
}

loadState();
</script>
</body>
</html>
'''


@app.route('/start', methods=['POST'])
def start_robot():
    run_enabled_event.set()
    set_status("[nav] start requested from web")
    return "Robot start enabled."


@app.route('/pause', methods=['POST'])
def pause_robot():
    run_enabled_event.clear()
    set_status("[nav] pause requested from web")
    return "Robot paused."


@app.route('/restart', methods=['POST'])
def restart_robot():
    restart_requested_event.set()
    run_enabled_event.set()
    set_status("[nav] restart requested from web")
    return "Robot restart requested."


@app.route('/control_state')
def control_state():
    return jsonify(get_control_state())


@app.route('/set_turn_seconds', methods=['POST'])
def set_turn_seconds():
    blocked = edit_requires_pause()
    if blocked is not None:
        return blocked
    payload = request.get_json(silent=True) or {}
    try:
        seconds = float(payload.get("seconds", 0.0))
    except (TypeError, ValueError):
        return Response("Invalid turn seconds.", status=400)
    if not 0.1 <= seconds <= 10.0:
        return Response("Turn seconds must be between 0.1 and 10.0.", status=400)
    update_turn_seconds(seconds)
    mark_path_edited(f"[nav] turn timing updated to {seconds:.2f}s while paused")
    return f"Turn timing updated to {seconds:.2f}s. Press START or RESTART when ready."


@app.route('/path/add', methods=['POST'])
def path_add():
    blocked = edit_requires_pause()
    if blocked is not None:
        return blocked
    payload = request.get_json(silent=True) or {}
    step_name = str(payload.get("step", "")).strip().lower()
    if step_name not in {"straight", "left", "right", "center"}:
        return Response("Invalid path step.", status=400)
    append_path_step(step_name)
    mark_path_edited(f"[nav] path step appended: {step_name}")
    return f"Added path step: {step_name}. Path will restart from step 0 on next START."


@app.route('/path/clear', methods=['POST'])
def path_clear():
    blocked = edit_requires_pause()
    if blocked is not None:
        return blocked
    clear_path_sequence()
    mark_path_edited("[nav] path cleared from web")
    return "Path cleared. Add new steps before START."


@app.route('/path/reset_default', methods=['POST'])
def path_reset_default():
    blocked = edit_requires_pause()
    if blocked is not None:
        return blocked
    reset_default_path_sequence()
    mark_path_edited("[nav] path reset to default from web")
    return "Path reset to default. Press START or RESTART when ready."


@app.route('/video_feed')
def video_feed():
    return Response(frame_generator(), mimetype='multipart/x-mixed-replace; boundary=frame')


@app.route('/stop_server')
def stop_server():
    run_enabled_event.clear()
    shutdown_event.set()
    os.kill(os.getpid(), signal.SIGINT)
    return 'Shutdown complete.'


def main():
    debug = '--debug' in sys.argv
    pi = pigpio.pi()
    if not pi.connected:
        print('pigpio daemon not running or not reachable.', file=sys.stderr)
        sys.exit(1)
    setup_motors(pi)
    setup_ultrasonic(pi)
    setup_servo(pi)
    stop(pi)
    camera_thread = threading.Thread(target=camera_worker, args=(pi,), daemon=True)
    nav_thread = threading.Thread(target=navigation_worker, args=(pi, debug), daemon=True)
    camera_thread.start()
    if not camera_ready_event.wait(timeout=5.0):
        shutdown_event.set()
        raise RuntimeError('Camera thread did not become ready.')
    nav_thread.start()
    print(f'initial_path_camera.py ready on port {STREAM_PORT}.')
    try:
        app.run(host='0.0.0.0', port=STREAM_PORT, threaded=True)
    except KeyboardInterrupt:
        print('\nStopped by user')
    finally:
        shutdown_event.set()
        stop(pi)
        set_servo_home(pi)
        release_servo(pi)
        cleanup_ultrasonic(pi)
        pi.stop()


if __name__ == '__main__':
    main()
