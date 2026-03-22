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
from flask import Flask, Response

app = Flask(__name__)

IN1, IN2, IN3, IN4, ENA, ENB = 17, 27, 22, 23, 13, 12
PWM_HZ = 1000
FRONT_TRIG, FRONT_ECHO = 18, 19
LEFT_TRIG, LEFT_ECHO = 24, 25
RIGHT_TRIG, RIGHT_ECHO = 4, 26

BASE_CRUISE_PWM_DUTY = 56
BASE_TIGHT_PWM_DUTY = 40
BASE_CRAWL_PWM_DUTY = 32
TURN_PWM_DUTY = 90
TURN_MICRO_PWM_DUTY = 84
MIN_DRIVE_PWM_DUTY = 20
MAX_DRIVE_PWM_DUTY = 65
LEFT_MOTOR_TRIM_PWM = 3.0
RIGHT_MOTOR_TRIM_PWM = 0.0

FRONT_STOP_CM = 10.0
FRONT_SLOW_CM = 18.0
TURN_OPENING_CLEAR_CM = 10.0
LEFT_REFERENCE_CM = 8.6
RIGHT_REFERENCE_CM = 7.8
PASSAGE_TIGHT_SIDE_CM = 5.0
SIDE_HARD_LIMIT_CM = 2.5
SIDE_PID_ACTIVE_MAX_CM = 20.0
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
TURN_SECONDS = 2.0
TURN_COARSE_RATIO = 0.78
TURN_MICRO_SETTLE_S = 0.05
TURN_MICRO_FIRST_PULSE_S = 0.06
TURN_MICRO_DOUBLE_PULSE_S = 0.12
TURN_MICRO_MAX_TOTAL_SECONDS = 2.4
TURN_FRONT_CHECK_SECONDS = 0.12
TURN_PRE_MICRO_VERIFY_SECONDS = 1.2
RESUME_PAUSE_S = 0.20

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

PATH_TURN_SECONDS = 2.0
PATH_BACKWARD_SECONDS = 1.0
PATH_SEQUENCE = [
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
STEP_DURATION_BY_DIRECTION = {
    "left": PATH_TURN_SECONDS,
    "right": PATH_TURN_SECONDS,
    "backward": PATH_BACKWARD_SECONDS,
}
SENSORS = {"front": {"trig": FRONT_TRIG, "echo": FRONT_ECHO, "last_trigger": 0.0}, "left": {"trig": LEFT_TRIG, "echo": LEFT_ECHO, "last_trigger": 0.0}, "right": {"trig": RIGHT_TRIG, "echo": RIGHT_ECHO, "last_trigger": 0.0}}

state_lock = threading.Lock()
shutdown_event = threading.Event()
camera_ready_event = threading.Event()
run_enabled_event = threading.Event()
restart_requested_event = threading.Event()
latest_jpeg = None
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
}
sensor_snapshot = {"front": "invalid", "left": "invalid", "right": "invalid"}
status_text = "idle-waiting-start"


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


def get_path_steps():
    return [(direction, STEP_DURATION_BY_DIRECTION.get(direction)) for direction in PATH_SEQUENCE]


def motion_restart_requested():
    return restart_requested_event.is_set()


def wait_until_run_enabled(pi):
    while not shutdown_event.is_set() and not run_enabled_event.is_set():
        stop(pi)
        set_status("[nav] paused - press START")
        time.sleep(0.05)
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
    edges = cv2.Canny(blur, LINE_EDGE_CANNY_LOW, LINE_EDGE_CANNY_HIGH)
    overlay = cv2.cvtColor(edges, cv2.COLOR_GRAY2BGR)
    lines = cv2.HoughLinesP(edges, 1, np.pi / 180.0, LINE_HOUGH_THRESHOLD, minLineLength=LINE_HOUGH_MIN_LENGTH, maxLineGap=LINE_HOUGH_MAX_GAP)
    left_edges, right_edges = [], []
    if lines is not None:
        for line in lines:
            x1, y1, x2, y2 = line[0]
            bottom_x = line_bottom_x(x1, y1, x2, y2, roi.shape[0] - 1.0)
            if bottom_x is None:
                continue
            mid_x = (x1 + x2) / 2.0
            cv2.line(overlay, (x1, y1), (x2, y2), (255, 0, 0) if mid_x < (w / 2.0) else (0, 0, 255), 2)
            if mid_x < (w / 2.0):
                left_edges.append(bottom_x)
            else:
                right_edges.append(bottom_x)
    lane_center, mode = None, "none"
    if left_edges and right_edges:
        lane_center = (median(left_edges) + median(right_edges)) / 2.0
        mode = "double-edge"
    elif left_edges:
        lane_center = median(left_edges) + LINE_SINGLE_EDGE_HALF_WIDTH_PX
        mode = "single-left"
    elif right_edges:
        lane_center = median(right_edges) - LINE_SINGLE_EDGE_HALF_WIDTH_PX
        mode = "single-right"
    if lane_center is not None:
        lane_center = clamp(lane_center, 0.0, float(w - 1))
        cv2.line(overlay, (int(lane_center), 0), (int(lane_center), roi.shape[0] - 1), (0, 255, 255), 2)
        result.update({"found": True, "mode": mode, "offset": clamp((lane_center - (w / 2.0)) / (w / 2.0), -1.0, 1.0), "heading": 0.0, "confidence": 0.55 if mode == "double-edge" else 0.35})
    return result, overlay


def camera_worker():
    global latest_jpeg
    cap = cv2.VideoCapture(CAMERA_INDEX)
    if not cap.isOpened():
        set_status("[camera] failed to open")
        shutdown_event.set()
        return
    camera_ready_event.set()
    set_status("[camera] ready")
    try:
        while not shutdown_event.is_set():
            ok, frame = cap.read()
            if not ok or frame is None:
                time.sleep(0.03)
                continue
            result, roi_overlay = analyze_line(frame)
            h, w = frame.shape[:2]
            roi_y = int(h * (1.0 - LINE_ROI_HEIGHT))
            with state_lock:
                line_state.update(result)
                current_status = status_text
                front_text = sensor_snapshot["front"]
                left_text = sensor_snapshot["left"]
                right_text = sensor_snapshot["right"]
            overlay_resized = cv2.resize(roi_overlay, (w, h - roi_y))
            frame[roi_y:, :] = cv2.addWeighted(frame[roi_y:, :], 0.55, overlay_resized, 0.45, 0.0)
            cv2.rectangle(frame, (0, roi_y), (w - 1, h - 1), (255, 255, 0), 2)
            cv2.line(frame, (w // 2, roi_y), (w // 2, h - 1), (255, 255, 255), 1)
            texts = [
                current_status,
                f"LINE {result['mode']} found={result['found']} conf={float(result['confidence']):.2f}",
                f"OFFSET={float(result['offset']):+.2f} HEADING={float(result['heading']):+.2f}",
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

    if not found or confidence < TURN_CAMERA_MIN_CONFIDENCE:
        return {
            "usable": False,
            "aligned": False,
            "direction": None,
            "reason": f"camera-weak mode={mode} conf={confidence:.2f}",
        }

    turn_signal = (offset + (heading * TURN_CAMERA_ALIGN_GAIN))
    side_bias = right_open - left_open
    if (
        abs(offset) <= TURN_CAMERA_OFFSET_OK
        and abs(heading) <= TURN_CAMERA_HEADING_OK
        and not (left_blocked ^ right_blocked)
        and abs(side_bias) <= TURN_CAMERA_SIDE_DIFF_THRESHOLD
    ):
        return {
            "usable": True,
            "aligned": True,
            "direction": None,
            "reason": (
                f"camera-aligned mode={mode} off={offset:+.2f} "
                f"head={heading:+.2f} side={side_bias:+.2f} conf={confidence:.2f}"
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
            f"head={heading:+.2f} side={side_bias:+.2f} conf={confidence:.2f}"
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


def verify_turn_before_micro(direction, pi, debug):
    if not wait_until_run_enabled(pi):
        return True

    print(
        f"Verifying post-turn distances for {TURN_PRE_MICRO_VERIFY_SECONDS:.1f}s "
        "before micro-correction"
    )
    front_avg = average_distance(pi, "front", TURN_PRE_MICRO_VERIFY_SECONDS, debug=debug)
    left_avg = average_distance(pi, "left", TURN_PRE_MICRO_VERIFY_SECONDS * 0.7, debug=debug)
    right_avg = average_distance(pi, "right", TURN_PRE_MICRO_VERIFY_SECONDS * 0.7, debug=debug)
    aligned, pulse_direction, reason = choose_micro_direction(direction, front_avg, left_avg, right_avg)
    print(
        "[pre-micro] "
        f"front={format_distance(front_avg)} "
        f"left={format_distance(left_avg)} "
        f"right={format_distance(right_avg)} -> {reason}"
    )
    if aligned or pulse_direction is None:
        return True
    return False


def micro_adjust_after_turn(direction, pi, debug):
    started_at = time.perf_counter()
    step = 0
    while (time.perf_counter() - started_at) < TURN_MICRO_MAX_TOTAL_SECONDS and not shutdown_event.is_set():
        step += 1
        if motion_restart_requested():
            return "restart"
        if not wait_until_run_enabled(pi):
            return "paused"
        front_avg = average_distance(pi, "front", 0.15, debug=debug)
        left_avg = average_distance(pi, "left", 0.12, debug=debug)
        right_avg = average_distance(pi, "right", 0.12, debug=debug)
        aligned, pulse_direction, reason = choose_micro_direction(direction, front_avg, left_avg, right_avg)
        print(f"[micro-turn {step}] front={format_distance(front_avg)} left={format_distance(left_avg)} right={format_distance(right_avg)} -> {reason}")
        if aligned or pulse_direction is None:
            return "aligned"
        clear_now, front_check = front_clear_for_turn(pi, debug)
        if not clear_now:
            print(f"[micro-turn {step}] front still not clear ({format_distance(front_check)}), pulsing {direction} with stronger turn power")
            pulse_direction = direction
        pulse_seconds = TURN_MICRO_FIRST_PULSE_S if step == 1 else TURN_MICRO_DOUBLE_PULSE_S
        result = perform_turn(pulse_direction, pulse_seconds, pi, TURN_MICRO_PWM_DUTY)
        if result != "ok":
            return result
        time.sleep(TURN_MICRO_SETTLE_S)
    print(f"[micro-turn] correction window ended after {time.perf_counter() - started_at:.2f}s")
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
    coarse_duration = duration_s * TURN_COARSE_RATIO
    clear_now, front_check = front_clear_for_turn(pi, debug)
    if not clear_now:
        print(f"Pre-turn front not clear ({format_distance(front_check)}). Applying stronger coarse turn.")
    print(f"{direction} coarse turn for {coarse_duration:.2f}s, then micro-adjust")
    result = perform_turn(direction, coarse_duration, pi, TURN_PWM_DUTY)
    if result != "ok":
        return result
    time.sleep(TURN_MICRO_SETTLE_S)
    if not verify_turn_before_micro(direction, pi, debug):
        result = micro_adjust_after_turn(direction, pi, debug)
        if result in ("restart", "paused"):
            return result
    stabilize_after_turn(pi, debug)
    return "ok"


def run_forward_with_camera(duration_s, pi, debug):
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
        camera_adjust, camera_mode = get_line_correction()
        combined_adjust = clamp(ultra_adjust + camera_adjust, -PID_MAX_LEFT_ADJUST, PID_MAX_RIGHT_ADJUST)
        base_duty = choose_forward_base_duty(front_filtered, left_filtered, right_filtered)
        left_duty = clamp(base_duty + LEFT_MOTOR_TRIM_PWM + combined_adjust, MIN_DRIVE_PWM_DUTY, MAX_DRIVE_PWM_DUTY)
        right_duty = clamp(base_duty + RIGHT_MOTOR_TRIM_PWM - combined_adjust, MIN_DRIVE_PWM_DUTY, MAX_DRIVE_PWM_DUTY)
        apply_drive(pi, left_duty, right_duty)
        previous_error = error
        if debug and (now - last_debug_at) >= PID_DEBUG_PRINT_INTERVAL_S:
            print(f"[pid-camera] mode={mode}{'+coarse' if coarse_recovery else ''} {camera_mode} front={format_distance(front_filtered)} left={format_distance(left_filtered)} right={format_distance(right_filtered)} err={error:.2f} delta={side_delta:.2f} ultra={ultra_adjust:.1f} cam={camera_adjust:.1f} adj={combined_adjust:.1f} pwm=({left_duty:.0f},{right_duty:.0f})")
            last_debug_at = now
        sleep_for = PID_LOOP_DT_S - (time.perf_counter() - now)
        if sleep_for > 0:
            time.sleep(sleep_for)
    stop(pi)
    return "stopped"


def execute_step(direction, duration_s, pi, debug):
    direction = direction.strip().lower()
    if not wait_until_run_enabled(pi):
        return "paused"
    if direction in ("forward", "straight"):
        print(f"{direction} with line assist + side PID" + (" until front stop" if duration_s is None else f" for {duration_s:.1f}s"))
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
    return (
        '<html><body style="background:#000;color:#0f0;font-family:monospace;text-align:center">'
        '<h3>INITIAL PATH CAMERA</h3>'
        '<div id="ui_status" style="margin:8px 0;color:#8f8">Ready</div>'
        '<div style="margin:12px 0">'
        '<button onclick="sendCmd(\'/start\')" style="color:#0f0;background:#111;border:1px solid #0f0;padding:8px 16px;margin:0 12px">START</button>'
        '<button onclick="sendCmd(\'/pause\')" style="color:#ff0;background:#111;border:1px solid #ff0;padding:8px 16px;margin:0 12px">PAUSE</button>'
        '<button onclick="sendCmd(\'/restart\')" style="color:#0ff;background:#111;border:1px solid #0ff;padding:8px 16px;margin:0 12px">RESTART</button>'
        '<a href="/stop_server" style="color:#f66;margin:0 12px">SHUTDOWN</a>'
        '</div>'
        '<img src="/video_feed" style="max-width:90%;border:2px solid #222">'
        '<script>'
        'async function sendCmd(path) {'
        '  const el = document.getElementById("ui_status");'
        '  el.textContent = "Sending command...";'
        '  try {'
        '    const res = await fetch(path, {method: "POST"});'
        '    const text = await res.text();'
        '    el.textContent = text || "OK";'
        '  } catch (err) {'
        '    el.textContent = "Command failed";'
        '  }'
        '}'
        '</script>'
        '</body></html>'
    )


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
    stop(pi)
    camera_thread = threading.Thread(target=camera_worker, daemon=True)
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
        cleanup_ultrasonic(pi)
        pi.stop()


if __name__ == '__main__':
    main()
