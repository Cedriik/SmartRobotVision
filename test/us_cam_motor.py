import os
os.environ.setdefault("OPENCV_VIDEOIO_PRIORITY_GSTREAMER", "0")

import statistics
import sys
import threading
import time
from collections import deque

import cv2
import numpy as np
import pigpio
from flask import Flask, Response

try:
    import RPi.GPIO as GPIO

    GPIO_AVAILABLE = True
except ImportError:
    GPIO_AVAILABLE = False


# Ultrasonic pins (BCM)
FRONT_TRIG, FRONT_ECHO = 18, 19
LEFT_TRIG, LEFT_ECHO = 24, 25
RIGHT_TRIG, RIGHT_ECHO = 4, 26

# Motor pins (BCM)
Motor_ENA, Motor_IN1, Motor_IN2, Motor_IN3, Motor_IN4, Motor_ENB = 13, 17, 27, 22, 23, 12
USE_PWM_EN = False

# Thresholds (tune these)
FRONT_STOP_CM = 20.0
SIDE_OBS_CM = 18.0

# Sampling requirements
STOP_CONFIRM_SECONDS = 1.0
TURN_SAMPLE_SECONDS = 3.0
US_ONLY_STOP_SECONDS = 2.0  # stop if front US blocked this long even if camera is clear

# Rotation duration
TURN_SECONDS = 2.0

# Ultrasonic timing/noise
MIN_INTERVAL_S = 0.14
ECHO_RISE_TIMEOUT_S = 0.03
ECHO_FALL_TIMEOUT_S = 0.025
ECHO_GLITCH_US = 350
STUCK_HIGH_RECOVER_S = 0.01

LOCK_DISTANCE_CM = 1000.0
LOCK_TOLERANCE_CM = 120.0

# Camera settings
CAM_INDEX = 0
CAM_WIDTH = 640
CAM_HEIGHT = 480
CAM_FPS = 30
CAM_CENTER_LAPLACIAN_VAR_THRESHOLD = 25.0

# Color detection (ported from camera_test.py)
COLORS = {
    "Red": [
        (np.array([0, 150, 100]), np.array([4, 255, 255])),
        (np.array([176, 150, 100]), np.array([180, 255, 255])),
    ],
    "Green": [(np.array([55, 150, 60]), np.array([75, 255, 255]))],
    "Black": [(np.array([0, 0, 0]), np.array([180, 255, 30]))],
}
COLOR_DRAW = {
    "Red": (0, 0, 255),
    "Green": (0, 255, 0),
    "Black": (255, 255, 255),
}
MIN_CONTOUR_AREA = 700

# Web server
WEB_HOST = "0.0.0.0"
WEB_PORT = 5000


app = Flask(__name__)


HTML = """\
<!doctype html>
<html>
  <head>
    <meta charset="utf-8" />
    <meta name="viewport" content="width=device-width, initial-scale=1" />
    <title>Robot Camera</title>
    <style>
      body { margin: 0; background: #0b0d10; color: #d8dee9; font-family: monospace; }
      .wrap { padding: 12px; }
      .row { display: flex; gap: 12px; align-items: center; flex-wrap: wrap; }
      .pill { padding: 6px 10px; border: 1px solid #2e3440; border-radius: 999px; background: #11151b; }
      img { width: 100%; max-width: 960px; border: 1px solid #2e3440; border-radius: 10px; display: block; }
    </style>
  </head>
  <body>
    <div class="wrap">
      <div class="row">
        <div class="pill">Stream: <code>/video_feed</code></div>
        <div class="pill">Run: <code>python3 us_cam_motor.py --debug</code></div>
      </div>
      <div style="height: 10px"></div>
      <img src="/video_feed" />
    </div>
  </body>
</html>
"""


class SharedFrames:
    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._frame = None
        self._jpeg = None

    def update(self, frame, jpeg: bytes) -> None:
        with self._lock:
            self._frame = frame
            self._jpeg = jpeg

    def get_frame(self):
        with self._lock:
            return None if self._frame is None else self._frame.copy()

    def get_jpeg(self) -> bytes | None:
        with self._lock:
            return self._jpeg


shared = SharedFrames()


pwm_list = []


def setup_motors() -> None:
    if not GPIO_AVAILABLE:
        return
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)

    for pin in (Motor_ENA, Motor_IN1, Motor_IN2, Motor_IN3, Motor_IN4, Motor_ENB):
        GPIO.setup(pin, GPIO.OUT)

    global pwm_list
    pwm_list = []

    if USE_PWM_EN:
        for p in (Motor_ENA, Motor_ENB):
            p_obj = GPIO.PWM(p, 1000)
            p_obj.start(0)
            pwm_list.append(p_obj)
    else:
        GPIO.output(Motor_ENA, GPIO.HIGH)
        GPIO.output(Motor_ENB, GPIO.HIGH)


def _apply_speed(speed: int) -> None:
    if not GPIO_AVAILABLE:
        return
    if USE_PWM_EN:
        for p in pwm_list:
            p.ChangeDutyCycle(speed)
    else:
        GPIO.output(Motor_ENA, GPIO.HIGH)
        GPIO.output(Motor_ENB, GPIO.HIGH)


def stop_motors() -> None:
    if not GPIO_AVAILABLE:
        return
    if USE_PWM_EN:
        for p in pwm_list:
            p.ChangeDutyCycle(0)
    for pin in (Motor_IN1, Motor_IN2, Motor_IN3, Motor_IN4):
        GPIO.output(pin, GPIO.LOW)


def motor_forward(speed: int = 100) -> None:
    if not GPIO_AVAILABLE:
        return
    _apply_speed(speed)
    GPIO.output(Motor_IN1, GPIO.LOW)
    GPIO.output(Motor_IN2, GPIO.HIGH)
    GPIO.output(Motor_IN3, GPIO.LOW)
    GPIO.output(Motor_IN4, GPIO.HIGH)


def motor_right(speed: int = 100) -> None:
    if not GPIO_AVAILABLE:
        return
    _apply_speed(speed)
    GPIO.output(Motor_IN1, GPIO.LOW)
    GPIO.output(Motor_IN2, GPIO.HIGH)
    GPIO.output(Motor_IN3, GPIO.HIGH)
    GPIO.output(Motor_IN4, GPIO.LOW)


def motor_left(speed: int = 100) -> None:
    if not GPIO_AVAILABLE:
        return
    _apply_speed(speed)
    GPIO.output(Motor_IN1, GPIO.HIGH)
    GPIO.output(Motor_IN2, GPIO.LOW)
    GPIO.output(Motor_IN3, GPIO.LOW)
    GPIO.output(Motor_IN4, GPIO.HIGH)


def center_block_info(frame) -> tuple[bool, tuple[int, int, int, int], float]:
    h, w = frame.shape[:2]
    s_h, e_h, s_w, e_w = h // 4, 3 * h // 4, w // 4, 3 * w // 4
    roi = frame[s_h:e_h, s_w:e_w]
    gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
    score = cv2.Laplacian(gray, cv2.CV_64F).var()
    blocked = score < CAM_CENTER_LAPLACIAN_VAR_THRESHOLD
    return blocked, (s_w, s_h, e_w, e_h), float(score)


def overlay_vision(frame) -> None:
    blocked, roi, score = center_block_info(frame)
    (x1, y1, x2, y2) = roi

    if blocked:
        cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 0, 255), 3)
        cv2.putText(
            frame,
            f"BLOCKED (lapVar {score:.1f})",
            (x1, max(0, y1 - 10)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 0, 255),
            2,
        )
    else:
        cv2.rectangle(frame, (x1, y1), (x2, y2), (80, 0, 0), 1)
        cv2.putText(
            frame,
            f"CLEAR (lapVar {score:.1f})",
            (x1, max(0, y1 - 10)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (80, 0, 0),
            2,
        )

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    for name, ranges in COLORS.items():
        mask = None
        for lo, hi in ranges:
            m = cv2.inRange(hsv, lo, hi)
            mask = m if mask is None else cv2.bitwise_or(mask, m)

        mask = cv2.erode(mask, None, iterations=1)
        mask = cv2.dilate(mask, None, iterations=2)

        cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not cnts:
            continue

        c = max(cnts, key=cv2.contourArea)
        area = cv2.contourArea(c)
        if area < MIN_CONTOUR_AREA:
            continue

        x, y, w, h = cv2.boundingRect(c)
        color = COLOR_DRAW.get(name, (255, 255, 255))
        cv2.rectangle(frame, (x, y), (x + w, y + h), color, 2)
        cv2.putText(
            frame,
            name,
            (x, max(0, y - 8)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            color,
            2,
        )


class Ultrasonic:
    def __init__(self, pi: pigpio.pi, trig: int, echo: int, name: str):
        self.pi = pi
        self.trig = trig
        self.echo = echo
        self.name = name
        self._last_trigger_time = 0.0

    def setup(self) -> None:
        self.pi.set_mode(self.trig, pigpio.OUTPUT)
        self.pi.set_mode(self.echo, pigpio.INPUT)
        self.pi.set_pull_up_down(self.echo, pigpio.PUD_DOWN)
        self.pi.set_glitch_filter(self.echo, ECHO_GLITCH_US)
        self.pi.write(self.trig, 0)

    def read_cm(self, debug_errors: bool = False) -> float | None:
        now = time.perf_counter()
        if now - self._last_trigger_time < MIN_INTERVAL_S:
            time.sleep(MIN_INTERVAL_S - (now - self._last_trigger_time))

        if self.pi.read(self.echo) == 1:
            t_h = time.perf_counter()
            while self.pi.read(self.echo) == 1 and (time.perf_counter() - t_h) < STUCK_HIGH_RECOVER_S:
                pass
            if self.pi.read(self.echo) == 1:
                if debug_errors:
                    print(f"[debug] {self.name}: ECHO stuck high pre-trigger")
                return None

        self.pi.gpio_trigger(self.trig, 10, 1)
        self._last_trigger_time = time.perf_counter()
        t0 = self._last_trigger_time

        while self.pi.read(self.echo) == 0:
            if time.perf_counter() - t0 > ECHO_RISE_TIMEOUT_S:
                if debug_errors:
                    print(f"[debug] {self.name}: timeout waiting ECHO rise")
                return None

        start = time.perf_counter()
        while self.pi.read(self.echo) == 1:
            if time.perf_counter() - start > ECHO_FALL_TIMEOUT_S:
                if debug_errors:
                    print(f"[debug] {self.name}: timeout waiting ECHO fall (long pulse)")
                return None

        duration = time.perf_counter() - start
        distance = (duration * 34300) / 2

        if abs(distance - LOCK_DISTANCE_CM) <= LOCK_TOLERANCE_CM:
            return None
        if distance < 2 or distance > 400:
            return None
        return distance


class USSampler:
    def __init__(self, front: Ultrasonic, left: Ultrasonic, right: Ultrasonic):
        self.front = front
        self.left = left
        self.right = right
        self._lock = threading.Lock()
        self._buf = {
            "front": deque(),
            "left": deque(),
            "right": deque(),
        }

    def _add(self, name: str, v: float | None) -> None:
        if v is None:
            return
        t = time.perf_counter()
        with self._lock:
            self._buf[name].append((t, v))

    def _prune(self, now_t: float, window_s: float) -> None:
        cutoff = now_t - window_s
        for k in self._buf:
            dq = self._buf[k]
            while dq and dq[0][0] < cutoff:
                dq.popleft()

    def latest(self, name: str) -> float | None:
        with self._lock:
            dq = self._buf[name]
            return None if not dq else dq[-1][1]

    def median(self, name: str, window_s: float) -> float | None:
        now_t = time.perf_counter()
        with self._lock:
            self._prune(now_t, window_s)
            vals = [v for _, v in self._buf[name]]
        if not vals:
            return None
        return float(statistics.median(vals))

    def loop(self, stop_evt: threading.Event, debug_errors: bool) -> None:
        while not stop_evt.is_set():
            # Read sequentially to avoid simultaneous triggers.
            self._add("front", self.front.read_cm(debug_errors=debug_errors))
            self._add("left", self.left.read_cm(debug_errors=debug_errors))
            self._add("right", self.right.read_cm(debug_errors=debug_errors))


def decide_turn(front_cm: float | None, left_cm: float | None, right_cm: float | None) -> str:
    front_blocked = front_cm is not None and front_cm < FRONT_STOP_CM
    left_blocked = left_cm is not None and left_cm < SIDE_OBS_CM
    right_blocked = right_cm is not None and right_cm < SIDE_OBS_CM

    if front_blocked and left_blocked and not right_blocked:
        return "right"
    if front_blocked and right_blocked and not left_blocked:
        return "left"

    if left_blocked and not right_blocked:
        return "right"
    if right_blocked and not left_blocked:
        return "left"

    if left_cm is None and right_cm is None:
        return "right"
    if left_cm is None:
        return "left"
    if right_cm is None:
        return "right"
    return "right" if right_cm >= left_cm else "left"


def open_camera():
    cap = cv2.VideoCapture(CAM_INDEX, cv2.CAP_V4L2)
    if not cap.isOpened():
        cap = cv2.VideoCapture(CAM_INDEX)

    cap.set(cv2.CAP_PROP_FRAME_WIDTH, CAM_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAM_HEIGHT)
    cap.set(cv2.CAP_PROP_FPS, CAM_FPS)

    try:
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    except Exception:
        pass

    try:
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    except Exception:
        pass

    return cap


def camera_loop(stop_evt: threading.Event) -> None:
    cap = open_camera()
    if not cap.isOpened():
        print("camera not available")
        return

    try:
        while not stop_evt.is_set():
            ok, frame = cap.read()
            if not ok or frame is None:
                time.sleep(0.02)
                continue

            overlay = frame.copy()
            overlay_vision(overlay)

            ok_jpg, buf = cv2.imencode(".jpg", overlay)
            if ok_jpg:
                shared.update(frame, buf.tobytes())

            time.sleep(0.001)
    finally:
        cap.release()


def control_loop(stop_evt: threading.Event, debug: bool, debug_errors: bool) -> None:
    pi = pigpio.pi()
    if not pi.connected:
        print("pigpio daemon not running")
        return

    front = Ultrasonic(pi, FRONT_TRIG, FRONT_ECHO, "front")
    left = Ultrasonic(pi, LEFT_TRIG, LEFT_ECHO, "left")
    right = Ultrasonic(pi, RIGHT_TRIG, RIGHT_ECHO, "right")

    front.setup()
    left.setup()
    right.setup()

    sampler = USSampler(front, left, right)
    t_us = threading.Thread(target=sampler.loop, args=(stop_evt, debug_errors), daemon=True)
    t_us.start()

    setup_motors()

    def fmt(v: float | None) -> str:
        return "None" if v is None else f"{v:.1f}"

    last_dbg = 0.0

    stop_pending_since: float | None = None
    deciding_since: float | None = None
    turning_until: float | None = None
    turning_dir: str | None = None
    front_us_blocked_since: float | None = None

    try:
        motor_forward(100)

        while not stop_evt.is_set():
            frame = shared.get_frame()
            if frame is None:
                time.sleep(0.02)
                continue

            now_t = time.perf_counter()

            if debug and (now_t - last_dbg) >= 0.5:
                print(
                    f"front:{fmt(sampler.latest('front'))} "
                    f"right:{fmt(sampler.latest('right'))} "
                    f"left:{fmt(sampler.latest('left'))}"
                )
                last_dbg = now_t

            # If currently turning, just wait it out while sampler continues.
            if turning_until is not None:
                if now_t >= turning_until:
                    turning_until = None
                    turning_dir = None
                    motor_forward(100)
                time.sleep(0.02)
                continue

            cam_blocked, _, _ = center_block_info(frame)
            front_now = sampler.latest("front")

            # While deciding, keep sampling and debug printing.
            if deciding_since is not None:
                if now_t - deciding_since >= TURN_SAMPLE_SECONDS:
                    f_med = sampler.median("front", TURN_SAMPLE_SECONDS)
                    l_med = sampler.median("left", TURN_SAMPLE_SECONDS)
                    r_med = sampler.median("right", TURN_SAMPLE_SECONDS)
                    direction = decide_turn(f_med, l_med, r_med)

                    if debug:
                        print(
                            f"front:{fmt(f_med)} right:{fmt(r_med)} left:{fmt(l_med)} -> {direction}"
                        )

                    if direction == "right":
                        motor_right(100)
                    else:
                        motor_left(100)

                    turning_dir = direction
                    turning_until = now_t + TURN_SECONDS
                    deciding_since = None
                time.sleep(0.02)
                continue

            # Stop confirmation logic:
            # - cam+front: require cam_blocked AND front US blocked for 1s, confirm by median.
            # - US-only lock: if front US stays blocked for >=2s even when camera is clear, stop anyway (confirmed by median).
            front_blocked = front_now is not None and front_now < FRONT_STOP_CM

            if front_blocked:
                if front_us_blocked_since is None:
                    front_us_blocked_since = now_t
            else:
                front_us_blocked_since = None

            cam_stop_candidate = cam_blocked and front_blocked
            us_only_stop_candidate = (
                front_us_blocked_since is not None
                and (now_t - front_us_blocked_since) >= US_ONLY_STOP_SECONDS
            )

            if cam_stop_candidate:
                if stop_pending_since is None:
                    stop_pending_since = now_t
            else:
                stop_pending_since = None

            should_stop = False
            if stop_pending_since is not None and (now_t - stop_pending_since) >= STOP_CONFIRM_SECONDS:
                f_med = sampler.median("front", STOP_CONFIRM_SECONDS)
                if f_med is not None and f_med < FRONT_STOP_CM:
                    should_stop = True

            if us_only_stop_candidate:
                f_med = sampler.median("front", STOP_CONFIRM_SECONDS)
                if f_med is not None and f_med < FRONT_STOP_CM:
                    should_stop = True

            if should_stop:
                stop_motors()
                deciding_since = now_t
                stop_pending_since = None
                front_us_blocked_since = None

            time.sleep(0.02)

    except Exception as e:
        print(f"control loop error: {e}")
    finally:
        stop_motors()
        if GPIO_AVAILABLE:
            GPIO.cleanup()
        pi.stop()


@app.get("/")
def index():
    return HTML


@app.get("/video_feed")
def video_feed():
    def gen():
        while True:
            jpeg = shared.get_jpeg()
            if jpeg is None:
                time.sleep(0.05)
                continue
            yield (
                b"--frame\r\n"
                b"Content-Type: image/jpeg\r\n\r\n" + jpeg + b"\r\n"
            )
            time.sleep(0.03)

    return Response(gen(), mimetype="multipart/x-mixed-replace; boundary=frame")


def main() -> None:
    debug = "--debug" in sys.argv
    debug_errors = "--debug-errors" in sys.argv

    stop_evt = threading.Event()

    t_cam = threading.Thread(target=camera_loop, args=(stop_evt,), daemon=True)
    t_ctl = threading.Thread(target=control_loop, args=(stop_evt, debug, debug_errors), daemon=True)

    t_cam.start()
    t_ctl.start()

    try:
        app.run(host=WEB_HOST, port=WEB_PORT, threaded=True, use_reloader=False)
    finally:
        stop_evt.set()
        t_cam.join(timeout=2)
        t_ctl.join(timeout=2)


if __name__ == "__main__":
    main()
