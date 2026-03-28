from __future__ import annotations

import signal
import sys
import threading
import time
from statistics import median

import cv2
import pigpio
from flask import Flask, Response, jsonify, request


app = Flask(__name__)

IN1, IN2, IN3, IN4, ENA, ENB = 17, 27, 22, 23, 13, 12
FRONT_TRIG, FRONT_ECHO = 18, 19
LEFT_TRIG, LEFT_ECHO = 24, 25
RIGHT_TRIG, RIGHT_ECHO = 4, 26
SERVO_PIN = 16
SERVO_HOME_US = 500
SERVO_ALERT_US = 1300

PWM_HZ = 1000
DEFAULT_DRIVE_PWM = 65.0
DEFAULT_TURN_PWM = 80.0
DEFAULT_TURN_SECONDS = 0.6
CURVE_RATIO = 0.45
MIN_PWM = 20.0
MAX_PWM = 100.0
SERVO_MIN_US = 500
SERVO_MAX_US = 1300
STREAM_PORT = 5002
CAMERA_INDEX = 0
MIN_INTERVAL_S = 0.03
ECHO_RISE_TIMEOUT_S = 0.03
ECHO_FALL_TIMEOUT_S = 0.025
ECHO_GLITCH_US = 350
STUCK_HIGH_RECOVER_S = 0.01
LOCK_DISTANCE_CM = 1000.0
LOCK_TOLERANCE_CM = 120.0
MIN_DISTANCE_CM = 2.0
MAX_DISTANCE_CM = 400.0
SENSOR_SAMPLE_SECONDS = 0.12

pi: pigpio.pi | None = None
camera_thread: threading.Thread | None = None
sensor_thread: threading.Thread | None = None
shutdown_event = threading.Event()
state_lock = threading.Lock()

latest_jpeg: bytes | None = None
status_text = "idle"
current_command = "stop"
drive_pwm = DEFAULT_DRIVE_PWM
turn_pwm = DEFAULT_TURN_PWM
turn_seconds = DEFAULT_TURN_SECONDS
servo_pwm = SERVO_HOME_US
motion_token = 0
sensor_snapshot = {"front": None, "left": None, "right": None}
SENSORS = {
    "front": {"trig": FRONT_TRIG, "echo": FRONT_ECHO, "last_trigger": 0.0},
    "left": {"trig": LEFT_TRIG, "echo": LEFT_ECHO, "last_trigger": 0.0},
    "right": {"trig": RIGHT_TRIG, "echo": RIGHT_ECHO, "last_trigger": 0.0},
}


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def set_status(text: str) -> None:
    global status_text
    with state_lock:
        status_text = text


def next_motion_token() -> int:
    global motion_token
    with state_lock:
        motion_token += 1
        return motion_token


def format_distance(value: float | None) -> str:
    return "--" if value is None else f"{value:.1f} cm"


def set_sensor_snapshot(front: float | None, left: float | None, right: float | None) -> None:
    with state_lock:
        sensor_snapshot["front"] = front
        sensor_snapshot["left"] = left
        sensor_snapshot["right"] = right


def setup_motors(hw: pigpio.pi) -> None:
    for pin in (IN1, IN2, IN3, IN4, ENA, ENB):
        hw.set_mode(pin, pigpio.OUTPUT)
    hw.set_PWM_frequency(ENA, PWM_HZ)
    hw.set_PWM_frequency(ENB, PWM_HZ)
    hw.set_PWM_range(ENA, 100)
    hw.set_PWM_range(ENB, 100)
    hw.set_PWM_dutycycle(ENA, 0)
    hw.set_PWM_dutycycle(ENB, 0)


def setup_ultrasonic(hw: pigpio.pi) -> None:
    for sensor in SENSORS.values():
        hw.set_mode(sensor["trig"], pigpio.OUTPUT)
        hw.set_mode(sensor["echo"], pigpio.INPUT)
        hw.set_pull_up_down(sensor["echo"], pigpio.PUD_DOWN)
        hw.set_glitch_filter(sensor["echo"], ECHO_GLITCH_US)
        hw.write(sensor["trig"], 0)
    time.sleep(0.08)


def cleanup_ultrasonic(hw: pigpio.pi) -> None:
    for sensor in SENSORS.values():
        hw.write(sensor["trig"], 0)
        hw.set_glitch_filter(sensor["echo"], 0)


def setup_servo(hw: pigpio.pi) -> None:
    hw.set_mode(SERVO_PIN, pigpio.OUTPUT)
    set_servo_pwm(SERVO_HOME_US)


def set_servo_pwm(pulsewidth: float) -> None:
    global servo_pwm
    if pi is None:
        return
    pulsewidth = int(clamp(pulsewidth, SERVO_MIN_US, SERVO_MAX_US))
    pi.set_servo_pulsewidth(SERVO_PIN, pulsewidth)
    with state_lock:
        servo_pwm = pulsewidth
    set_status(f"servo @ {pulsewidth} us")


def release_servo() -> None:
    if pi is not None:
        pi.set_servo_pulsewidth(SERVO_PIN, 0)


def get_distance_cm(hw: pigpio.pi, sensor_name: str) -> float | None:
    sensor = SENSORS[sensor_name]
    now = time.perf_counter()
    since_last = now - sensor["last_trigger"]
    if since_last < MIN_INTERVAL_S:
        time.sleep(MIN_INTERVAL_S - since_last)
    echo = sensor["echo"]
    trig = sensor["trig"]
    if hw.read(echo) == 1:
        t_h = time.perf_counter()
        while hw.read(echo) == 1 and (time.perf_counter() - t_h) < STUCK_HIGH_RECOVER_S:
            pass
        if hw.read(echo) == 1:
            return None
    hw.gpio_trigger(trig, 10, 1)
    sensor["last_trigger"] = time.perf_counter()
    t0 = sensor["last_trigger"]
    while hw.read(echo) == 0:
        if time.perf_counter() - t0 > ECHO_RISE_TIMEOUT_S:
            return None
    start = time.perf_counter()
    while hw.read(echo) == 1:
        if time.perf_counter() - start > ECHO_FALL_TIMEOUT_S:
            return None
    duration = time.perf_counter() - start
    distance = (duration * 34300.0) / 2.0
    if abs(distance - LOCK_DISTANCE_CM) <= LOCK_TOLERANCE_CM or distance < MIN_DISTANCE_CM or distance > MAX_DISTANCE_CM:
        return None
    return distance


def average_distance(hw: pigpio.pi, sensor_name: str, sample_seconds: float) -> float | None:
    samples: list[float] = []
    started_at = time.perf_counter()
    while (time.perf_counter() - started_at) < sample_seconds and not shutdown_event.is_set():
        value = get_distance_cm(hw, sensor_name)
        if value is not None:
            samples.append(value)
        time.sleep(0.01)
    return None if not samples else median(samples)


def sensor_worker() -> None:
    while not shutdown_event.is_set():
        if pi is None:
            time.sleep(0.1)
            continue
        front = average_distance(pi, "front", SENSOR_SAMPLE_SECONDS)
        left = average_distance(pi, "left", SENSOR_SAMPLE_SECONDS * 0.8)
        right = average_distance(pi, "right", SENSOR_SAMPLE_SECONDS * 0.8)
        set_sensor_snapshot(front, left, right)
        time.sleep(0.05)


def apply_drive(hw: pigpio.pi, left_speed: float, right_speed: float) -> None:
    left_speed = clamp(left_speed, -MAX_PWM, MAX_PWM)
    right_speed = clamp(right_speed, -MAX_PWM, MAX_PWM)

    if left_speed > 0:
        hw.write(IN1, 0)
        hw.write(IN2, 1)
    elif left_speed < 0:
        hw.write(IN1, 1)
        hw.write(IN2, 0)
    else:
        hw.write(IN1, 0)
        hw.write(IN2, 0)

    if right_speed > 0:
        hw.write(IN3, 0)
        hw.write(IN4, 1)
    elif right_speed < 0:
        hw.write(IN3, 1)
        hw.write(IN4, 0)
    else:
        hw.write(IN3, 0)
        hw.write(IN4, 0)

    hw.set_PWM_dutycycle(ENA, int(abs(left_speed)))
    hw.set_PWM_dutycycle(ENB, int(abs(right_speed)))


def stop_motors() -> None:
    global current_command
    if pi is None:
        return
    apply_drive(pi, 0.0, 0.0)
    with state_lock:
        current_command = "stop"
    set_status("motors stopped")


def set_command(command: str) -> None:
    global current_command
    with state_lock:
        current_command = command


def run_drive(left_pwm: float, right_pwm: float, label: str) -> None:
    if pi is None:
        return
    apply_drive(pi, left_pwm, right_pwm)
    set_command(label)
    set_status(f"{label} L={left_pwm:.0f} R={right_pwm:.0f}")


def drive_forward(pwm: float) -> None:
    run_drive(pwm, pwm, "forward")


def drive_backward(pwm: float) -> None:
    run_drive(-pwm, -pwm, "backward")


def rotate_left(pwm: float) -> None:
    run_drive(-pwm, pwm, "left")


def rotate_right(pwm: float) -> None:
    run_drive(pwm, -pwm, "right")


def curve_forward_left(pwm: float) -> None:
    run_drive(pwm * CURVE_RATIO, pwm, "forward_left")


def curve_forward_right(pwm: float) -> None:
    run_drive(pwm, pwm * CURVE_RATIO, "forward_right")


def curve_backward_left(pwm: float) -> None:
    run_drive(-(pwm * CURVE_RATIO), -pwm, "backward_left")


def curve_backward_right(pwm: float) -> None:
    run_drive(-pwm, -(pwm * CURVE_RATIO), "backward_right")


def perform_command(command: str, pwm: float | None = None) -> tuple[bool, str]:
    global drive_pwm
    if pi is None:
        return False, "pigpio not initialized"

    if pwm is None:
        pwm = drive_pwm
    pwm = clamp(float(pwm), MIN_PWM, MAX_PWM)
    drive_pwm = pwm

    if command == "forward":
        drive_forward(pwm)
    elif command == "backward":
        drive_backward(pwm)
    elif command == "left":
        rotate_left(max(turn_pwm, pwm))
    elif command == "right":
        rotate_right(max(turn_pwm, pwm))
    elif command == "forward_left":
        curve_forward_left(pwm)
    elif command == "forward_right":
        curve_forward_right(pwm)
    elif command == "backward_left":
        curve_backward_left(pwm)
    elif command == "backward_right":
        curve_backward_right(pwm)
    elif command == "stop":
        stop_motors()
    else:
        return False, f"unknown command: {command}"
    return True, status_text


def schedule_timed_motion(command: str, pwm: float, duration_s: float) -> tuple[bool, str]:
    if pi is None:
        return False, "pigpio not initialized"

    token = next_motion_token()
    ok, message = perform_command(command, pwm)
    if not ok:
        return ok, message

    def worker(local_token: int, duration: float) -> None:
        started_at = time.perf_counter()
        while not shutdown_event.is_set() and (time.perf_counter() - started_at) < duration:
            with state_lock:
                if local_token != motion_token:
                    return
            time.sleep(0.02)
        with state_lock:
            if local_token != motion_token:
                return
        stop_motors()
        set_status(f"{command} timed turn complete")

    threading.Thread(target=worker, args=(token, duration_s), daemon=True).start()
    return True, f"{command} for {duration_s:.2f}s @ {pwm:.0f}"


def schedule_forward_calibration(left_pwm: float, right_pwm: float, duration_s: float) -> tuple[bool, str]:
    if pi is None:
        return False, "pigpio not initialized"

    left_pwm = clamp(float(left_pwm), MIN_PWM, MAX_PWM)
    right_pwm = clamp(float(right_pwm), MIN_PWM, MAX_PWM)
    duration_s = clamp(float(duration_s), 0.1, 10.0)
    token = next_motion_token()
    run_drive(left_pwm, right_pwm, "forward_calibrate")

    def worker(local_token: int, duration: float) -> None:
        started_at = time.perf_counter()
        while not shutdown_event.is_set() and (time.perf_counter() - started_at) < duration:
            with state_lock:
                if local_token != motion_token:
                    return
            time.sleep(0.02)
        with state_lock:
            if local_token != motion_token:
                return
        stop_motors()
        set_status(f"forward calibration complete L={left_pwm:.0f} R={right_pwm:.0f}")

    threading.Thread(target=worker, args=(token, duration_s), daemon=True).start()
    return True, f"forward calibration {duration_s:.2f}s L={left_pwm:.0f} R={right_pwm:.0f}"


def camera_worker() -> None:
    global latest_jpeg
    cap = cv2.VideoCapture(CAMERA_INDEX)
    if not cap.isOpened():
        set_status("camera open failed")
        return
    try:
        while not shutdown_event.is_set():
            ok, frame = cap.read()
            if not ok or frame is None:
                time.sleep(0.03)
                continue
            with state_lock:
                status = status_text
                command = current_command
                current_drive = drive_pwm
                current_turn = turn_pwm
                current_servo = servo_pwm
                front = sensor_snapshot["front"]
                left = sensor_snapshot["left"]
                right = sensor_snapshot["right"]
            cv2.putText(frame, status, (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            cv2.putText(frame, f"cmd={command}", (10, 54), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 2)
            cv2.putText(frame, f"drive={current_drive:.0f} turn={current_turn:.0f}", (10, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 2)
            cv2.putText(frame, f"servo={current_servo}", (10, 106), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 200, 0), 2)
            cv2.putText(frame, f"ultra F:{format_distance(front)} L:{format_distance(left)} R:{format_distance(right)}", (10, 132), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 0), 2)
            ok, buffer = cv2.imencode(".jpg", frame)
            if ok:
                with state_lock:
                    latest_jpeg = buffer.tobytes()
            time.sleep(0.02)
    finally:
        cap.release()


def stream_frames():
    boundary = b"--frame\r\nContent-Type: image/jpeg\r\n\r\n"
    while not shutdown_event.is_set():
        with state_lock:
            frame = latest_jpeg
        if frame is None:
            time.sleep(0.05)
            continue
        yield boundary + frame + b"\r\n"
        time.sleep(0.03)


@app.route("/")
def index() -> str:
    return f"""
<!doctype html>
<html>
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>Remote Control</title>
  <style>
    :root {{
      color-scheme: dark;
      --bg: #10181c;
      --panel: #18242a;
      --panel2: #223138;
      --accent: #f4b400;
      --danger: #d94b4b;
      --text: #edf4f6;
      --muted: #9cb0b7;
      --good: #2e7d57;
      --turn: #345f7a;
      --diag: #5f5a2e;
    }}
    body {{
      margin: 0;
      font-family: Consolas, "Courier New", monospace;
      background: radial-gradient(circle at top, #1f3239, var(--bg) 60%);
      color: var(--text);
      overflow: hidden;
      overscroll-behavior: none;
    }}
    .wrap {{
      max-width: 1200px;
      margin: 0 auto;
      padding: 24px 16px 40px;
      height: 100vh;
      box-sizing: border-box;
      overflow: auto;
    }}
    .layout {{
      display: grid;
      grid-template-columns: minmax(320px, 1fr) minmax(320px, 1fr);
      gap: 18px;
    }}
    .panel {{
      background: linear-gradient(180deg, var(--panel), var(--panel2));
      border: 1px solid #2f444c;
      border-radius: 16px;
      padding: 18px;
      box-shadow: 0 14px 30px rgba(0, 0, 0, 0.28);
    }}
    h1, h2 {{
      margin: 0 0 10px;
    }}
    .muted {{
      color: var(--muted);
      margin-bottom: 16px;
      line-height: 1.45;
    }}
    .status {{
      margin-bottom: 18px;
      padding: 12px 14px;
      border-radius: 12px;
      background: rgba(255,255,255,0.04);
      border: 1px solid rgba(255,255,255,0.08);
      line-height: 1.6;
    }}
    .video {{
      width: 100%;
      border-radius: 12px;
      border: 1px solid #35505a;
      background: #0a0f12;
      aspect-ratio: 4 / 3;
      object-fit: cover;
    }}
    .drive-grid {{
      display: grid;
      grid-template-columns: repeat(3, minmax(90px, 1fr));
      gap: 12px;
      margin: 18px 0;
    }}
    button {{
      min-height: 68px;
      border: 0;
      border-radius: 14px;
      font-size: 16px;
      font-weight: 700;
      color: var(--text);
      background: #2c4048;
      cursor: pointer;
      transition: transform 0.05s ease, filter 0.1s ease;
    }}
    button:hover {{ filter: brightness(1.08); }}
    button:active, .active {{
      transform: scale(0.98);
      filter: brightness(1.14);
    }}
    .forward, .backward {{ background: var(--good); }}
    .turn {{ background: var(--turn); }}
    .diag {{ background: var(--diag); }}
    .stop {{ background: var(--danger); }}
    .servo {{ background: #6a4b1b; }}
    .row {{
      display: flex;
      gap: 12px;
      flex-wrap: wrap;
      align-items: center;
      margin-top: 12px;
    }}
    .field {{
      flex: 1 1 180px;
      min-width: 160px;
    }}
    label {{
      display: block;
      margin-bottom: 6px;
      color: var(--muted);
      font-size: 14px;
    }}
    input[type=range], input[type=number] {{
      width: 100%;
      box-sizing: border-box;
    }}
    .small-buttons {{
      display: grid;
      grid-template-columns: repeat(3, minmax(90px, 1fr));
      gap: 12px;
      margin-top: 14px;
    }}
    .quad-grid {{
      display: grid;
      grid-template-columns: repeat(2, minmax(120px, 1fr));
      gap: 12px;
      margin-top: 14px;
    }}
    .full {{
      width: 100%;
    }}
    @media (max-width: 900px) {{
      .layout {{
        grid-template-columns: 1fr;
      }}
    }}
  </style>
</head>
<body>
  <div class="wrap">
    <div class="layout">
      <div class="panel">
        <h1>Remote Control</h1>
        <div class="muted">Manual motor test page with camera feed, servo controls, mixed turning, and timed turn debugging.</div>
        <div class="status">
          <div><strong>Status:</strong> <span id="status">idle</span></div>
          <div><strong>Command:</strong> <span id="command">stop</span></div>
          <div><strong>Drive PWM:</strong> <span id="drive_pwm_label">{int(DEFAULT_DRIVE_PWM)}</span></div>
          <div><strong>Turn PWM:</strong> <span id="turn_pwm_label">{int(DEFAULT_TURN_PWM)}</span></div>
          <div><strong>Turn Time:</strong> <span id="turn_seconds_label">{DEFAULT_TURN_SECONDS:.2f}</span>s</div>
          <div><strong>Servo PWM:</strong> <span id="servo_pwm_label">{SERVO_HOME_US}</span></div>
          <div><strong>Ultra Front:</strong> <span id="front_distance">--</span></div>
          <div><strong>Ultra Left:</strong> <span id="left_distance">--</span></div>
          <div><strong>Ultra Right:</strong> <span id="right_distance">--</span></div>
        </div>
        <img class="video" src="/video_feed" alt="camera feed">
      </div>

      <div class="panel">
        <h2>Drive</h2>
        <div class="muted">Hold a button or use keyboard: W/A/S/D, arrows, and diagonal combinations like W+A or S+D. Releasing the keys sends stop.</div>
        <div class="drive-grid">
          <button class="diag" data-command="forward_left">Fwd Left</button>
          <button class="forward" data-command="forward">Forward</button>
          <button class="diag" data-command="forward_right">Fwd Right</button>
          <button class="turn" data-command="left">Rotate Left</button>
          <button class="stop" data-command="stop">Stop</button>
          <button class="turn" data-command="right">Rotate Right</button>
          <button class="diag" data-command="backward_left">Back Left</button>
          <button class="backward" data-command="backward">Backward</button>
          <button class="diag" data-command="backward_right">Back Right</button>
        </div>

        <div class="row">
          <div class="field">
            <label for="drive_pwm">Drive PWM</label>
            <input id="drive_pwm" type="range" min="{int(MIN_PWM)}" max="{int(MAX_PWM)}" value="{int(DEFAULT_DRIVE_PWM)}">
          </div>
          <div class="field">
            <label for="drive_pwm_value">Drive PWM Value</label>
            <input id="drive_pwm_value" type="number" min="{int(MIN_PWM)}" max="{int(MAX_PWM)}" step="1" value="{int(DEFAULT_DRIVE_PWM)}">
          </div>
          <div class="field">
            <label for="turn_pwm">Turn PWM</label>
            <input id="turn_pwm" type="range" min="{int(MIN_PWM)}" max="{int(MAX_PWM)}" value="{int(DEFAULT_TURN_PWM)}">
          </div>
          <div class="field">
            <label for="turn_pwm_value">Turn PWM Value</label>
            <input id="turn_pwm_value" type="number" min="{int(MIN_PWM)}" max="{int(MAX_PWM)}" step="1" value="{int(DEFAULT_TURN_PWM)}">
          </div>
        </div>

        <div class="row">
          <div class="field">
            <label for="turn_seconds">Timed Turn Seconds</label>
            <input id="turn_seconds" type="number" min="0.1" max="5.0" step="0.1" value="{DEFAULT_TURN_SECONDS:.1f}">
          </div>
          <div class="field">
            <label>&nbsp;</label>
            <button class="turn full" id="timed_left">Timed Left</button>
          </div>
          <div class="field">
            <label>&nbsp;</label>
            <button class="turn full" id="timed_right">Timed Right</button>
          </div>
        </div>

        <h2 style="margin-top:22px;">Servo</h2>
        <div class="small-buttons">
          <button class="servo" id="servo_home">Servo Home</button>
          <button class="servo" id="servo_alert">Servo Alert</button>
          <button class="servo" id="servo_release">Servo Release</button>
        </div>
        <div class="row">
          <div class="field">
            <label for="servo_pwm">Servo PWM</label>
            <input id="servo_pwm" type="range" min="{SERVO_MIN_US}" max="{SERVO_MAX_US}" value="{SERVO_HOME_US}">
          </div>
          <div class="field">
            <label for="servo_pwm_value">Servo PWM Value</label>
            <input id="servo_pwm_value" type="number" min="{SERVO_MIN_US}" max="{SERVO_MAX_US}" step="1" value="{SERVO_HOME_US}">
          </div>
          <div class="field">
            <label>&nbsp;</label>
            <button class="servo full" id="servo_set_pwm">Set Servo PWM</button>
          </div>
        </div>

        <h2 style="margin-top:22px;">Forward Calibration</h2>
        <div class="muted">Use ultrasonic live readings while running a custom forward test with separate left and right PWM values.</div>
        <div class="quad-grid">
          <div class="field">
            <label for="cal_left_pwm">Left PWM</label>
            <input id="cal_left_pwm" type="number" min="{int(MIN_PWM)}" max="{int(MAX_PWM)}" step="1" value="{int(DEFAULT_DRIVE_PWM)}">
          </div>
          <div class="field">
            <label for="cal_right_pwm">Right PWM</label>
            <input id="cal_right_pwm" type="number" min="{int(MIN_PWM)}" max="{int(MAX_PWM)}" step="1" value="{int(DEFAULT_DRIVE_PWM)}">
          </div>
          <div class="field">
            <label for="cal_seconds">Duration Seconds</label>
            <input id="cal_seconds" type="number" min="0.1" max="10.0" step="0.1" value="1.5">
          </div>
          <div class="field">
            <label>&nbsp;</label>
            <button class="forward full" id="calibrate_forward">Run Forward Calibration</button>
          </div>
        </div>

        <div class="row">
          <button class="stop full" id="shutdown">Shutdown</button>
        </div>
      </div>
    </div>
  </div>
  <script>
    const statusNode = document.getElementById("status");
    const commandNode = document.getElementById("command");
    const drivePwmNode = document.getElementById("drive_pwm");
    const drivePwmValueNode = document.getElementById("drive_pwm_value");
    const turnPwmNode = document.getElementById("turn_pwm");
    const turnPwmValueNode = document.getElementById("turn_pwm_value");
    const turnSecondsNode = document.getElementById("turn_seconds");
    const servoPwmNode = document.getElementById("servo_pwm");
    const servoPwmValueNode = document.getElementById("servo_pwm_value");
    const calLeftPwmNode = document.getElementById("cal_left_pwm");
    const calRightPwmNode = document.getElementById("cal_right_pwm");
    const calSecondsNode = document.getElementById("cal_seconds");
    const drivePwmLabel = document.getElementById("drive_pwm_label");
    const turnPwmLabel = document.getElementById("turn_pwm_label");
    const turnSecondsLabel = document.getElementById("turn_seconds_label");
    const servoPwmLabel = document.getElementById("servo_pwm_label");
    const frontDistanceNode = document.getElementById("front_distance");
    const leftDistanceNode = document.getElementById("left_distance");
    const rightDistanceNode = document.getElementById("right_distance");
    const holdButtons = Array.from(document.querySelectorAll("button[data-command]"));
    const heldKeys = new Set();

    function currentDrivePwm() {{
      return Number(drivePwmNode.value);
    }}

    function currentTurnPwm() {{
      return Number(turnPwmNode.value);
    }}

    function currentTurnSeconds() {{
      return Number(turnSecondsNode.value);
    }}

    function currentServoPwm() {{
      return Number(servoPwmNode.value);
    }}

    function updateLabels() {{
      drivePwmLabel.textContent = drivePwmNode.value;
      turnPwmLabel.textContent = turnPwmNode.value;
      turnSecondsLabel.textContent = Number(turnSecondsNode.value).toFixed(2);
      servoPwmLabel.textContent = servoPwmNode.value;
      drivePwmValueNode.value = drivePwmNode.value;
      turnPwmValueNode.value = turnPwmNode.value;
      servoPwmValueNode.value = servoPwmNode.value;
    }}

    async function refreshStatus() {{
      const response = await fetch("/status");
      const data = await response.json();
      statusNode.textContent = data.status;
      commandNode.textContent = data.command;
      drivePwmLabel.textContent = data.drive_pwm;
      turnPwmLabel.textContent = data.turn_pwm;
      turnSecondsLabel.textContent = Number(data.turn_seconds).toFixed(2);
      servoPwmLabel.textContent = data.servo_pwm;
      drivePwmNode.value = data.drive_pwm;
      drivePwmValueNode.value = data.drive_pwm;
      turnPwmNode.value = data.turn_pwm;
      turnPwmValueNode.value = data.turn_pwm;
      servoPwmNode.value = data.servo_pwm;
      servoPwmValueNode.value = data.servo_pwm;
      frontDistanceNode.textContent = data.front_distance;
      leftDistanceNode.textContent = data.left_distance;
      rightDistanceNode.textContent = data.right_distance;
    }}

    async function sendMove(command) {{
      const response = await fetch("/move", {{
        method: "POST",
        headers: {{ "Content-Type": "application/json" }},
        body: JSON.stringify({{
          command,
          pwm: currentDrivePwm(),
          turn_pwm: currentTurnPwm()
        }})
      }});
      const data = await response.json();
      statusNode.textContent = data.status;
      commandNode.textContent = data.command;
      holdButtons.forEach(btn => btn.classList.toggle("active", btn.dataset.command === command && command !== "stop"));
    }}

    async function sendTimedTurn(command) {{
      const response = await fetch("/timed_turn", {{
        method: "POST",
        headers: {{ "Content-Type": "application/json" }},
        body: JSON.stringify({{
          command,
          pwm: currentTurnPwm(),
          duration: currentTurnSeconds()
        }})
      }});
      const data = await response.json();
      statusNode.textContent = data.status;
      commandNode.textContent = data.command;
    }}

    async function sendServo(action) {{
      const response = await fetch("/servo", {{
        method: "POST",
        headers: {{ "Content-Type": "application/json" }},
        body: JSON.stringify({{
          action,
          pwm: currentServoPwm()
        }})
      }});
      const data = await response.json();
      statusNode.textContent = data.status;
      servoPwmLabel.textContent = data.servo_pwm;
    }}

    async function sendCalibration() {{
      const response = await fetch("/calibrate_forward", {{
        method: "POST",
        headers: {{ "Content-Type": "application/json" }},
        body: JSON.stringify({{
          left_pwm: Number(calLeftPwmNode.value),
          right_pwm: Number(calRightPwmNode.value),
          duration: Number(calSecondsNode.value)
        }})
      }});
      const data = await response.json();
      statusNode.textContent = data.status;
      commandNode.textContent = data.command;
    }}

    function commandFromKeys() {{
      const forward = heldKeys.has("arrowup") || heldKeys.has("w");
      const backward = heldKeys.has("arrowdown") || heldKeys.has("s");
      const left = heldKeys.has("arrowleft") || heldKeys.has("a");
      const right = heldKeys.has("arrowright") || heldKeys.has("d");

      if (forward && left) return "forward_left";
      if (forward && right) return "forward_right";
      if (backward && left) return "backward_left";
      if (backward && right) return "backward_right";
      if (forward) return "forward";
      if (backward) return "backward";
      if (left) return "left";
      if (right) return "right";
      return "stop";
    }}

    function syncKeyCommand() {{
      sendMove(commandFromKeys());
    }}

    holdButtons.forEach((button) => {{
      button.addEventListener("mousedown", () => sendMove(button.dataset.command));
      button.addEventListener("mouseup", () => sendMove("stop"));
      button.addEventListener("mouseleave", () => sendMove("stop"));
      button.addEventListener("touchstart", (event) => {{
        event.preventDefault();
        sendMove(button.dataset.command);
      }}, {{ passive: false }});
      button.addEventListener("touchend", () => sendMove("stop"));
    }});

    drivePwmNode.addEventListener("input", updateLabels);
    drivePwmValueNode.addEventListener("input", () => {{
      drivePwmNode.value = drivePwmValueNode.value;
      updateLabels();
    }});
    turnPwmNode.addEventListener("input", updateLabels);
    turnPwmValueNode.addEventListener("input", () => {{
      turnPwmNode.value = turnPwmValueNode.value;
      updateLabels();
    }});
    turnSecondsNode.addEventListener("input", updateLabels);
    servoPwmNode.addEventListener("input", updateLabels);
    servoPwmValueNode.addEventListener("input", () => {{
      servoPwmNode.value = servoPwmValueNode.value;
      updateLabels();
    }});

    document.addEventListener("keydown", (event) => {{
      const key = event.key.toLowerCase();
      if (["arrowup", "arrowdown", "arrowleft", "arrowright", "w", "a", "s", "d", " ", "x"].includes(key)) {{
        event.preventDefault();
      }}
      if (key === " " || key === "x") {{
        heldKeys.clear();
        sendMove("stop");
        return;
      }}
      if (["arrowup", "arrowdown", "arrowleft", "arrowright", "w", "a", "s", "d"].includes(key)) {{
        heldKeys.add(key);
        syncKeyCommand();
      }}
    }});

    document.addEventListener("keyup", (event) => {{
      const key = event.key.toLowerCase();
      if (["arrowup", "arrowdown", "arrowleft", "arrowright", "w", "a", "s", "d", " ", "x"].includes(key)) {{
        event.preventDefault();
      }}
      if (heldKeys.has(key)) {{
        heldKeys.delete(key);
        syncKeyCommand();
      }}
    }});

    document.getElementById("timed_left").addEventListener("click", () => sendTimedTurn("left"));
    document.getElementById("timed_right").addEventListener("click", () => sendTimedTurn("right"));
    document.getElementById("servo_home").addEventListener("click", () => sendServo("home"));
    document.getElementById("servo_alert").addEventListener("click", () => sendServo("alert"));
    document.getElementById("servo_release").addEventListener("click", () => sendServo("release"));
    document.getElementById("servo_set_pwm").addEventListener("click", () => sendServo("set_pwm"));
    document.getElementById("calibrate_forward").addEventListener("click", sendCalibration);
    document.getElementById("shutdown").addEventListener("click", async () => {{
      await fetch("/shutdown", {{ method: "POST" }});
      statusNode.textContent = "shutdown requested";
      commandNode.textContent = "stop";
    }});

    updateLabels();
    refreshStatus();
    setInterval(refreshStatus, 1000);
  </script>
</body>
</html>
"""


@app.route("/video_feed")
def video_feed() -> Response:
    return Response(stream_frames(), mimetype="multipart/x-mixed-replace; boundary=frame")


@app.route("/move", methods=["POST"])
def move() -> Response:
    global turn_pwm
    payload = request.get_json(silent=True) or {}
    command = str(payload.get("command", "stop")).strip().lower()
    pwm = payload.get("pwm", drive_pwm)
    requested_turn_pwm = payload.get("turn_pwm", turn_pwm)
    turn_pwm = clamp(float(requested_turn_pwm), MIN_PWM, MAX_PWM)
    next_motion_token()
    ok, message = perform_command(command, pwm)
    status = 200 if ok else 400
    return jsonify(
        {
            "ok": ok,
            "status": message,
            "command": current_command,
        }
    ), status


@app.route("/timed_turn", methods=["POST"])
def timed_turn() -> Response:
    global turn_pwm, turn_seconds
    payload = request.get_json(silent=True) or {}
    command = str(payload.get("command", "")).strip().lower()
    turn_pwm = clamp(float(payload.get("pwm", turn_pwm)), MIN_PWM, MAX_PWM)
    turn_seconds = clamp(float(payload.get("duration", turn_seconds)), 0.1, 5.0)
    if command not in ("left", "right"):
        return jsonify({"ok": False, "status": "timed turn only supports left/right", "command": current_command}), 400
    ok, message = schedule_timed_motion(command, turn_pwm, turn_seconds)
    status = 200 if ok else 400
    return jsonify(
        {
            "ok": ok,
            "status": message,
            "command": current_command,
        }
    ), status


@app.route("/calibrate_forward", methods=["POST"])
def calibrate_forward() -> Response:
    payload = request.get_json(silent=True) or {}
    left_pwm = payload.get("left_pwm", drive_pwm)
    right_pwm = payload.get("right_pwm", drive_pwm)
    duration = payload.get("duration", 1.5)
    ok, message = schedule_forward_calibration(left_pwm, right_pwm, duration)
    status = 200 if ok else 400
    return jsonify(
        {
            "ok": ok,
            "status": message,
            "command": current_command,
        }
    ), status


@app.route("/servo", methods=["POST"])
def servo() -> Response:
    payload = request.get_json(silent=True) or {}
    action = str(payload.get("action", "")).strip().lower()
    pwm = clamp(float(payload.get("pwm", servo_pwm)), SERVO_MIN_US, SERVO_MAX_US)

    if action == "home":
        set_servo_pwm(SERVO_HOME_US)
    elif action == "alert":
        set_servo_pwm(SERVO_ALERT_US)
    elif action == "set_pwm":
        set_servo_pwm(pwm)
    elif action == "release":
        release_servo()
        set_status("servo released")
    else:
        return jsonify({"ok": False, "status": f"unknown servo action: {action}", "servo_pwm": servo_pwm}), 400

    with state_lock:
        current_servo = servo_pwm
    return jsonify({"ok": True, "status": status_text, "servo_pwm": current_servo})


@app.route("/status")
def status() -> Response:
    with state_lock:
        payload = {
            "status": status_text,
            "command": current_command,
            "drive_pwm": int(drive_pwm),
            "turn_pwm": int(turn_pwm),
            "turn_seconds": round(turn_seconds, 2),
            "servo_pwm": int(servo_pwm),
            "front_distance": format_distance(sensor_snapshot["front"]),
            "left_distance": format_distance(sensor_snapshot["left"]),
            "right_distance": format_distance(sensor_snapshot["right"]),
        }
    return jsonify(payload)


@app.route("/shutdown", methods=["POST"])
def shutdown() -> str:
    shutdown_event.set()
    next_motion_token()
    stop_motors()

    def delayed_shutdown() -> None:
        time.sleep(0.4)
        signal.raise_signal(signal.SIGINT)

    threading.Thread(target=delayed_shutdown, daemon=True).start()
    return "Shutting down remote control."


def main() -> None:
    global pi, camera_thread, sensor_thread
    pi = pigpio.pi()
    if not pi.connected:
        print("pigpio daemon not running or not reachable.", file=sys.stderr)
        sys.exit(1)

    setup_motors(pi)
    setup_ultrasonic(pi)
    setup_servo(pi)
    stop_motors()
    camera_thread = threading.Thread(target=camera_worker, daemon=True)
    sensor_thread = threading.Thread(target=sensor_worker, daemon=True)
    camera_thread.start()
    sensor_thread.start()
    print(f"remote_control.py ready on port {STREAM_PORT}")
    try:
        app.run(host="0.0.0.0", port=STREAM_PORT, threaded=True)
    except KeyboardInterrupt:
        print("\nStopped by user")
    finally:
        shutdown_event.set()
        next_motion_token()
        stop_motors()
        set_servo_pwm(SERVO_HOME_US)
        release_servo()
        if pi is not None:
            cleanup_ultrasonic(pi)
            pi.stop()


if __name__ == "__main__":
    main()
