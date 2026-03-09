import cv2
import numpy as np
import os
import signal
import time
import threading
from flask import Flask, Response, render_template_string

import pigpio

# Try to import RPi.GPIO, handling the case where it might be missing (dev mode)
try:
    import RPi.GPIO as GPIO
    GPIO_AVAILABLE = True
except ImportError:
    print("!! RPi.GPIO not found. Running in EMULATION mode (No motors).")
    GPIO_AVAILABLE = False

app = Flask(__name__)

# --- HARDWARE CONFIGURATION (From integrated_bot.py) ---
# Moved ENA off UART TX (BCM14) to BCM13; ENB already on BCM12 to avoid serial conflicts
# If you don't need PWM speed control, set USE_PWM_EN=False to mimic Remote/new_motors.py (always-on enables).
USE_PWM_EN = False
Motor_ENA, Motor_IN1, Motor_IN2, Motor_IN3, Motor_IN4, Motor_ENB = 13, 17, 27, 22, 23, 12
SERVO_PIN = 16

# Ultrasonic (pigpio)
TRIG, ECHO = 18, 19
MIN_DISTANCE_CM = 2
STOP_THRESHOLD_CM = 20
MAX_DISTANCE_CM = 400
ECHO_RISE_TIMEOUT_S = 0.02
ECHO_FALL_TIMEOUT_S = 0.065
MIN_INTERVAL_S = 0.08
SAMPLES_PER_REPORT = 10
INVALID_SKIP_THRESHOLD = 5

# State Variables
pwm_list = []
is_moving = False
servo_active = False
last_clear_time = 0
CONFIRM_DELAY = 1.0  # Time (sec) path must be clear before motors restart
pi_ultra = None
_last_trigger_time = 0.0
clear_ultra_batches = 0

# --- VISION CONFIGURATION (From test.py) ---
cap = cv2.VideoCapture(0)
# Aim for lower-res, faster capture
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
cap.set(cv2.CAP_PROP_FPS, 30)
cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

# Color Definitions
COLORS = {
    'Red': [(np.array([0, 150, 100]), np.array([4, 255, 255])),
            (np.array([176, 150, 100]), np.array([180, 255, 255]))],
    # widen green hue window by ~10% of original span
    'Green': [(np.array([53, 150, 60]), np.array([77, 255, 255]))],
    'Black': [(np.array([0, 0, 0]), np.array([180, 255, 30]))]
}

detection_memory = {
    'Red':   {'box': None, 'frames': 0, 'consecutive': 0, 'saved': False},
    'Green': {'box': None, 'frames': 0, 'consecutive': 0, 'saved': False},
    'Black': {'box': None, 'frames': 0, 'consecutive': 0, 'saved': False}
}

# --- HARDWARE FUNCTIONS ---
def setup_hardware():
    if not GPIO_AVAILABLE: return
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    pins = [Motor_ENA, Motor_IN1, Motor_IN2, Motor_IN3, Motor_IN4, Motor_ENB, SERVO_PIN]
    for pin in pins:
        GPIO.setup(pin, GPIO.OUT)
    
    global pwm_list
    pwm_list = []

    if USE_PWM_EN:
        # Initialize Motor PWMs
        for p in [Motor_ENA, Motor_ENB]:
            p_obj = GPIO.PWM(p, 1000)
            p_obj.start(0)
            pwm_list.append(p_obj)
    else:
        # Keep enables simply HIGH (matches Remote/new_motors.py behavior)
        GPIO.output(Motor_ENA, GPIO.HIGH)
        GPIO.output(Motor_ENB, GPIO.HIGH)


def setup_ultrasonic():
    global pi_ultra
    pi_ultra = pigpio.pi()
    if not pi_ultra.connected:
        raise SystemExit("pigpio daemon not running or not reachable.")
    pi_ultra.set_mode(TRIG, pigpio.OUTPUT)
    pi_ultra.set_mode(ECHO, pigpio.INPUT)
    pi_ultra.set_pull_up_down(ECHO, pigpio.PUD_DOWN)
    pi_ultra.write(TRIG, 0)

def stop_motors():
    global is_moving
    is_moving = False
    if not GPIO_AVAILABLE: return
    for p in pwm_list: 
        p.ChangeDutyCycle(0)
    # Set all IN pins low (avoid list writes which can silently fail on some RPi.GPIO builds)
    for pin in (Motor_IN1, Motor_IN2, Motor_IN3, Motor_IN4):
        GPIO.output(pin, GPIO.LOW)


# --- Ultrasonic helpers ---
def single_distance_cm():
    global _last_trigger_time
    now = time.perf_counter()
    if now - _last_trigger_time < MIN_INTERVAL_S:
        time.sleep(MIN_INTERVAL_S - (now - _last_trigger_time))

    if pi_ultra.read(ECHO) == 1:
        return None

    pi_ultra.gpio_trigger(TRIG, 10, 1)
    _last_trigger_time = time.perf_counter()
    t0 = _last_trigger_time

    while pi_ultra.read(ECHO) == 0:
        if time.perf_counter() - t0 > ECHO_RISE_TIMEOUT_S:
            return None

    start = time.perf_counter()

    while pi_ultra.read(ECHO) == 1:
        if time.perf_counter() - start > ECHO_FALL_TIMEOUT_S:
            return None

    duration = time.perf_counter() - start
    dist = (duration * 34300) / 2
    if dist < MIN_DISTANCE_CM or dist > MAX_DISTANCE_CM:
        return None
    return dist


def sampled_distance():
    samples = []
    invalid = 0
    for _ in range(SAMPLES_PER_REPORT):
        d = single_distance_cm()
        if d is not None:
            samples.append(d)
        else:
            invalid += 1
        time.sleep(MIN_INTERVAL_S)
    if invalid >= INVALID_SKIP_THRESHOLD or not samples:
        return None
    samples.sort()
    return samples[len(samples) // 2]

# --- DISCRETE MOTOR CONTROLS (parity with Remote/new_motors.py) ---
def _apply_speed(speed):
    """Helper to set duty cycle for both enables or hold them high."""
    if not GPIO_AVAILABLE: return
    if USE_PWM_EN:
        for p in pwm_list:
            p.ChangeDutyCycle(speed)
    else:
        # With static enables, just ensure they stay HIGH
        GPIO.output(Motor_ENA, GPIO.HIGH)
        GPIO.output(Motor_ENB, GPIO.HIGH)

def motor_forward(speed=100):
    """Both wheels forward."""
    global is_moving
    is_moving = True
    if not GPIO_AVAILABLE: return
    _apply_speed(speed)
    GPIO.output(Motor_IN1, GPIO.LOW)
    GPIO.output(Motor_IN2, GPIO.HIGH)
    GPIO.output(Motor_IN3, GPIO.LOW)
    GPIO.output(Motor_IN4, GPIO.HIGH)

def motor_backward(speed=100):
    """Both wheels backward."""
    global is_moving
    is_moving = True
    if not GPIO_AVAILABLE: return
    _apply_speed(speed)
    GPIO.output(Motor_IN1, GPIO.HIGH)
    GPIO.output(Motor_IN2, GPIO.LOW)
    GPIO.output(Motor_IN3, GPIO.HIGH)
    GPIO.output(Motor_IN4, GPIO.LOW)

def motor_right(speed=100):
    """Pivot right: left wheel forward, right wheel backward."""
    global is_moving
    is_moving = True
    if not GPIO_AVAILABLE: return
    _apply_speed(speed)
    GPIO.output(Motor_IN1, GPIO.LOW)
    GPIO.output(Motor_IN2, GPIO.HIGH)
    GPIO.output(Motor_IN3, GPIO.HIGH)
    GPIO.output(Motor_IN4, GPIO.LOW)

def motor_left(speed=100):
    """Pivot left: left wheel backward, right wheel forward."""
    global is_moving
    is_moving = True
    if not GPIO_AVAILABLE: return
    _apply_speed(speed)
    GPIO.output(Motor_IN1, GPIO.HIGH)
    GPIO.output(Motor_IN2, GPIO.LOW)
    GPIO.output(Motor_IN3, GPIO.LOW)
    GPIO.output(Motor_IN4, GPIO.HIGH)

# Backwards compatibility: keep original naming used in the vision loop
def move_forward(speed=100):
    global is_moving
    is_moving = True
    if not GPIO_AVAILABLE: return
    motor_forward(speed)

def servo_scan_routine():
    """Runs the servo dance in a separate thread so video doesn't freeze."""
    global servo_active
    if not GPIO_AVAILABLE: return
    
    servo_active = True
    s_pwm = GPIO.PWM(SERVO_PIN, 50)
    s_pwm.start(0)
    
    # UP
    s_pwm.ChangeDutyCycle(12.5) 
    time.sleep(1.0) # Reduced from 5s to 1s for better responsiveness
    
    # DOWN
    s_pwm.ChangeDutyCycle(2.5)
    time.sleep(1.0)
    
    # CENTER/OFF
    s_pwm.stop()
    servo_active = False

def trigger_servo_thread():
    """Helper to start servo thread safely."""
    if not servo_active:
        t = threading.Thread(target=servo_scan_routine)
        t.daemon = True
        t.start()

# --- VISION LOGIC ---
def is_center_blocked(frame):
    """Detects obstructions in the center 50% of the frame."""
    try:
        h, w = frame.shape[:2]
        s_h, e_h, s_w, e_w = h // 4, 3 * h // 4, w // 4, 3 * w // 4
        roi = frame[s_h:e_h, s_w:e_w]
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        score = cv2.Laplacian(gray, cv2.CV_64F).var()
        return score < 25, (s_w, s_h, e_w, e_h)
    except:
        return False, (0,0,0,0)

HTML = """
<html>
    <head><title>Robot Vision Integrated</title>
    <style>
        body { background: #000; color: #0f0; font-family: monospace; text-align: center; margin: 0; }
        .container { display: flex; flex-direction: column; align-items: center; justify-content: center; height: 100vh; }
        img { border: 2px solid #222; max-width: 90%; box-shadow: 0 0 20px #040; }
        .hud { margin-bottom: 5px; font-size: 11px; letter-spacing: 1px; }
        .status-moving { color: #0f0; font-weight: bold; }
        .status-stopped { color: #f00; font-weight: bold; }
    </style></head>
    <body>
        <div class="container">
            <div class="hud">SYSTEM: INTEGRATED // MOTORS: <span id="m_stat">STANDBY</span></div>
            <img src="/video_feed">
            <br><a href="/stop_server" style="color:#600; text-decoration:none; margin-top:10px; display:block;">[ EMERGENCY SHUTDOWN ]</a>
        </div>
    </body>
</html>
"""

def generate_frames():
    global detection_memory, last_clear_time, clear_ultra_batches
    
    # Initial Start
    move_forward(100)
    
    frame_idx = 0
    while True:
        success, frame = cap.read()
        if not success or frame is None:
            continue
        
        try:
            # 1. OBSTACLE DETECTION (Logic from test.py)
            blocked, roi = is_center_blocked(frame)
            dist = sampled_distance()
            if dist is not None and (frame_idx % 10 == 0):
                print(f"[ultra] median {dist:.1f} cm")
            blocked_ultra = dist is not None and dist <= STOP_THRESHOLD_CM
            
            # --- MOTOR & SERVO LOGIC INTEGRATION ---
            if blocked and blocked_ultra:
                # Both camera and ultrasonic blocked -> Stop Motors, Start Servo
                if is_moving:
                    print(f"OBSTRUCTION DETECTED (cam+ultra {dist:.1f} cm) - STOPPING")
                    stop_motors()
                clear_ultra_batches = 0
                
                # Trigger Servo (Non-blocking thread)
                trigger_servo_thread()
                
                # Visuals
                cv2.rectangle(frame, (roi[0], roi[1]), (roi[2], roi[3]), (0, 0, 255), 3)
                cv2.putText(frame, "!!! BLOCKED !!!", (roi[0], roi[1]-10), 1, 1, (0, 0, 255), 2)
                
                # Reset clear timer
                last_clear_time = time.time()
                
            else:
                # Condition: Vision clear -> Wait for delay -> Move Motors
                cv2.rectangle(frame, (roi[0], roi[1]), (roi[2], roi[3]), (80, 0, 0), 1)
                if dist is not None and dist > STOP_THRESHOLD_CM:
                    clear_ultra_batches = min(clear_ultra_batches + 1, 3)
                else:
                    clear_ultra_batches = 0

                if not blocked and not is_moving:
                    # Check both camera clear and ultrasonic clear for 3 batches
                    if clear_ultra_batches >= 3 and (time.time() - last_clear_time) > CONFIRM_DELAY:
                        print("PATH CLEAR (cam+ultra) - RESUMING")
                        move_forward(100)

            cv2.putText(frame, f"Ultra:{dist:.1f}cm" if dist is not None else "Ultra:N/A",
                        (10, 15), 1, 1, (0, 255, 0), 1)

            # 2. COLOR DETECTION (Logic from test.py)
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            for color_name, ranges in COLORS.items():
                mask = None
                for (low, high) in ranges:
                    m = cv2.inRange(hsv, low, high)
                    mask = m if mask is None else cv2.bitwise_or(mask, m)
                
                mask = cv2.medianBlur(mask, 3) 
                cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                
                found_now = False
                for c in cnts:
                    area = cv2.contourArea(c)
                    # allow ~20% smaller blobs
                    min_area = 80 if color_name == 'Black' else 200
                    
                    if min_area < area < 50000:
                        x, y, w, h = cv2.boundingRect(c)
                        aspect_ratio = float(w)/h
                        
                        if 0.4 < aspect_ratio < 2.5:
                            detection_memory[color_name]['box'] = (x, y, w, h)
                            detection_memory[color_name]['frames'] = 3
                            detection_memory[color_name]['consecutive'] += 1
                            found_now = True
                            break 
                
                if not found_now:
                    detection_memory[color_name]['consecutive'] = 0
                    detection_memory[color_name]['saved'] = False

                mem = detection_memory[color_name]
                if mem['frames'] > 0:
                    bx, by, bw, bh = mem['box']
                    b_col = (0, 255, 0) if color_name == 'Green' else (0, 0, 255)
                    if color_name == 'Black': b_col = (255, 255, 255)
                    
                    is_locked = mem['consecutive'] >= 10
                    cv2.rectangle(frame, (bx, by), (bx+bw, by+bh), b_col, 2 if is_locked else 1)
                    cv2.putText(frame, f"{color_name} {'[LOCKED]' if is_locked else '...'}", (bx, by-5), 1, 0.7, b_col, 1)

            ret, buffer = cv2.imencode('.jpg', frame)
            yield (b'--frame\r\n' b'Content-Type: image/jpeg\r\n\r\n' + buffer.tobytes() + b'\r\n')
        except Exception as e:
            print(f"Error in loop: {e}")
            continue
        finally:
            frame_idx += 1

@app.route('/')
def index(): return render_template_string(HTML)

@app.route('/stop_server')
def stop_server():
    stop_motors()
    cap.release()
    if GPIO_AVAILABLE: GPIO.cleanup()
    os.kill(os.getpid(), signal.SIGINT)
    return "Shutdown complete."

@app.route('/video_feed')
def video_feed(): return Response(generate_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

if __name__ == "__main__":
    setup_hardware()
    setup_ultrasonic()
    try:
        # Host 0.0.0.0 allows access from other devices on the network
        app.run(host='0.0.0.0', port=5000, threaded=True)
    finally:
        stop_motors()
        if GPIO_AVAILABLE: GPIO.cleanup()
        if pi_ultra: pi_ultra.stop()
