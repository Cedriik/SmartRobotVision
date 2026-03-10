import statistics
import sys
import time

import cv2
import pigpio

# Try to import GPIO for motor control (allows running without motors)
try:
    import RPi.GPIO as GPIO

    GPIO_AVAILABLE = True
except ImportError:
    GPIO_AVAILABLE = False


# Ultrasonic pins (BCM)
FRONT_TRIG, FRONT_ECHO = 18, 19
LEFT_TRIG, LEFT_ECHO = 24, 25
RIGHT_TRIG, RIGHT_ECHO = 4, 26

# Motor pins (BCM) - matches your existing motor scripts
Motor_ENA, Motor_IN1, Motor_IN2, Motor_IN3, Motor_IN4, Motor_ENB = 13, 17, 27, 22, 23, 12
USE_PWM_EN = False

# Thresholds (tune these)
FRONT_STOP_CM = 20.0
SIDE_OBS_CM = 18.0

# Sampling requirements
STOP_CONFIRM_SECONDS = 1.0   # before stopping motors
TURN_SAMPLE_SECONDS = 3.0    # before choosing rotation direction

# Ultrasonic timing/noise (fast profile-ish)
MIN_INTERVAL_S = 0.14
ECHO_RISE_TIMEOUT_S = 0.03
ECHO_FALL_TIMEOUT_S = 0.025
ECHO_GLITCH_US = 350
STUCK_HIGH_RECOVER_S = 0.01

LOCK_DISTANCE_CM = 1000.0
LOCK_TOLERANCE_CM = 120.0

# Camera settings
CAM_INDEX = 0
CAM_CENTER_LAPLACIAN_VAR_THRESHOLD = 25.0  # lower => more "blocked" (blurry/low texture)


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


def is_center_blocked(frame) -> bool:
    # Same idea as your previous camera logic: look at the middle ROI and use Laplacian variance.
    h, w = frame.shape[:2]
    s_h, e_h, s_w, e_w = h // 4, 3 * h // 4, w // 4, 3 * w // 4
    roi = frame[s_h:e_h, s_w:e_w]
    gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
    score = cv2.Laplacian(gray, cv2.CV_64F).var()
    return score < CAM_CENTER_LAPLACIAN_VAR_THRESHOLD


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

    def read_cm(self, debug: bool = False) -> float | None:
        now = time.perf_counter()
        if now - self._last_trigger_time < MIN_INTERVAL_S:
            time.sleep(MIN_INTERVAL_S - (now - self._last_trigger_time))

        # recover from stuck-high
        if self.pi.read(self.echo) == 1:
            t_h = time.perf_counter()
            while self.pi.read(self.echo) == 1 and (time.perf_counter() - t_h) < STUCK_HIGH_RECOVER_S:
                pass
            if self.pi.read(self.echo) == 1:
                if debug:
                    print(f"[debug] {self.name}: ECHO stuck high pre-trigger")
                return None

        self.pi.gpio_trigger(self.trig, 10, 1)
        self._last_trigger_time = time.perf_counter()
        t0 = self._last_trigger_time

        while self.pi.read(self.echo) == 0:
            if time.perf_counter() - t0 > ECHO_RISE_TIMEOUT_S:
                if debug:
                    print(f"[debug] {self.name}: timeout waiting ECHO rise")
                return None

        start = time.perf_counter()
        while self.pi.read(self.echo) == 1:
            if time.perf_counter() - start > ECHO_FALL_TIMEOUT_S:
                if debug:
                    print(f"[debug] {self.name}: timeout waiting ECHO fall (long pulse)")
                return None

        duration = time.perf_counter() - start
        distance = (duration * 34300) / 2

        if debug:
            print(f"[debug] {self.name}: {duration*1e6:.0f} us -> {distance:.1f} cm")

        if abs(distance - LOCK_DISTANCE_CM) <= LOCK_TOLERANCE_CM:
            return None
        if distance < 2 or distance > 400:
            return None
        return distance


def sample_median(sensor: Ultrasonic, seconds: float, debug: bool = False) -> float | None:
    deadline = time.perf_counter() + seconds
    samples: list[float] = []

    while time.perf_counter() < deadline:
        d = sensor.read_cm(debug=debug)
        if d is not None:
            samples.append(d)

    if not samples:
        return None
    return float(statistics.median(samples))


def decide_turn(front_cm: float | None, left_cm: float | None, right_cm: float | None) -> str:
    # Returns 'left' or 'right' (sample code decision table)
    front_blocked = front_cm is not None and front_cm < FRONT_STOP_CM
    left_blocked = left_cm is not None and left_cm < SIDE_OBS_CM
    right_blocked = right_cm is not None and right_cm < SIDE_OBS_CM

    if front_blocked and left_blocked and not right_blocked:
        return "right"
    if front_blocked and right_blocked and not left_blocked:
        return "left"

    # If only one side is blocked, bias away from it.
    if left_blocked and not right_blocked:
        return "right"
    if right_blocked and not left_blocked:
        return "left"

    # Otherwise choose the side with more space (None treated as unknown -> conservative)
    if left_cm is None and right_cm is None:
        return "right"
    if left_cm is None:
        return "left"
    if right_cm is None:
        return "right"
    return "right" if right_cm >= left_cm else "left"


def main() -> None:
    debug = "--debug" in sys.argv

    pi = pigpio.pi()
    if not pi.connected:
        raise SystemExit("pigpio daemon not running")

    front = Ultrasonic(pi, FRONT_TRIG, FRONT_ECHO, "front")
    left = Ultrasonic(pi, LEFT_TRIG, LEFT_ECHO, "left")
    right = Ultrasonic(pi, RIGHT_TRIG, RIGHT_ECHO, "right")

    front.setup()
    left.setup()
    right.setup()

    setup_motors()

    cap = cv2.VideoCapture(CAM_INDEX)
    if not cap.isOpened():
        raise SystemExit("camera not available")

    try:
        motor_forward(100)

        while True:
            ok, frame = cap.read()
            if not ok or frame is None:
                continue

            cam_blocked = is_center_blocked(frame)
            # quick front check (single sample) so we only do the expensive confirm-sampling when needed
            front_now = front.read_cm(debug=debug)

            if cam_blocked and (front_now is not None and front_now < FRONT_STOP_CM):
                # Confirm stop condition with 1s median sampling
                front_med = sample_median(front, STOP_CONFIRM_SECONDS, debug=debug)
                if front_med is not None and front_med < FRONT_STOP_CM:
                    stop_motors()

                    # Decide turn using 3s sampling windows
                    f_med = sample_median(front, TURN_SAMPLE_SECONDS, debug=debug)
                    l_med = sample_median(left, TURN_SAMPLE_SECONDS, debug=debug)
                    r_med = sample_median(right, TURN_SAMPLE_SECONDS, debug=debug)

                    direction = decide_turn(f_med, l_med, r_med)

                    if debug:
                        print(
                            "[decision] "
                            f"front={f_med} left={l_med} right={r_med} -> {direction}"
                        )

                    if direction == "right":
                        motor_right(100)
                        time.sleep(0.6)
                    else:
                        motor_left(100)
                        time.sleep(0.6)

                    motor_forward(100)

            # Small delay to keep CPU reasonable; ultrasonic has its own min interval
            time.sleep(0.02)

    except KeyboardInterrupt:
        pass
    finally:
        stop_motors()
        if GPIO_AVAILABLE:
            GPIO.cleanup()
        cap.release()
        pi.stop()


if __name__ == "__main__":
    main()
