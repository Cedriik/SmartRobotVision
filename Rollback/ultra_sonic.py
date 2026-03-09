import RPi.GPIO as GPIO
import time

# Setup
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

# Ultrasonic sensor pins (non-conflicting with new_motors.py)
TRIG_FRONT = 23
ECHO_FRONT = 24
TRIG_LEFT = 5
ECHO_LEFT = 6
TRIG_RIGHT = 21
ECHO_RIGHT = 20

GPIO.setup(TRIG_FRONT, GPIO.OUT)
GPIO.setup(ECHO_FRONT, GPIO.IN)
GPIO.setup(TRIG_LEFT, GPIO.OUT)
GPIO.setup(ECHO_LEFT, GPIO.IN)
GPIO.setup(TRIG_RIGHT, GPIO.OUT)
GPIO.setup(ECHO_RIGHT, GPIO.IN)

print("Triple distance measurement in progress")
GPIO.output(TRIG_FRONT, False)
GPIO.output(TRIG_LEFT, False)
GPIO.output(TRIG_RIGHT, False)
print("Waiting for sensors to settle")
time.sleep(2)


def measure_distance(trig_pin, echo_pin):
    GPIO.output(trig_pin, True)
    time.sleep(0.00001)
    GPIO.output(trig_pin, False)

    timeout = time.time() + 0.04
    pulse_start = None
    while GPIO.input(echo_pin) == 0 and time.time() < timeout:
        pulse_start = time.time()

    timeout = time.time() + 0.04
    pulse_end = None
    while GPIO.input(echo_pin) == 1 and time.time() < timeout:
        pulse_end = time.time()

    if pulse_start is None or pulse_end is None:
        return None

    pulse_duration = pulse_end - pulse_start
    distance = pulse_duration * 17150
    return round(distance, 2)


try:
    while True:
        readings_front = []
        readings_left = []
        readings_right = []

        for _ in range(3):
            d_front = measure_distance(TRIG_FRONT, ECHO_FRONT)
            time.sleep(0.06)  # small gap to reduce sensor crosstalk
            d_left = measure_distance(TRIG_LEFT, ECHO_LEFT)
            time.sleep(0.06)
            d_right = measure_distance(TRIG_RIGHT, ECHO_RIGHT)

            if d_front is not None:
                readings_front.append(d_front)
            if d_left is not None:
                readings_left.append(d_left)
            if d_right is not None:
                readings_right.append(d_right)

            time.sleep(0.05)

        if readings_front:
            avg_front = round(sum(readings_front) / len(readings_front), 2)
        else:
            avg_front = None

        if readings_left:
            avg_left = round(sum(readings_left) / len(readings_left), 2)
        else:
            avg_left = None
        if readings_right:
            avg_right = round(sum(readings_right) / len(readings_right), 2)
        else:
            avg_right = None

        print("Front:", f"{avg_front} cm")
        print("Left:", f"{avg_left} cm")
        print("Right:", f"{avg_right} cm")
        print("-" * 30)

        time.sleep(0.5)

except KeyboardInterrupt:
    print("\nMeasurement stopped by user")
finally:
    GPIO.cleanup()
