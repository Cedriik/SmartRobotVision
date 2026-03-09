import RPi.GPIO as GPIO
import time

# Pin setup
IN1 = 17
IN2 = 27
IN3 = 22
IN4 = 23
ENA = 13
ENB = 12

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

GPIO.setup(IN1, GPIO.OUT)
GPIO.setup(IN2, GPIO.OUT)
GPIO.setup(IN3, GPIO.OUT)
GPIO.setup(IN4, GPIO.OUT)
GPIO.setup(ENA, GPIO.OUT)
GPIO.setup(ENB, GPIO.OUT)

# Enable both channels
GPIO.output(ENA, True)
GPIO.output(ENB, True)

def stop():
    GPIO.output(IN1, False)
    GPIO.output(IN2, False)
    GPIO.output(IN3, False)
    GPIO.output(IN4, False)

def forward():
    GPIO.output(IN1, False)
    GPIO.output(IN2, True)
    GPIO.output(IN3, False)
    GPIO.output(IN4, True)

def backward():
    GPIO.output(IN1, True)
    GPIO.output(IN2, False)
    GPIO.output(IN3, True)
    GPIO.output(IN4, False)

def right():
    GPIO.output(IN1, False)
    GPIO.output(IN2, True)
    GPIO.output(IN3, True)
    GPIO.output(IN4, False)

def left():
    GPIO.output(IN1, True)
    GPIO.output(IN2, False)
    GPIO.output(IN3, False)
    GPIO.output(IN4, True)

try:
    print("Forward")
    forward()
    time.sleep(3)
    stop()

    print("Backward")
    backward()
    time.sleep(3)
    stop()

    print("Right")
    right()
    time.sleep(2)
    stop()

    print("Left")
    left()
    time.sleep(2)
    stop()

except KeyboardInterrupt:
    print("\nStopped by user")

finally:
    stop()
    GPIO.cleanup()
