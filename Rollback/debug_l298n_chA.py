import RPi.GPIO as GPIO
import time

# L298N Driver 2 - Channel A (BCM)
ENA = 18
IN1 = 17
IN2 = 27

def main():
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)

    GPIO.setup(ENA, GPIO.OUT)
    GPIO.setup(IN1, GPIO.OUT)
    GPIO.setup(IN2, GPIO.OUT)

    pwm = GPIO.PWM(ENA, 1000)
    pwm.start(100)  # 100% speed

    try:
        print("Forward 5s")
        GPIO.output(IN1, GPIO.HIGH)
        GPIO.output(IN2, GPIO.LOW)
        time.sleep(3)

        print("Backward 5s")
        GPIO.output(IN1, GPIO.LOW)
        GPIO.output(IN2, GPIO.HIGH)
        time.sleep(3)
    finally:
        print("Stop")
        GPIO.output(IN1, GPIO.LOW)
        GPIO.output(IN2, GPIO.LOW)
        pwm.stop()
        GPIO.cleanup()

if __name__ == "__main__":
    main()
