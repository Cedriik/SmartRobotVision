import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

TRIG = 18
ECHO = 19

GPIO.setup(TRIG, GPIO.OUT)
GPIO.setup(ECHO, GPIO.IN)

print("Ultrasonic sensor test (Right)")
GPIO.output(TRIG, False)
time.sleep(2)


def measure_distance():
    # Reset trigger
    GPIO.output(TRIG, False)
    time.sleep(0.002)

    # Send 10us pulse
    GPIO.output(TRIG, True)
    time.sleep(0.00001)
    GPIO.output(TRIG, False)

    # Wait for echo start
    timeout = time.time() + 0.2
    while GPIO.input(ECHO) == 0:
        pulse_start = time.time()
        if time.time() > timeout:
            return None

    # Wait for echo end
    timeout = time.time() + 0.2
    while GPIO.input(ECHO) == 1:
        pulse_end = time.time()
        if time.time() > timeout:
            return None

    # Calculate distance
    pulse_duration = pulse_end - pulse_start
    distance = pulse_duration * 17150
    return round(distance, 2)


try:
    while True:
        d = measure_distance()
        if d is not None:
            print("Distance:", d, "cm")
        else:
            print("No valid reading")
        time.sleep(0.5)

except KeyboardInterrupt:
    print("\nStopped by user")
finally:
    GPIO.cleanup()
