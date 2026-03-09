import RPi.GPIO as GPIO
import time

# Setup
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

TRIG = 23
ECHO = 24

GPIO.setup(TRIG, GPIO.OUT)
GPIO.setup(ECHO, GPIO.IN)

print("Distance measurement in progress")
GPIO.output(TRIG, False)
print("Waiting for sensor to settle")
time.sleep(2)

# Continuous loop
try:
    while True:
        # Send trigger pulse
        GPIO.output(TRIG, True)
        time.sleep(0.00001)
        GPIO.output(TRIG, False)

        # Wait for echo start
        while GPIO.input(ECHO) == 0:
            pulse_start = time.time()

        # Wait for echo end
        while GPIO.input(ECHO) == 1:
            pulse_end = time.time()

        # Calculate distance
        pulse_duration = pulse_end - pulse_start
        distance = pulse_duration * 17150
        distance = round(distance, 2)

        print("Distance:", distance, "cm")

        time.sleep(1)  # Delay between readings

except KeyboardInterrupt:
    print("\nMeasurement stopped by user")
    GPIO.cleanup()
