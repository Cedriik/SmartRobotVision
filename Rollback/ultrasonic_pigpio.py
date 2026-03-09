import pigpio
import time

# GPIO Pin Setup (BCM)
TRIG = 18
ECHO = 19

# Initialize pigpio
pi = pigpio.pi()
if not pi.connected:
    exit()

# Set pin modes
pi.set_mode(TRIG, pigpio.OUTPUT)
pi.set_mode(ECHO, pigpio.INPUT)

# Initialize trigger low
pi.write(TRIG, 0)
time.sleep(0.1)

def get_distance():
    # Send 10us trigger pulse
    pi.write(TRIG, 1)
    time.sleep(0.00001)
    pi.write(TRIG, 0)
    
    # Wait for echo to start
    while pi.read(ECHO) == 0:
        start = time.time()
        
    # Wait for echo to stop
    while pi.read(ECHO) == 1:
        end = time.time()
        
    # Calculate pulse duration
    duration = end - start
    # Speed of sound is ~343m/s or 0.0343 cm/us
    distance = (duration * 34300) / 2
    return distance

try:
    while True:
        dist = get_distance()
        print(f"Distance: {dist:.1f} cm")
        time.sleep(0.5)

except KeyboardInterrupt:
    pi.write(TRIG, 0)
    pi.stop()

