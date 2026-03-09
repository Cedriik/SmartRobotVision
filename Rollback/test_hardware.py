import RPi.GPIO as GPIO
import time

# --- PIN CONFIGURATION ---
# Ultrasonic
US_FRONT = {'trig': 20, 'echo': 21}
US_LEFT = {'trig': 26, 'echo': 8}
US_RIGHT = {'trig': 10, 'echo': 11}

# LDR
LDR_PIN = 4

# Servo
SERVO_PIN = 16

# Motor 1 left
M1_ENA = 18
M1_IN1 = 17
M1_IN2 = 27
M1_IN3 = 22
M1_IN4 = 23
M1_ENB = 13

# Motor 2 right
M2_ENA = 12
M2_IN1 = 5
M2_IN2 = 6
M2_IN3 = 24
M2_IN4 = 25
M2_ENB = 19

def setup():
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)

    # Ultrasonic Setup
    for us in [US_FRONT, US_LEFT, US_RIGHT]:
        GPIO.setup(us['trig'], GPIO.OUT)
        GPIO.setup(us['echo'], GPIO.IN)
        GPIO.output(us['trig'], False)

    # LDR Setup
    GPIO.setup(LDR_PIN, GPIO.IN)

    # Motor Setup
    motor_pins = [M1_ENA, M1_IN1, M1_IN2, M1_IN3, M1_IN4, M1_ENB,
                  M2_ENA, M2_IN1, M2_IN2, M2_IN3, M2_IN4, M2_ENB]
    for pin in motor_pins:
        GPIO.setup(pin, GPIO.OUT)
    
    # Servo Setup
    GPIO.setup(SERVO_PIN, GPIO.OUT)

def get_distance(trig, echo):
    GPIO.output(trig, True)
    time.sleep(0.00001)
    GPIO.output(trig, False)

    start_time = time.time()
    stop_time = time.time()

    timeout = time.time() + 0.1
    while GPIO.input(echo) == 0:
        start_time = time.time()
        if start_time > timeout: return -1

    while GPIO.input(echo) == 1:
        stop_time = time.time()
        if stop_time > timeout: return -1

    elapsed = stop_time - start_time
    distance = (elapsed * 34300) / 2
    return distance

def test_ultrasonic():
    print("\n--- Testing Ultrasonic Sensors (Ctrl+C to stop) ---")
    try:
        while True:
            f = get_distance(US_FRONT['trig'], US_FRONT['echo'])
            l = get_distance(US_LEFT['trig'], US_LEFT['echo'])
            r = get_distance(US_RIGHT['trig'], US_RIGHT['echo'])
            print(f"Front: {f:.1f}cm | Left: {l:.1f}cm | Right: {r:.1f}cm")
            time.sleep(0.5)
    except KeyboardInterrupt:
        print("\nStopped.")

def test_ldr():
    print("\n--- Testing LDR (Cover sensor to test) ---")
    try:
        while True:
            val = GPIO.input(LDR_PIN)
            status = "Dark (LED would be ON)" if val == 0 else "Light (LED OFF)"
            print(f"LDR Pin State: {val} | {status}")
            time.sleep(0.5)
    except KeyboardInterrupt:
        print("\nStopped.")

def run_motor(ena, in1, in2, enb, in3, in4, name):
    print(f"Running {name} Forward for 3s...")
    # Enable PWM
    p1 = GPIO.PWM(ena, 1000)
    p2 = GPIO.PWM(enb, 1000)
    p1.start(100)  # 100% Speed (max)
    p2.start(100)
    
    # Forward
    GPIO.output(in1, GPIO.LOW); GPIO.output(in2, GPIO.HIGH)
    GPIO.output(in3, GPIO.LOW); GPIO.output(in4, GPIO.HIGH)
    time.sleep(5)
    
    print(f"Running {name} Backward for 3s...")
    GPIO.output(in1, GPIO.HIGH); GPIO.output(in2, GPIO.LOW)
    GPIO.output(in3, GPIO.HIGH); GPIO.output(in4, GPIO.LOW)
    time.sleep(5)
    
    print("Stop.")
    p1.stop()
    p2.stop()
    GPIO.output([in1, in2, in3, in4], GPIO.LOW)

def test_motors():
    print("\n1. Test Front Motors (M1)")
    print("2. Test Rear Motors (M2)")
    sel = input("Select: ")
    if sel == '1':
        run_motor(M1_ENA, M1_IN1, M1_IN2, M1_ENB, M1_IN3, M1_IN4, "Front M1")
    elif sel == '2':
        run_motor(M2_ENA, M2_IN1, M2_IN2, M2_ENB, M2_IN3, M2_IN4, "Rear M2")

def test_servo():
    print("\n--- Testing Servo 0 -> 30 -> 45 ---")
    pwm = GPIO.PWM(SERVO_PIN, 50) # 50Hz
    pwm.start(0)
    
    try:
        # 0 Degrees (Approx 2.5% duty)
        print("Position: 0 Degrees")
        pwm.ChangeDutyCycle(2.5)
        time.sleep(1)
        
        # 30 Degrees (Approx 4.16% duty)
        print("Position: 30 Degrees")
        pwm.ChangeDutyCycle(4.16)
        time.sleep(1)
        
        # 45 Degrees (Approx 5.0% duty)
        print("Position: 45 Degrees")
        pwm.ChangeDutyCycle(5.0)
        time.sleep(1)
        
    except KeyboardInterrupt:
        pass
    finally:
        pwm.stop()
        
def main():
    setup()
    while True:
        print("\n--- SAFETY HARDWARE CHECK ---")
        print("1. Test Ultrasonic Sensors")
        print("2. Test LDR")
        print("3. Test Motors")
        print("4. Test Servo")
        print("q. Quit")
        choice = input("Enter choice: ")
        
        if choice == '1': test_ultrasonic()
        elif choice == '2': test_ldr()
        elif choice == '3': test_motors()
        elif choice == '4': test_servo()
        elif choice == 'q': 
            GPIO.cleanup()
            break

if __name__ == "__main__":
    main()

