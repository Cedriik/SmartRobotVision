import time
import pigpio

RIGHT_TRIG, RIGHT_ECHO = 18, 19
LEFT_TRIG, LEFT_ECHO = 24, 25
LEGACY_RIGHT_TRIG, LEGACY_RIGHT_ECHO = 4, 26

R_ECHO_TIMEOUT_S = 0.03
F_ECHO_TIMEOUT_S = 0.03


def probe_sensor(pi, name, trig, echo, n=5):
    print(f"\n[{name}] TRIG={trig} ECHO={echo}")
    pi.set_mode(trig, pigpio.OUTPUT)
    pi.set_mode(echo, pigpio.INPUT)
    pi.set_pull_up_down(echo, pigpio.PUD_DOWN)
    pi.write(trig, 0)
    time.sleep(0.01)

    idle = [pi.read(echo) for _ in range(10)]
    print(f"idle ECHO samples: {idle}")

    for i in range(n):
        pi.write(trig, 0)
        time.sleep(0.002)

        if pi.read(echo) == 1:
            print(f"  #{i+1}: ECHO already HIGH before trigger")
            continue

        pi.gpio_trigger(trig, 10, 1)
        t0 = time.perf_counter()

        while pi.read(echo) == 0:
            if time.perf_counter() - t0 > R_ECHO_TIMEOUT_S:
                print(f"  #{i+1}: no ECHO rise within {R_ECHO_TIMEOUT_S*1000:.0f} ms")
                break
        else:
            t_rise = time.perf_counter()
            while pi.read(echo) == 1:
                if time.perf_counter() - t_rise > F_ECHO_TIMEOUT_S:
                    print(f"  #{i+1}: ECHO rose but did not fall within {F_ECHO_TIMEOUT_S*1000:.0f} ms")
                    break
            else:
                dur_us = (time.perf_counter() - t_rise) * 1e6
                dist_cm = (dur_us / 1e6) * 34300 / 2
                print(f"  #{i+1}: pulse {dur_us:.0f} us => {dist_cm:.1f} cm")

        time.sleep(0.1)


def main():
    pi = pigpio.pi()
    if not pi.connected:
        print("pigpio daemon not running")
        return

    try:
        print("GPIO ultrasonic diagnostic start")
        probe_sensor(pi, "FRONT(current)", RIGHT_TRIG, RIGHT_ECHO)
        probe_sensor(pi, "LEFT(current)", LEFT_TRIG, LEFT_ECHO)
        probe_sensor(pi, "RIGHT(current)", LEGACY_RIGHT_TRIG, LEGACY_RIGHT_ECHO)
    finally:
        for pin in [RIGHT_TRIG, LEFT_TRIG, LEGACY_RIGHT_TRIG]:
            pi.write(pin, 0)
        pi.stop()


if __name__ == "__main__":
    main()
