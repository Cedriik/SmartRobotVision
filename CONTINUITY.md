# CONTINUITY.md

## Change Log
- 2026-03-09: Initialized tracking files (AGENT.md, CONTINUITY.md), created ~/Robot/test, and prepared rollback workflow.

## Current Working Directory
- ~/Robot/test

## Rollback Location
- ~/Robot/Rollback
- 2026-03-09: Created ~/Robot/test/camera_test.py from ~/Robot/Rollback/robot_main.py and ~/Robot/test/motor_test.py from ~/Robot/Rollback/new_motors.py.
- 2026-03-09: Updated ~/Robot/test/camera_test.py with ENABLE_SERVO toggle (default OFF), added rollback backup at ~/Robot/Rollback/camera_test.py.
- 2026-03-09: Initialized git in ~/Robot, added .gitignore, committed current files, set origin to https://github.com/Cedriik/SmartRobotVision.git. Push blocked because GitHub repo is missing and Pi has no GitHub auth configured.
- 2026-03-09: Pushed ~/Robot main branch to GitHub repo Cedriik/SmartRobotVision (force-with-lease due remote main pre-existing commit).
- 2026-03-09: Copied pigpio ultrasonic front script from ~/Robot/Rollback/ultrasonic_front.py to ~/Robot/test/us_front_test.py for debugging.
- 2026-03-09: Tuned ~/Robot/test/us_front_test.py for slower pulses (MIN_INTERVAL_S=0.14), added ECHO glitch filter (ECHO_GLITCH_US=200), and explicit invalid rejection for lock-like ~1000 cm values.
- 2026-03-09: Hardened ~/Robot/test/us_front_test.py against recurring ~58ms bogus pulses by capping ECHO fall timeout to 25ms, adding stuck-high recovery, stronger glitch filter, and slower trigger interval.
- 2026-03-09: Tuned ~/Robot/test/us_front_test.py with speed profiles (--speed=slow|normal|fast), min-valid thresholds, jump rejection, and EMA smoothing for moving robot use.
- 2026-03-09: Pushed latest ultrasonic tuning updates (us_front_test.py, continuity logs) to GitHub Cedriik/SmartRobotVision.
- 2026-03-10: Updated ~/Robot/Rollback/right_ultra.py pins to TRIG=18, ECHO=19. Created ~/Robot/test/us_left_test.py (TRIG=20, ECHO=21) and ~/Robot/test/us_right_test.py (TRIG=18, ECHO=19) by duplicating us_front_test.py.
- 2026-03-10: Added and ran ~/Robot/test/gpio_diag_ultra.py. Results: RIGHT(current 18/19)=no ECHO rise, LEFT(current 20/21)=ECHO stuck high, RIGHT(legacy 4/26)=valid ~29 cm pulses; indicates right wiring still on legacy pins and left ECHO line is held high.
- 2026-03-10: Updated left ultrasonic pins to TRIG=24, ECHO=25 in ~/Robot/Rollback/left_ultra.py and ~/Robot/test/us_left_test.py for rewiring test.
- 2026-03-11: Pushed Pi changes (left/right ultrasonic tests, pin updates, gpio diagnostic) to GitHub Cedriik/SmartRobotVision.
- 2026-03-11: Created ~/Robot/test/us_cam_motor.py integrating camera + front/left/right ultrasonics + motor control with 1s stop confirmation sampling and 3s turn decision sampling. Rollback snapshot: ~/Robot/Rollback/us_cam_motor.py.
- 2026-03-11: Synced `~/Robot/test/us_cam_motor.py` from the Pi into this repo as `test/us_cam_motor.py`; fixed the front-US lock/stop confirmation bug (`sampler.median("front", ...)`), and created a checkpoint snapshot `test/us_cam_motor_cp1.py`.
- 2026-03-11: Fixed a crash path where the control loop could exit and call `pi.stop()` while the ultrasonic sampler thread was still running; now `stop_evt.set()` is called and `t_us.join()` runs (best-effort) before shutting down pigpio. Checkpoint snapshot: `test/us_cam_motor_cp2.py`.
