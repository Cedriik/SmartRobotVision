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
- 2026-03-11: Added a 2s camera-clearance resume gate in `test/us_cam_motor.py`: any camera blockage pulse resets the timer, and forward motion won’t start until camera has been continuously clear for `CAM_CLEAR_CONFIRM_SECONDS` and front ultrasonic is clear (`FRONT_CLEAR_CM=20.0`). Checkpoint snapshot: `test/us_cam_motor_cp3.py`.
- 2026-03-11: Updated `stop_motors()` in `test/us_cam_motor.py` to also drive ENA/ENB LOW when not using PWM on EN pins (`USE_PWM_EN=False`), to ensure a hard motor stop. Checkpoint snapshot: `test/us_cam_motor_cp4.py`.
- 2026-03-11: Added a simple hold gate in `test/us_cam_motor.py`: while (camera blocked) AND (front ultrasonic < `FRONT_STOP_CM`), keep motors stopped and do not start/turn; this prevents immediate movement during pulsing blockage. Checkpoint snapshot: `test/us_cam_motor_cp5.py`.
- 2026-03-11: Added Checkpoint/us_cam_motor_frontworking.py (copied from test/us_cam_motor_cp5.py).
- 2026-03-19: Created `test/init_path_AllUS.py` with front/left/right ultrasonic handling. Front blockage stops the robot, averages left/right clearance for 2s, turns only toward a side `>=20 cm`, and keeps the predetermined path commented out for safe staging. Rollback snapshots: `Rollback/init_path_AllUS.py`.
- 2026-03-19: Replaced `test/init_path.py` with the current all-ultrasonic logic from `test/init_path_AllUS.py`. Rollback snapshot: `Rollback/init_path.py`.
- 2026-03-19: Updated `test/init_path.py` and `test/init_path_AllUS.py` to run a single guarded 2s forward test in `main()` instead of idling.
- 2026-03-19: Pushed `test/init_path.py`, `test/init_path_AllUS.py`, `Rollback/init_path.py`, and `Rollback/init_path_AllUS.py` to GitHub from the local PC repo as commit `4daebcf` (`Add all-ultrasonic init path logic`) because the Pi repo could not authenticate to the HTTPS remote.
- 2026-03-19: Saved `Rollback/init_path_cp1.py` as a pre-PID checkpoint before refactoring `Rollback/init_path.py` for tight-passage work.
- 2026-03-19: Reworked `Rollback/init_path.py` from threshold-only forward motion into side-PID forward control with differential left/right PWM, ultrasonic median sampling, front hard-stop gating, side-wall references (`LEFT_REFERENCE_CM=7.8`, `RIGHT_REFERENCE_CM=8.0`), and lower cruise PWM for narrow clearances.
- 2026-03-19: Tuned `Rollback/init_path.py` over several live runs from Pi debug output: added then softened right-turn bias, reduced derivative aggression, reset integral on center crossing, lowered turn PWM, and relaxed `TURN_OPENING_CLEAR_CM` from `20.0` to `10.0` to match the measured turn space.
- 2026-03-20: Saved `Rollback/init_path_cp2.py` as a checkpoint of the right-biased/trim-tuned path controller before the next timing pass.
- 2026-03-20: Updated `Rollback/init_path.py` for faster test iteration by halving the ultrasonic trigger interval (`MIN_INTERVAL_S 0.06 -> 0.03`), halving the PID loop target (`PID_LOOP_DT_S 0.05 -> 0.025`), and reducing median-sampling sleep (`0.02 -> 0.01`).
- 2026-03-20: Raised the right-side correction ceiling in `Rollback/init_path.py` (`PID_MAX_RIGHT_ADJUST 18.0 -> 22.0`) and extended debug output to print unclamped `raw` control so PID saturation can be distinguished from motor-response limits during tuning.
- 2026-03-20: Saved `Rollback/init_path_cp3.py` before adding broader slant-recovery tuning to the timed-turn + PID-straight controller.
- 2026-03-20: Saved `Rollback/init_path_CurrentConfig.py` as the exact current configuration snapshot before the next straight-line tuning pass.
- 2026-03-20: Moved the route definition to top-level `PATH_STEPS` in `Rollback/init_path.py` for faster debugging, kept turns as fixed-timing `TURN_PWM_DUTY=80`, and increased straight-line recovery gains (`PID_KP`, `PID_KI`, `PID_KD`, positive exponential/integral weighting, right-side boost).
- 2026-03-20: Added a two-level straight recovery model in `Rollback/init_path.py`: normal fine balancing for small in-range wall error, plus hysteresis-based `coarse` recovery for strong slant conditions using side-delta/error thresholds, with front ultrasonic used only as a near-obstacle verification gate rather than continuous steering input.
- 2026-03-20: Saved `Rollback/init_path_PIDTuned.py` as the PID-tuned snapshot before reverting turn execution in `Rollback/init_path.py` back to fixed-timing rotation at `TURN_PWM_DUTY=80`; removed ultrasonic pre/post turn checks so straight PID recovery handles post-rotation alignment.
- 2026-03-20: Created `~/Robot/Draft` on the Raspberry Pi and copied the current `~/Robot/Rollback/init_path.py` there as `~/Robot/Draft/init_path.py`. Mirrored the same draft into the local repo as `Draft/init_path.py` for Git tracking.
