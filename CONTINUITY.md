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
- 2026-03-11: Updated ~/Robot/test/us_cam_motor.py to serve camera stream via Flask (/video_feed) and prefer V4L2 capture to reduce GStreamer warnings. Snapshotted to ~/Robot/Rollback/us_cam_motor.py.
- 2026-03-11: Added HSV color detection overlays (Red/Green/Black) and obstruction HUD to the Flask stream in ~/Robot/test/us_cam_motor.py. Snapshotted to ~/Robot/Rollback/us_cam_motor.py.
- 2026-03-11: Updated `~/Robot/test/us_cam_motor.py` debug output to hide pulse microseconds and print a compact left/right line; increased rotation duration to 2.0s.
- 2026-03-11: Changed ~/Robot/test/us_cam_motor.py --debug output to a single-line ront/right/left cm status (throttled) and added --debug-errors for sensor error prints. Snapshotted to ~/Robot/Rollback/us_cam_motor.py.
- 2026-03-11: Kept ultrasonic sampling and --debug output running during front blockage/turn decision by moving US reads into a background sampler and using a non-blocking state machine.
- 2026-03-11: Added front ultrasonic-only stop lock (>=2s blocked) to `~/Robot/test/us_cam_motor.py`, confirmed by 1s median, independent of camera blockage.
- 2026-03-11: Fixed front median sampling calls in US-only lock (`sampler.median(front, ...)`) in `~/Robot/test/us_cam_motor.py`.
- 2026-03-11: Added 2s camera-clearance gating before resuming forward motion in ~/Robot/test/us_cam_motor.py (timer resets on any blockage pulse). Forward resumes only when camera is continuously clear and front ultrasonic clearance is 20cm. Rollback refreshed: ~/Robot/Rollback/us_cam_motor.py.
- 2026-03-11: Updated stop_motors() in ~/Robot/test/us_cam_motor.py to also drive ENA/ENB LOW when USE_PWM_EN is false, ensuring a hard motor stop (not just IN pins low). Rollback refreshed: ~/Robot/Rollback/us_cam_motor.py.
- 2026-03-11: Added a simple hold gate in ~/Robot/test/us_cam_motor.py: while (camera blocked) AND (front ultrasonic < FRONT_STOP_CM), keep motors stopped and do not start/turn; this prevents immediate movement during pulsing blockage. Rollback refreshed: ~/Robot/Rollback/us_cam_motor.py.
- 2026-03-11: Created ~/Robot/Checkpoint and saved cp5 as ~/Robot/Checkpoint/us_cam_motor_frontworking.py.
- 2026-03-19: Created ~/Robot/test/init_path_AllUS.py with front/left/right ultrasonic handling. Front blockage stops the robot, averages left/right clearance for 2s, turns only toward a side >=20cm, and keeps the predetermined path commented out for safe staging. Rollback snapshot: ~/Robot/Rollback/init_path_AllUS.py.

- 2026-03-19: Replaced ~/Robot/test/init_path.py with the current ~/Robot/test/init_path_AllUS.py logic. Rollback snapshot: ~/Robot/Rollback/init_path.py.

- 2026-03-19: Updated ~/Robot/test/init_path_AllUS.py and ~/Robot/test/init_path.py to run a 2s guarded forward test in main() instead of idling. Rollback snapshots refreshed in ~/Robot/Rollback.

- 2026-03-19: Pushed test/init_path.py, test/init_path_AllUS.py, Rollback/init_path.py, and Rollback/init_path_AllUS.py to GitHub from the local PC repo as commit 4daebcf (Add all-ultrasonic init path logic) because the Pi repo could not authenticate to the HTTPS remote.

- 2026-03-19: Re-enabled the predetermined path in ~/Robot/test/init_path_AllUS.py and ~/Robot/test/init_path.py, replacing the temporary 2s guarded forward test. Rollback snapshots refreshed in ~/Robot/Rollback.

- 2026-03-19: Changed ~/Robot/test/init_path_AllUS.py and ~/Robot/test/init_path.py so straight/forward path steps no longer use timers; they now move until the front ultrasonic stops them. Timed path entries remain only for turns. Rollback snapshots refreshed in ~/Robot/Rollback.

- 2026-03-19: Added 1s side-ultrasonic verification before each predetermined left/right turn in ~/Robot/test/init_path_AllUS.py and ~/Robot/test/init_path.py, plus a 1s post-turn stabilization/front check to catch misalignment or blockage after rotation. Rollback snapshots refreshed in ~/Robot/Rollback.

- 2026-03-19: Replaced ~/Robot/test/init_path.py from the current init_path_AllUS.py baseline and added a first-pass proportional side-alignment controller for forward motion. While moving straight/forward, left/right ultrasonic averages now bias ENA/ENB PWM to partially correct heading; ~/Robot/test/init_path_AllUS.py was left unchanged. Rollback snapshot refreshed: ~/Robot/Rollback/init_path.py.
- 2026-03-22: Moved ~/Robot/test/camera_test.py and ~/Robot/test/motor_test.py into ~/Robot/ per user request.
- 2026-03-22: Moved ~/Robot/camera_test.py and ~/Robot/motor_test.py into ~/ per user clarification.
- 2026-03-22: Created ~/robot.py merging Flask camera stream with pigpio motor/servo/ultrasonic control, adding Yellow pause latch, Green resume latch, and camera+front-ultrasonic obstacle confirmation with timed side turns + straight PID.
- 2026-03-23: Increased init_path forward PWM tuning for faster movement. Updated BASE_CRUISE=56, BASE_TIGHT=40, BASE_CRAWL=32, MAX_DRIVE=80 in ~/init_path.py and ~/Robot/test/init_path.py. Backups: ~/init_path.py.bak_ and ~/Robot/Rollback/init_path_speed_bak_.py.
- 2026-03-23: Renamed init_path speed-tuning backups to timestamped files: ~/init_path.py.bak_20260323_003555 and ~/Robot/Rollback/init_path_speed_bak_20260323_003555.py.
- 2026-03-23: Added ~/initial_path_camera.py and ~/Robot/test/initial_path_camera.py as a standalone camera+ultrasonic prototype. It uses lower-frame line/edge detection for straight alignment and coarse-turn plus micro-turn pulses for post-turn correction.
- 2026-03-23: Updated ~/initial_path_camera.py and ~/Robot/test/initial_path_camera.py to use corridor-centroid camera guidance first. The camera now tests both dark-floor and bright-floor masks with image moments, and falls back to edge/line contour guidance only if corridor detection is weak.
- 2026-03-23: Updated ~/initial_path_camera.py and ~/Robot/test/initial_path_camera.py to start idle behind web controls. Added /start, /pause, and /stop_server controls, stronger turn PWM, pause-safe motion loops, and explicit front-clear checks before coarse and micro turn pulses.
- 2026-03-23: Updated ~/initial_path_camera.py and ~/Robot/test/initial_path_camera.py with camera-based turn verification using corridor tilt/centering, extended path turn durations from 1.0s to 2.0s, and in-page web Start/Pause controls that stay on the main URL via POST/fetch.
- 2026-03-23: Added a post-turn ultrasonic verification window to ~/initial_path_camera.py and ~/Robot/test/initial_path_camera.py. After the coarse turn, the code now waits about 1.2s to average front/left/right distances before deciding whether micro-correction is still needed.
- 2026-03-23: Updated ~/initial_path_camera.py and ~/Robot/test/initial_path_camera.py with easier path editing via PATH_SEQUENCE + STEP_DURATION_BY_DIRECTION, lowered MAX_DRIVE_PWM_DUTY to 65, added left/right camera angle blockage checks, replaced fixed micro-step count with adaptive pulses (0.06s then 0.12s within a bounded correction window), and added an in-page /restart control that resets the path to step 0.
- 2026-03-23: Pushed initial_path_camera prototype changes to GitHub main (commit e16b2c7). Updated ~/start_robot.sh to prefer ~/initial_path_camera.py and display the wlan0 AP IP, and added /etc/update-motd.d/98-robot-ap-info to show RaspberryPiRobot AP info and Vision UI URL on login.
- 2026-03-23: Pushed initial_path_camera prototype changes to GitHub main (commit e16b2c7). Updated ~/start_robot.sh to prefer ~/initial_path_camera.py and display the wlan0 AP IP, and added /etc/update-motd.d/98-robot-ap-info to show RaspberryPiRobot AP info and Vision UI URL on login.
- 2026-03-23: Replaced short micro-turn correction timings in ~/initial_path_camera.py and ~/Robot/test/initial_path_camera.py with longer mini-pulse timings. New correction pulses use 0.14s first pulse, 0.28s follow-up pulses, 0.08s settle time, 3.2s max correction window, and TURN_MINI_PWM_DUTY=86.
- 2026-03-23: Extended ~/initial_path_camera.py and ~/Robot/test/initial_path_camera.py with combined camera scene-alignment checks. Added corridor direction from Hough lines, vanishing-style corridor offset, frame-to-frame feature tracking stability, and integrated those signals into the post-turn camera alignment decision without replacing the existing side openness or ultrasonic checks.

2026-03-23: Adjusted mini-turn timing so the first correction pulse is stronger (0.24s) and follow-up pulses are shorter (0.14s), based on live motor behavior where the first pulse rotates reliably.

2026-03-23: Added paused-only web editor to initial_path_camera.py for turn timing and route editing. Added /control_state, /set_turn_seconds, /path/add, /path/clear, and /path/reset_default. Route edits now mark the path for restart-from-step-0 on the next START.

2026-03-23: Added checkpoint snapshot Checkpoint/init_path_camera_cp1.py from the current initial_path_camera.py state before further turn-trigger changes.
