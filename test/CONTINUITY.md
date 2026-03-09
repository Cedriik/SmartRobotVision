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
