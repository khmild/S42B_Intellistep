# Intellistep

A rework of the terribly coded firmware from the BTT S42B v2 boards

I am currently working on redoing a lot of the functions found in the firmware. A list of the current things that are being worked on is in the Projects tab. If you don't see your issue there, then please let me know.

New Features:

- Redone stepping setup
- Redone serial commands (based on gcode)

Future Features:

- Manual tuning utility for manual tuning
- Much better stepper calibration
- New menu items for setting PID parameters

GCode Table

- M93 (ex M93 V1.8) - Sets the angle of a full step. This value should be 1.8° or 0.9°
- M306 (ex M306 P1 I1 D1) - Sets the PID values for the motor
- M307 (ex M307) - Runs an autotune sequence for the PID loop
- M308 (ex M308) - Runs the manual PID tuning interface. Serial is filled with encoder angles
- M350 (ex M350 V16) - Sets the microstepping divisor for the motor. This value can be 1, 2, 4, 8, 16, or 32
- M500 (ex M500) - Saves the currently loaded parameters into flash
- M907 (ex M907 V3000) - Sets the current in mA
