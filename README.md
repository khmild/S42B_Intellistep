# Intellistep

A rework of the terribly coded firmware from the BTT S42B v2 boards

I am currently working on redoing a lot of the functions found in the firmware. A list of the current things that are being worked on is in the Projects tab. If you don't see your issue there, then please let me know.

***Note: For the time being, all serial and CAN messages should start with "<" and end with ">". The serial baud rate is 115200.***

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
- M352 (ex M320 S1) - Sets the direction pin inversion for the motor (0 is standard, 1 is inverted)
- M353 (ex M353 S1) - Sets the enable pin inversion for the motor (0 is standard, 1 is inverted)
- M354 (ex M354 S1) - Sets if the motor dip switches were installed incorrectly (reversed) (0 is standard, 1 is inverted)
- M500 (ex M500) - Saves the currently loaded parameters into flash
- M907 (ex M907 V3000) - Sets the current in mA

## Credits

- BTT - Original code
- CAP1Sup - Main rewrite of code
- Misfittech - The Smart Stepper [project](https://github.com/Misfittech/nano_stepper). This was a large help when writing the correct stepping code.
