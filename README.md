# Intellistep

A rework of the terribly coded firmware from the BTT S42B v2 boards.

I am currently working on redoing a lot of the functions found in the firmware. A list of the current things that are being worked on is in the Projects tab. If you don't see your issue there, then please let me know.

***Note: This is a pre-release firmware still in beta testing. It is not ready for actual use yet. Only flash it if you would like to help with the development process. If you compile Intellistep and don't like it, you can always switch back to the production firmware using the file in the "precompiled" folder. The production firmware is `v2ProductionFirmware.bin`. Original BTT build of this project is `firmware-v2.0.bin`***

***Note: For the time being, all serial and CAN messages should start with "<" and end with ">". The serial baud rate is 115200.***

***Note: If you're having large oscillations in step correction, then try increasing the microstepping using the dip switches while increasing the microstep multiplier***

New Features:

- Redone stepping for higher torque and quieter operation
- Redone serial commands (based on gcode)
- Temperature readout on the display
- CAN subnetwork for driver configuration

Future Features:

- Motor and driver overtemp current reduction
- Dynamic current control based on speed and acceleration
- Manual tuning utility for manual PID tuning
- Much better stepper calibration
- New menu items for setting PID parameters

GCode Table

- M17 (ex M17) - Enables the motor (overrides enable pin)
- M18 / M84 (ex M18 or M84) - Disables the motor (overrides enable pin)
- M93 (ex M93 V1.8) - Sets the angle of a full step. This value should be 1.8° or 0.9°
- M115 (ex M115) - Prints out firmware information, consisting of the version and any enabled features.
- M116 (ex M116 S1 M"A message") - Simple forward command that will forward a message across the CAN bus. Can be used for pinging or allowing a Serial to connect to the CAN network
- M306 (ex M306 P1 I1 D1) - Sets the PID values for the motor
- M307 (ex M307) - Runs an autotune sequence for the PID loop
- M308 (ex M308) - Runs the manual PID tuning interface. Serial is filled with encoder angles
- M350 (ex M350 V16) - Sets the microstepping divisor for the motor. This value can be 1, 2, 4, 8, 16, or 32
- M352 (ex M320 S1) - Sets the direction pin inversion for the motor (0 is standard, 1 is inverted)
- M353 (ex M353 S1) - Sets the enable pin inversion for the motor (0 is standard, 1 is inverted)
- M354 (ex M354 S1) - Sets if the motor dip switches were installed incorrectly (reversed) (0 is standard, 1 is inverted)
- M355 (ex M355 V1.34) - Sets the microstep multiplier for the board. Allows to use multiple motors connected to the same mainboard pin.
- M356 (ex M356 V1 or M356 VX2) - Sets the CAN ID of the board. Can be set using the axis character or actual ID.
- M500 (ex M500) - Saves the currently loaded parameters into flash
- M907 (ex M907 R3000 or M907 P3000) - Sets the RMS or Peak current in mA

## Credits

- [BTT](https://github.com/bigtreetech) - [Original code](https://github.com/bigtreetech/BIGTREETECH-Stepper-Motor-Driver)
- [CAP1Sup](https://github.com/CAP1Sup) - Main rewrite of code
- [TazerReloaded](https://github.com/TazerReloaded) - Encoder protocol and communication, much faster snprintf string printing
- [Misfittech](https://github.com/Misfittech) - The [Smart Stepper project](https://github.com/Misfittech/nano_stepper). This was a large help when writing the correct stepping code.
- [Matt Fryer](https://github.com/MattFryer) - [Smoothed](https://github.com/MattFryer/Smoothed) library. This was a huge help when smoothing out the OLED values to make them more readable and accurate
- [Marlin](https://github.com/MarlinFirmware/Marlin) - Autobuilding templates
