# Intellistep

GCode Commands
  - M93 (ex M93 V1.8) - Sets the angle of a full step. This value should be 1.8° or 0.9°
  - M306 (ex M306 P1 I1 D1) - Sets the PID values for the motor
  - M307 (ex M307) - Runs an autotune sequence for the PID loop
  - M350 (ex M350 V16) - Sets the microstepping divisor for the motor. This value can be 1, 2, 4, 8, 16, or 32
  - M500 (ex M500) - Saves the currently loaded parameters into flash
  - M907 (ex M907 V3000) - Sets the current in mA

Special thanks to
  - [Afiskon](https://github.com/afiskon/) for his [SSD1306 library](https://github.com/afiskon/stm32-ssd1306)