there are two parts to the imu right now:

- sensortest
- scanner.ino 

sensortest can be run in Arduino ide. its from an imu library for the mpu9250 called ivensense. if you want more details pull up the GitHub repo for ivensense.

next is scanner.ino. this is part of the base teensy 4.1 library and it checks if there are any i2c devices connected.

scl is connected to 19, sda is 18, then we do ground and 3v. 