/*
* Brian R Taylor
* brian.taylor@bolderflight.com
* 
* Copyright (c) 2021 Bolder Flight Systems Inc
*
* Permission is hereby granted, free of charge, to any person obtaining a copy
* of this software and associated documentation files (the “Software”), to
* deal in the Software without restriction, including without limitation the
* rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
* sell copies of the Software, and to permit persons to whom the Software is
* furnished to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included in
* all copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
* FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
* IN THE SOFTWARE.
*/

#include "src/mpu9250.h"
#include <SD.h>

/* SD card setup */
const int chipSelect = BUILTIN_SDCARD;
File IMUdata; 

/* Mpu9250 object */
bfs::Mpu9250 imu;

void setup() {
  /* Serial to display data */
  Serial.begin(115200);

  while(!Serial) {
    /* SD card initalization*/
    Serial.print("Initializing SD card: ");
    if (!SD.begin(chipSelect)) {
      Serial.print("initialization failed. Check card.");
      return;
    }
    Serial.println("initialization done.");
  }

  /* Open/create CSV */
  IMUdata = SD.open("imudata.csv, FILE_WRITE");
  if (IMUdata) {
    /* Write headers */
    IMUdata.println("Millisec,AccelX,AccelY,AccelZ,GyroX,GyroY,GyroZ,TempC");
    IMUdata.close();
  }
  else {
    Serial.println("Error opening imudata.csv");
  }

  /* Start the I2C bus */
  Wire.begin();
  Wire.setClock(400000);

  /* I2C bus,  0x68 address */
  imu.Config(&Wire, bfs::Mpu9250::I2C_ADDR_PRIM);

  /* Initialize and configure IMU */
  if (!imu.Begin()) {
    Serial.println("Error initializing communication with IMU");
    while(1) {}
  }

  /* Set the sample rate divider */
  if (!imu.ConfigSrd(19)) {
    Serial.println("Error configured SRD");
    while(1) {}
  }

  
}

void loop() {
  /*
  //all vars
  if (imu.Read()) {
    Serial.print(imu.new_imu_data());
    Serial.print("\t");
    Serial.print(imu.new_mag_data());
    Serial.print("\t");
    Serial.print(imu.accel_x_mps2());
    Serial.print("\t");
    Serial.print(imu.accel_y_mps2());
    Serial.print("\t");
    Serial.print(imu.accel_z_mps2());
    Serial.print("\t");
    Serial.print(imu.gyro_x_radps());
    Serial.print("\t");
    Serial.print(imu.gyro_y_radps());
    Serial.print("\t");
    Serial.print(imu.gyro_z_radps());
    Serial.print("\t");
    Serial.print(imu.mag_x_ut());
    Serial.print("\t");
    Serial.print(imu.mag_y_ut());
    Serial.print("\t");
    Serial.print(imu.mag_z_ut());
    Serial.print("\t");
    Serial.print(imu.die_temp_c());
    Serial.print("\n");
  }
  */

  unsigned long var0 = millis();
  int var1 = imu.accel_x_mps2();
  int var2 = imu.accel_y_mps2();
  int var3 = imu.accel_z_mps2();
  int var4 = imu.gyro_x_radps();
  int var5 = imu.gyro_y_radps();
  int var6 = imu.gyro_z_radps();
  int var7 = imu.die_temp_c();
  
  String csvRow = String(var0) + "," + String(var1) + "," + String(var2) + "," + String(var3) + "," + String(var4) + "," + String(var5) + "," + String(var6) + "," + String(var7);

  Serial.println(csvRow);

  IMUdata = SD.open("imudata.csv", FILE_WRITE);
  if (IMUdata) {
    IMUdata.println(csvRow);
    IMUdata.close();
  } else {
    Serial.println("Error writing to datalog.csv");
  }
  delay(100);
}
