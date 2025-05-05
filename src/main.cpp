#include <Arduino.h>
#include <BMI088.h>
#include <Wire.h>
#include <stdio.h>

#define WIRE Wire
#define I2C_ADDRESS_ACCEL_1 0x19
#define I2C_ADDRESS_GYRO_1 0x69

Bmi088 bmi(Wire,I2C_ADDRESS_ACCEL_1,I2C_ADDRESS_GYRO_1);

int status = 0;

void setup() {
  Serial.begin(115200);

  Serial.printf("Hello BMI088\n");

  // Accelerometer Setup
  status = bmi.begin();
  bmi.setOdr(Bmi088::ODR_1000HZ);
  bmi.setRange(Bmi088::ACCEL_RANGE_6G,Bmi088::GYRO_RANGE_500DPS);
}

void loop() {
  // Grab Accelerometer Values  
  bmi.readSensor();
  char buf[100] = {};
  float t, ax, ay, az, gx, gy, gz;
  t = bmi.getTemperature_C();
  ax = bmi.getAccelX_mss();
  ay = bmi.getAccelY_mss();
  az = bmi.getAccelZ_mss();
  gx = (bmi.getGyroX_rads() / PI) * 180;
  gy = (bmi.getGyroY_rads() / PI) * 180;
  gz = (bmi.getGyroY_rads() / PI) * 180;

  Serial.print("Temp: ");
  Serial.print(t, 4);
  Serial.print("°C Gyro: ");
  Serial.print(gx, 4);
  Serial.print(" ");
  Serial.print(gy, 4);
  Serial.print(" ");
  Serial.print(gz, 4);
  Serial.print(" Accel: ");
  Serial.print(ax, 4);
  Serial.print(" ");
  Serial.print(ay, 4);
  Serial.print(" ");
  Serial.print(az, 4);
  Serial.print(" Status: ");
  Serial.println(status);

  delay(500);
}