#include <Arduino.h>
#include <BMI088.h>
#include <Wire.h>
#include <stdio.h>

// #define I2C_SCL 0
// #define I2C_SDA 0 
// #define I2C_PORT_NUMBER 0
#define I2C_ADDRESS_ACCEL_1 0x18
#define I2C_ADDRESS_GYRO_1 0x68

Bmi088 bmi(Wire,I2C_ADDRESS_ACCEL_1,I2C_ADDRESS_GYRO_1);

void setup() {
  Serial.begin(115200);

  printf("Hello BMI088\n");

  // Accelerometer Setup
  bmi.begin();
  bmi.setOdr(Bmi088::ODR_1000HZ);
  bmi.setRange(Bmi088::ACCEL_RANGE_6G,Bmi088::GYRO_RANGE_500DPS);
}

void loop() {
  // Grab Accelerometer Values  
  bmi.readSensor();
  float t, ax, ay, az, gx, gy, gz;
  t = bmi.getTemperature_C();
  ax = bmi.getAccelX_mss();
  ay = bmi.getAccelY_mss();
  az = bmi.getAccelZ_mss();
  gx = bmi.getGyroX_rads();
  gy = bmi.getGyroY_rads();
  gz = bmi.getGyroZ_rads();
  uint64_t time_ps;
  time_ps = bmi.getTime_ps();

  printf("Time: %d Temp: %d°C Gyro: %f %f %f Accel: %f %f %f\n",
          time_ps, t,           gx, gy, gz,     ax, ay, az);
  // printf("System Status: %d", status);

  delay(5);
}