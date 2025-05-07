#include <Arduino.h>
#include <BMI088.h>
#include <Wire.h>
#include <stdio.h>

#define WIRE Wire
#define I2C_ADDRESS_ACCEL_1 0x19
#define I2C_ADDRESS_GYRO_1 0x69

Bmi088 bmi(Wire,I2C_ADDRESS_ACCEL_1,I2C_ADDRESS_GYRO_1);

int status = 0;
float xDisplace, yDisplace, zDisplace, time = 0;
unsigned long lastTime = 0;

void setup() {
  Serial.begin(115200);

  Serial.println("Hello BMI088");

  // Accelerometer Setup
  status = bmi.begin();
  bmi.setOdr(Bmi088::ODR_1000HZ);
  bmi.setRange(Bmi088::ACCEL_RANGE_6G,Bmi088::GYRO_RANGE_500DPS);
  lastTime = millis();
}

// hopefully does what i want it to
float angDisplace(float angularVelocity, float time) {
  if(angularVelocity == 0)
    return 0;
  return angularVelocity * (time/1000);
}

void loop() {

  unsigned long now = millis();
  float deltaTime = (now - lastTime);
  lastTime = now; 

  // Grab Accelerometer Values  
  bmi.readSensor();
  
  char buf[100] = {};
  float t, ax, ay, az, gx, gy, gz, xChange = 0, yChange = 0, zChange = 0;

  t = bmi.getTemperature_C();
  ax = bmi.getAccelX_mss();
  ay = bmi.getAccelY_mss();
  az = bmi.getAccelZ_mss();

  gx = bmi.getGyroX_rads();
  gy = bmi.getGyroY_rads();
  gz = bmi.getGyroZ_rads();

  xChange = angDisplace(gx, deltaTime);
  yChange = angDisplace(gy, deltaTime);
  zChange = angDisplace(gz, deltaTime);

  xDisplace += xChange;
  yDisplace += yChange;
  zDisplace += zChange;

  float xDisplay = (xDisplace / PI) * 180;
  float yDisplay = (yDisplace / PI) * 180;
  float zDisplay = (zDisplace / PI) * 180;

  if(time == 500) {
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
    Serial.print("X: ");
    Serial.print(xDisplay, 4);
    Serial.print(" Y: ");
    Serial.print(yDisplay, 4);
    Serial.print(" Z: ");
    Serial.println(zDisplay, 4);
    time = 0;
  }
  time++;

  //delay(20);
}

