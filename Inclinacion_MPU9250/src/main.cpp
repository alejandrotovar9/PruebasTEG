#include "MPU9250.h"
#include <Wire.h>
#include <SPI.h>
#include <Arduino.h>

MPU9250 mpu;

void print_calibration() {
    Serial.println("< calibration parameters >");
    Serial.println("accel bias [g]: ");
    Serial.print(mpu.getAccBiasX() * 1000.f / (float)MPU9250::CALIB_ACCEL_SENSITIVITY);
    Serial.print(", ");
    Serial.print(mpu.getAccBiasY() * 1000.f / (float)MPU9250::CALIB_ACCEL_SENSITIVITY);
    Serial.print(", ");
    Serial.print(mpu.getAccBiasZ() * 1000.f / (float)MPU9250::CALIB_ACCEL_SENSITIVITY);
    Serial.println();
    Serial.println("gyro bias [deg/s]: ");
    Serial.print(mpu.getGyroBiasX() / (float)MPU9250::CALIB_GYRO_SENSITIVITY);
    Serial.print(", ");
    Serial.print(mpu.getGyroBiasY() / (float)MPU9250::CALIB_GYRO_SENSITIVITY);
    Serial.print(", ");
    Serial.print(mpu.getGyroBiasZ() / (float)MPU9250::CALIB_GYRO_SENSITIVITY);
    Serial.println();
    Serial.println("mag bias [mG]: ");
    Serial.print(mpu.getMagBiasX());
    Serial.print(", ");
    Serial.print(mpu.getMagBiasY());
    Serial.print(", ");
    Serial.print(mpu.getMagBiasZ());
    Serial.println();
    Serial.println("mag scale []: ");
    Serial.print(mpu.getMagScaleX());
    Serial.print(", ");
    Serial.print(mpu.getMagScaleY());
    Serial.print(", ");
    Serial.print(mpu.getMagScaleZ());
    Serial.println();
}

void print_roll_pitch_yaw()
{
  Serial.print(">Yaw:");
  Serial.println(mpu.getYaw(), 2);
  Serial.print(">Pitch:");
  Serial.println(mpu.getPitch(), 2);
  Serial.print(">Roll:");
  Serial.println(mpu.getRoll(), 2);
}

void print_acceleration()
{
  Serial.print(">AclX:");
  Serial.println(mpu.getLinearAccX());
  Serial.print(">AclY:");
  Serial.println(mpu.getLinearAccY());
  Serial.print(">AclZ:");
  Serial.println(mpu.getLinearAccZ());
}

void printAngles(){
  //Getting Euler angles  
  Serial.print(">EulerX [deg]:");
  Serial.println(mpu.getEulerX());
  Serial.print(">EulerY [deg]:");
  Serial.println(mpu.getEulerY());
  Serial.print(">EulerZ [deg]:");
  Serial.println(mpu.getEulerZ());
}


void setup() {
    Serial.begin(115200);
    Wire.begin();
    delay(2000);

    if (!mpu.setup(0x68)) {  // change to your own address
        while (1) {
            Serial.println("MPU connection failed. Please check your connection with `connection_check` example.");
            delay(5000);
        }
    }

    // // calibrate anytime you want to
    // Serial.println("Accel Gyro calibration will start in 5sec.");
    // Serial.println("Please leave the device still on the flat plane.");
    // mpu.verbose(true);
    // delay(5000);
    // mpu.calibrateAccelGyro();

    // Serial.println("Mag calibration will start in 5sec.");
    // Serial.println("Please Wave device in a figure eight until done.");
    // delay(5000);
    // mpu.calibrateMag();

    // print_calibration();
    // mpu.verbose(false);
}

void loop() {
    if (mpu.update()) {
        static uint32_t prev_ms = millis();
        if (millis() > prev_ms + 25) {
            //print_roll_pitch_yaw();
            //print_acceleration();
            printAngles();
            prev_ms = millis();
        }
    }
}