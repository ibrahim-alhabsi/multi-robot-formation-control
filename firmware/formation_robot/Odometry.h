#ifndef ODOMETRY_H
#define ODOMETRY_H

#include <Arduino.h>
#include <Wire.h>
#include "AS5600.h"
#include <MPU6050_tockn.h>
#include <Kalman.h>

// Wheel odometry with Kalman-fused heading (encoder rate + IMU yaw).
class Odometry {
  private:
    AS5600 *sensorL;
    AS5600 *sensorR;
    MPU6050 *imu;
    Kalman kalmanTheta;

    float wheelBase;
    float wheelDiameter;
    const float ppr = 4096.0f;  // AS5600: 12-bit
    const float pi = 3.14159265f;

    float x;
    float y;
    float theta;

    long lastStepsL;
    long lastStepsR;
    float currentV;
    float currentW;
    float speedL;
    float speedR;
    unsigned long lastUpdateMicros;

  public:
    Odometry(AS5600 *sL, AS5600 *sR, MPU6050 *mpu, float wBase, float wDiam);

    void begin();
    void update();
    void setPose(float startX, float startY, float startThetaRad);

    float getX() { return x; }
    float getY() { return y; }
    float getTheta() { return theta; }
    float getSpeedL() { return speedL; }
    float getSpeedR() { return speedR; }
    float getDistanceTo(float tx, float ty);
};

#endif
