#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <Arduino.h>

// Wheel speed PID + DRV8833 PWM driver (two inputs per motor).
class MotorControl {
  private:
    int pin1;
    int pin2;
    int ch1;
    int ch2;

    float kp;
    float ki;
    float kd;
    float integral;
    float prevError;
    float outMin;
    float outMax;

  public:
    MotorControl(int p1, int p2, int c1, int c2);
    void begin(float p, float i, float d);
    float computePID(float target, float current, float dt);
    void drive(float pwmOutput);
    void stop();
};

#endif
