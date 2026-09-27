#include "MotorControl.h"

MotorControl::MotorControl(int p1, int p2, int c1, int c2) {
    pin1 = p1;
    pin2 = p2;
    ch1 = c1;
    ch2 = c2;
    kp = 0.0f;
    ki = 0.0f;
    kd = 0.0f;
    integral = 0.0f;
    prevError = 0.0f;
    outMin = -255.0f;
    outMax = 255.0f;
}

void MotorControl::begin(float p, float i, float d) {
    kp = p;
    ki = i;
    kd = d;

    // 5 kHz, 8-bit PWM (Arduino-ESP32 core 2.x LEDC API)
    ledcSetup(ch1, 5000, 8);
    ledcSetup(ch2, 5000, 8);
    ledcAttachPin(pin1, ch1);
    ledcAttachPin(pin2, ch2);
}

float MotorControl::computePID(float target, float current, float dt) {
    if (dt <= 0.0f || dt > 0.1f) {
        return 0.0f;
    }

    float error = target - current;
    integral += error * dt;
    integral = constrain(integral, -100.0f, 100.0f);  // anti-windup clamp

    float derivative = (error - prevError) / dt;
    prevError = error;

    float output = (kp * error) + (ki * integral) + (kd * derivative);
    return constrain(output, outMin, outMax);
}

void MotorControl::drive(float pwmOutput) {
    float pwr = fabsf(pwmOutput);
    pwr = constrain(pwr, 0.0f, 255.0f);

    // Small commands: coast
    if (pwr < 8.0f) {
        ledcWrite(ch1, 0);
        ledcWrite(ch2, 0);
        return;
    }

    // Minimum PWM to overcome static friction in the gearbox
    if (pwr < 35.0f) {
        pwr = 35.0f;
    }

    if (pwmOutput >= 0.0f) {
        ledcWrite(ch1, (int)pwr);
        ledcWrite(ch2, 0);
    } else {
        ledcWrite(ch1, 0);
        ledcWrite(ch2, (int)pwr);
    }
}

void MotorControl::stop() {
    ledcWrite(ch1, 0);
    ledcWrite(ch2, 0);
    integral = 0.0f;
    prevError = 0.0f;
}
