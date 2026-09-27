#include "Odometry.h"

Odometry::Odometry(AS5600 *sL, AS5600 *sR, MPU6050 *mpu, float wBase, float wDiam) {
    sensorL = sL;
    sensorR = sR;
    imu = mpu;
    wheelBase = wBase;
    wheelDiameter = wDiam;

    kalmanTheta.setAngle(0.0f);
    kalmanTheta.setQangle(0.0001f);
    kalmanTheta.setRmeasure(0.01f);

    x = 0.0f;
    y = 0.0f;
    theta = 0.0f;
    lastStepsL = 0;
    lastStepsR = 0;
    currentV = 0.0f;
    currentW = 0.0f;
    speedL = 0.0f;
    speedR = 0.0f;
    lastUpdateMicros = 0;
}

void Odometry::begin() {
    sensorL->resetCumulativePosition();
    sensorR->resetCumulativePosition();
    lastStepsL = sensorL->getCumulativePosition();
    lastStepsR = sensorR->getCumulativePosition();
    lastUpdateMicros = micros();
}

void Odometry::setPose(float startX, float startY, float startThetaRad) {
    x = startX;
    y = startY;
    theta = startThetaRad;
    kalmanTheta.setAngle(theta * 180.0f / PI);
}

void Odometry::update() {
    unsigned long currentMicros = micros();
    float dt = (float)(currentMicros - lastUpdateMicros) / 1000000.0f;

    if (dt <= 0.0f) {
        lastUpdateMicros = currentMicros;
        return;
    }

    // Long gap (e.g. blocked task): resync encoders instead of integrating a jump
    if (dt > 0.25f) {
        lastStepsL = sensorL->getCumulativePosition();
        lastStepsR = sensorR->getCumulativePosition();
        speedL = 0.0f;
        speedR = 0.0f;
        currentV = 0.0f;
        currentW = 0.0f;
        imu->update();
        lastUpdateMicros = currentMicros;
        return;
    }

    lastUpdateMicros = currentMicros;

    long currL = sensorL->getCumulativePosition();
    long currR = sensorR->getCumulativePosition();

    // Wheel travel [mm]; right encoder is mirrored, so its sign is flipped
    float dL = ((currL - lastStepsL) / ppr) * (pi * wheelDiameter);
    float dR = -((currR - lastStepsR) / ppr) * (pi * wheelDiameter);

    lastStepsL = currL;
    lastStepsR = currR;

    // Heading fusion: encoder yaw rate drives the prediction, IMU yaw corrects it
    float encoderRate = ((dR - dL) / wheelBase) * (180.0f / PI) / dt;

    imu->update();
    float imuAngle = imu->getAngleZ();
    float fusedThetaDeg = kalmanTheta.getAngle(imuAngle, encoderRate, dt);
    theta = fusedThetaDeg * (PI / 180.0f);

    float dCenter = (dL + dR) * 0.5f;
    float dTheta = (dR - dL) / wheelBase;
    currentV = dCenter / dt;
    currentW = dTheta / dt;

    // First-order low-pass on wheel speeds (tau = 80 ms) for the PID feedback
    float tau = 0.08f;
    float alpha = tau / (tau + dt);
    speedL = (alpha * speedL) + ((1.0f - alpha) * (dL / dt));
    speedR = (alpha * speedR) + ((1.0f - alpha) * (dR / dt));

    x += dCenter * cosf(theta);
    y += dCenter * sinf(theta);

    if (theta > pi) theta -= 2.0f * pi;
    if (theta < -pi) theta += 2.0f * pi;
}

float Odometry::getDistanceTo(float targetX, float targetY) {
    float dx = targetX - x;
    float dy = targetY - y;
    return sqrtf((dx * dx) + (dy * dy));
}
