// Decentralized distance-based formation control for differential-drive robots.
// Each robot runs this same firmware; set ROBOT_ID in config.h before flashing.

#include <Wire.h>
#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include "AS5600.h"
#include <MPU6050_tockn.h>
#include "config.h"
#include "Odometry.h"
#include "MotorControl.h"

// ---------------- Data types ----------------

typedef struct struct_message {
    int robot_id;
    float x;
    float y;
} struct_message;

struct RobotSnapshot {
    float x;
    float y;
    float theta;
    float speedL;
    float speedR;
    uint32_t stampMs;
};

// ---------------- Hardware objects ----------------

TwoWire I2C_L = TwoWire(0);
TwoWire I2C_R = TwoWire(1);

AS5600 as5600_L(&I2C_L);
AS5600 as5600_R(&I2C_R);
MPU6050 mpu6050(I2C_L);
Odometry robot(&as5600_L, &as5600_R, &mpu6050, WHEEL_BASE_MM, WHEEL_DIAMETER_MM);

MotorControl leftMotor(MOTOR_L_IN1, MOTOR_L_IN2, 0, 1);
MotorControl rightMotor(MOTOR_R_IN1, MOTOR_R_IN2, 2, 3);

// ---------------- Shared state (guarded by mutexes) ----------------

SemaphoreHandle_t stateMutex;
SemaphoreHandle_t peerMutex;
QueueHandle_t recvQueue;

RobotSnapshot localState = {};
struct_message robotStates[MAX_ROBOTS + 1] = {};
bool robotStateValid[MAX_ROBOTS + 1] = {};
uint32_t robotLastRecvMs[MAX_ROBOTS + 1] = {};

// ---------------- Graph helpers ----------------

static bool isValidRobotId(int robotId) {
    return robotId > 0 && robotId <= NUM_ROBOTS && robotId <= MAX_ROBOTS;
}

static bool isBlankMac(const uint8_t mac[6]) {
    for (int i = 0; i < 6; i++) {
        if (mac[i] != 0) return false;
    }
    return true;
}

static bool isActiveEdge(const FormationEdge &edge) {
    return edge.a <= NUM_ROBOTS && edge.b <= NUM_ROBOTS;
}

// If this edge touches selfId, return the robot on the other end.
static bool getNeighborFromEdge(const FormationEdge &edge, int selfId, int &neighborId) {
    if (!isActiveEdge(edge)) return false;
    if (edge.a == selfId) {
        neighborId = edge.b;
        return true;
    }
    if (edge.b == selfId) {
        neighborId = edge.a;
        return true;
    }
    return false;
}

static uint8_t countRequiredNeighbors() {
    uint8_t count = 0;
    for (size_t i = 0; i < NUM_FORMATION_EDGES; i++) {
        int neighborId;
        if (getNeighborFromEdge(formationEdges[i], ROBOT_ID, neighborId)) {
            count++;
        }
    }
    return count;
}

// ---------------- Utilities ----------------

static float rampTowards(float current, float target, float maxStep) {
    if (target > current + maxStep) return current + maxStep;
    if (target < current - maxStep) return current - maxStep;
    return target;
}

static bool copyLocalState(RobotSnapshot &out, TickType_t waitTicks = pdMS_TO_TICKS(1)) {
    if (stateMutex == NULL) return false;
    if (xSemaphoreTake(stateMutex, waitTicks) != pdTRUE) return false;
    out = localState;
    xSemaphoreGive(stateMutex);
    return true;
}

// Returns true only if the peer's data is fresh (younger than COMM_TIMEOUT_MS).
static bool copyPeerState(int robotId, struct_message &out, uint32_t &ageMs) {
    if (!isValidRobotId(robotId)) return false;
    if (peerMutex == NULL) return false;
    if (xSemaphoreTake(peerMutex, pdMS_TO_TICKS(1)) != pdTRUE) return false;

    out = robotStates[robotId];
    ageMs = millis() - robotLastRecvMs[robotId];
    bool connected = robotStateValid[robotId] && ageMs < COMM_TIMEOUT_MS;

    xSemaphoreGive(peerMutex);
    return connected;
}

// Move packets from the ESP-NOW callback queue into the peer table.
static void processIncomingPackets() {
    if (recvQueue == NULL) return;

    struct_message msg;
    while (xQueueReceive(recvQueue, &msg, 0) == pdTRUE) {
        if (!isValidRobotId(msg.robot_id) || msg.robot_id == ROBOT_ID) {
            continue;
        }

        if (peerMutex != NULL && xSemaphoreTake(peerMutex, pdMS_TO_TICKS(1)) == pdTRUE) {
            robotStates[msg.robot_id] = msg;
            robotStateValid[msg.robot_id] = true;
            robotLastRecvMs[msg.robot_id] = millis();
            xSemaphoreGive(peerMutex);
        }
    }
}

// ---------------- Formation controller ----------------
//
// For each neighbor j:  u += Kp * (|p_j - p_i| - d_ij) * (p_j - p_i) / |p_j - p_i|
// (with a small deadband), averaged over the correcting edges.
// The planar command u is then mapped to (v, w) using an offset point
// LOOKAHEAD_OFFSET_M ahead of the wheel axle.
//
// Returns false (robot stops) if any required neighbor is missing or stale.
static bool calculateGraphFormationControl(
    const RobotSnapshot &self,
    float &outV,
    float &outW,
    uint8_t &connectedNeighbors,
    uint8_t &requiredNeighbors
) {
    float ux = 0.0f;
    float uy = 0.0f;
    uint8_t correctingEdges = 0;

    connectedNeighbors = 0;
    requiredNeighbors = 0;

    for (size_t i = 0; i < NUM_FORMATION_EDGES; i++) {
        int neighborId;
        if (!getNeighborFromEdge(formationEdges[i], ROBOT_ID, neighborId)) {
            continue;
        }

        requiredNeighbors++;

        struct_message peer;
        uint32_t peerAge;
        if (!copyPeerState(neighborId, peer, peerAge)) {
            continue;
        }

        connectedNeighbors++;

        float dx = (peer.x - self.x) / 1000.0f;
        float dy = (peer.y - self.y) / 1000.0f;
        float dist = sqrtf((dx * dx) + (dy * dy));
        if (dist < 0.001f) {
            continue;
        }

        float desired = formationEdges[i].desiredMm / 1000.0f;
        float error = dist - desired;
        float deadband = FORMATION_DISTANCE_DEADBAND_MM / 1000.0f;

        if (fabsf(error) <= deadband) {
            continue;
        }

        float activeError = error;
        if (activeError > 0.0f) {
            activeError -= deadband;
        } else {
            activeError += deadband;
        }

        ux += FORMATION_KP * activeError * (dx / dist);
        uy += FORMATION_KP * activeError * (dy / dist);
        correctingEdges++;
    }

    if (requiredNeighbors == 0 || connectedNeighbors < requiredNeighbors) {
        outV = 0.0f;
        outW = 0.0f;
        return false;
    }

    if (correctingEdges > 1) {
        ux /= correctingEdges;
        uy /= correctingEdges;
    }

    // Offset-point mapping: planar velocity -> unicycle (v, w)
    outV = (cosf(self.theta) * ux + sinf(self.theta) * uy) * 1000.0f;
    outW = FORMATION_YAW_GAIN * (-sinf(self.theta) * ux + cosf(self.theta) * uy) / LOOKAHEAD_OFFSET_M;

    outV = constrain(outV, -MAX_LINEAR_SPEED, MAX_LINEAR_SPEED);
    outW = constrain(outW, -MAX_ANGULAR_SPEED, MAX_ANGULAR_SPEED);

    if (fabsf(outV) < 2.0f) outV = 0.0f;
    if (fabsf(outW) < 0.03f) outW = 0.0f;

    return true;
}

// ---------------- ESP-NOW receive callback ----------------
// Runs in the WiFi task: only copies the packet into a queue.

void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
    if (len != sizeof(struct_message)) return;
    if (recvQueue == NULL) return;

    struct_message temp;
    memcpy(&temp, incomingData, sizeof(temp));
    xQueueSend(recvQueue, &temp, 0);
}

// ---------------- FreeRTOS tasks ----------------

// 100 Hz: encoders + IMU -> odometry snapshot
void sensorTask(void *pvParameters) {
    TickType_t lastWake = xTaskGetTickCount();

    for (;;) {
        vTaskDelayUntil(&lastWake, SENSOR_PERIOD);

        robot.update();

        RobotSnapshot snap;
        snap.x = robot.getX();
        snap.y = robot.getY();
        snap.theta = robot.getTheta();
        snap.speedL = robot.getSpeedL();
        snap.speedR = robot.getSpeedR();
        snap.stampMs = millis();

        if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(1)) == pdTRUE) {
            localState = snap;
            xSemaphoreGive(stateMutex);
        }
    }
}

// 100 Hz: formation law -> ramped (v, w) -> wheel speed PID -> motors
void controlTask(void *pvParameters) {
    TickType_t lastWake = xTaskGetTickCount();
    uint32_t lastMicros = micros();
    float smoothedV = 0.0f;
    float smoothedW = 0.0f;

    for (;;) {
        vTaskDelayUntil(&lastWake, CONTROL_PERIOD);

        uint32_t nowMicros = micros();
        float dt = (float)(nowMicros - lastMicros) / 1000000.0f;
        lastMicros = nowMicros;
        if (dt <= 0.0f || dt > 0.1f) {
            dt = 0.01f;
        }

        if (!ENABLE_MOTORS || millis() < STARTUP_DELAY_MS) {
            leftMotor.stop();
            rightMotor.stop();
            smoothedV = 0.0f;
            smoothedW = 0.0f;
            continue;
        }

        RobotSnapshot self;
        processIncomingPackets();
        bool haveSelf = copyLocalState(self);
        bool freshSensors = haveSelf && ((millis() - self.stampMs) < SENSOR_TIMEOUT_MS);

        uint8_t connectedNeighbors = 0;
        uint8_t requiredNeighbors = 0;
        float targetV = 0.0f;
        float targetW = 0.0f;
        bool graphReady = false;

        if (freshSensors) {
            graphReady = calculateGraphFormationControl(
                self, targetV, targetW, connectedNeighbors, requiredNeighbors);
        }

        // Fail-safe: stop if sensors are stale or any neighbor is missing
        if (!freshSensors || !graphReady) {
            leftMotor.stop();
            rightMotor.stop();
            smoothedV = 0.0f;
            smoothedW = 0.0f;
            continue;
        }

        smoothedV = rampTowards(smoothedV, targetV, MAX_LINEAR_ACCEL * dt);
        smoothedW = rampTowards(smoothedW, targetW, MAX_ANGULAR_ACCEL * dt);

        // Differential-drive inverse kinematics
        float targetL = smoothedV - (WHEEL_BASE_MM * 0.5f) * smoothedW;
        float targetR = smoothedV + (WHEEL_BASE_MM * 0.5f) * smoothedW;
        targetL = constrain(targetL, -MAX_WHEEL_SPEED, MAX_WHEEL_SPEED);
        targetR = constrain(targetR, -MAX_WHEEL_SPEED, MAX_WHEEL_SPEED);

        float outL = leftMotor.computePID(targetL, self.speedL, dt);
        float outR = rightMotor.computePID(targetR, self.speedR, dt);
        leftMotor.drive(outL);
        rightMotor.drive(outR);
    }
}

// 20 Hz: send own position to graph neighbors only
void commTask(void *pvParameters) {
    TickType_t lastWake = xTaskGetTickCount();

    for (;;) {
        vTaskDelayUntil(&lastWake, COMM_PERIOD);
        processIncomingPackets();

        RobotSnapshot self;
        if (!copyLocalState(self, pdMS_TO_TICKS(2))) continue;

        struct_message msg;
        msg.robot_id = ROBOT_ID;
        msg.x = self.x;
        msg.y = self.y;

        for (size_t i = 0; i < NUM_FORMATION_EDGES; i++) {
            int neighborId;
            if (!getNeighborFromEdge(formationEdges[i], ROBOT_ID, neighborId)) {
                continue;
            }
            if (isBlankMac(robotMacs[neighborId])) {
                continue;
            }
            esp_now_send(robotMacs[neighborId], (uint8_t *)&msg, sizeof(msg));
        }
    }
}

// 2 Hz: status over Serial
void debugTask(void *pvParameters) {
    TickType_t lastWake = xTaskGetTickCount();

    for (;;) {
        vTaskDelayUntil(&lastWake, DEBUG_PERIOD);
        processIncomingPackets();

        RobotSnapshot self;
        bool haveSelf = copyLocalState(self, pdMS_TO_TICKS(2));

        if (haveSelf) {
            uint32_t sensorAge = millis() - self.stampMs;
            uint8_t connectedNeighbors = 0;
            uint8_t requiredNeighbors = countRequiredNeighbors();

            for (size_t i = 0; i < NUM_FORMATION_EDGES; i++) {
                int neighborId;
                if (!getNeighborFromEdge(formationEdges[i], ROBOT_ID, neighborId)) {
                    continue;
                }
                struct_message peer;
                uint32_t peerAge;
                if (copyPeerState(neighborId, peer, peerAge)) {
                    connectedNeighbors++;
                }
            }

            Serial.printf(
                "R%d X:%.1f Y:%.1f T:%.1f | Neighbors:%u/%u | SensorAge:%lu | Motors:%s\n",
                ROBOT_ID, self.x, self.y, self.theta * 180.0f / PI,
                connectedNeighbors, requiredNeighbors,
                (unsigned long)sensorAge, ENABLE_MOTORS ? "ON" : "OFF");
        } else {
            Serial.println("No local odometry snapshot");
        }
    }
}

// ---------------- Setup ----------------

static void addEspNowPeers() {
    for (size_t i = 0; i < NUM_FORMATION_EDGES; i++) {
        int neighborId;
        if (!getNeighborFromEdge(formationEdges[i], ROBOT_ID, neighborId)) {
            continue;
        }
        if (isBlankMac(robotMacs[neighborId])) {
            Serial.printf("Robot %d MAC is blank. Fill robotMacs[%d] in config.h.\n", neighborId, neighborId);
            continue;
        }

        esp_now_peer_info_t peerInfo = {};
        memcpy(peerInfo.peer_addr, robotMacs[neighborId], 6);
        peerInfo.channel = 0;
        peerInfo.encrypt = false;

        esp_err_t result = esp_now_add_peer(&peerInfo);
        if (result == ESP_OK || result == ESP_ERR_ESPNOW_EXIST) {
            Serial.printf("Added ESP-NOW neighbor R%d\n", neighborId);
        } else {
            Serial.printf("Failed to add ESP-NOW neighbor R%d, error=%d\n", neighborId, result);
        }
    }
}

void setup() {
    Serial.begin(115200);

    stateMutex = xSemaphoreCreateMutex();
    peerMutex = xSemaphoreCreateMutex();
    recvQueue = xQueueCreate(12, sizeof(struct_message));
    if (stateMutex == NULL || peerMutex == NULL || recvQueue == NULL) {
        Serial.println("RTOS object creation failed");
        return;
    }

    I2C_L.begin(I2C_L_SDA, I2C_L_SCL, 100000);
    I2C_R.begin(I2C_R_SDA, I2C_R_SCL, 100000);
    I2C_L.setTimeOut(10);
    I2C_R.setTimeOut(10);

    mpu6050.begin();
    Serial.println("Calculating gyro offsets. Keep the robot still.");
    mpu6050.calcGyroOffsets(true);
    Serial.println("Gyro calibration done.");

    as5600_L.begin();
    as5600_R.begin();
    robot.begin();
    robot.setPose(initialPoseMm[ROBOT_ID][0], initialPoseMm[ROBOT_ID][1], 0.0f);

    leftMotor.begin(WHEEL_KP, WHEEL_KI, WHEEL_KD);
    rightMotor.begin(WHEEL_KP, WHEEL_KI, WHEEL_KD);
    leftMotor.stop();
    rightMotor.stop();

    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    esp_wifi_set_ps(WIFI_PS_MIN_MODEM);
    Serial.print("ESP-NOW WiFi STA MAC: ");
    Serial.println(WiFi.macAddress());

    if (esp_now_init() != ESP_OK) {
        Serial.println("ESP-NOW init failed");
        return;
    }

    esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecv));
    addEspNowPeers();

    // All tasks on core 1; WiFi stack runs on core 0
    xTaskCreatePinnedToCore(sensorTask,  "Sensors", 8192, NULL, 4, NULL, 1);
    xTaskCreatePinnedToCore(controlTask, "Control", 6144, NULL, 3, NULL, 1);
    xTaskCreatePinnedToCore(commTask,    "Comms",   4096, NULL, 2, NULL, 1);
    xTaskCreatePinnedToCore(debugTask,   "Debug",   4096, NULL, 1, NULL, 1);

    Serial.printf("R%d ready. N=%d, required neighbors=%u, motors=%s\n",
                  ROBOT_ID, NUM_ROBOTS, countRequiredNeighbors(),
                  ENABLE_MOTORS ? "ON" : "OFF");
}

void loop() {
    vTaskDelay(pdMS_TO_TICKS(1000));
}
