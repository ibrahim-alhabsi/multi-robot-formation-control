#pragma once
#include <Arduino.h>

// ============================================================
//  Per-robot settings: change ROBOT_ID before flashing each robot
// ============================================================
#define ROBOT_ID      1      // 1 .. NUM_ROBOTS
#define NUM_ROBOTS    3      // robots taking part in this run
#define ENABLE_MOTORS true   // false = sensors and comms only (bench testing)

static const int MAX_ROBOTS = 5;

// ------------------------------------------------------------
//  WiFi STA MAC address of each robot (index 0 unused).
//  Each robot prints its own MAC on the Serial Monitor at boot.
// ------------------------------------------------------------
static uint8_t robotMacs[MAX_ROBOTS + 1][6] = {
    {0, 0, 0, 0, 0, 0},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00},  // Robot 1
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00},  // Robot 2
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00},  // Robot 3
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00},  // Robot 4
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00},  // Robot 5
};

// ------------------------------------------------------------
//  Starting pose of each robot in the shared frame [mm].
//  All robots start facing +x (theta = 0).
// ------------------------------------------------------------
static const float initialPoseMm[MAX_ROBOTS + 1][2] = {
    {0.0f, 0.0f},
    {-600.0f,    0.0f},  // Robot 1
    { 600.0f,    0.0f},  // Robot 2
    {   0.0f, -600.0f},  // Robot 3
    {   0.0f, -400.0f},  // Robot 4
    {   0.0f,    0.0f},  // Robot 5
};

// ------------------------------------------------------------
//  Formation graph: undirected edges with desired distances [mm].
//  Minimally rigid in 2D: 2N - 3 edges.
//    N = 3 -> first 3 edges (triangle)
//    N = 4 -> first 5 edges
//    N = 5 -> all 7 edges
//  Edges touching robots above NUM_ROBOTS are ignored automatically.
// ------------------------------------------------------------
struct FormationEdge {
    uint8_t a;
    uint8_t b;
    float desiredMm;
};

static const FormationEdge formationEdges[] = {
    {1, 2, 400.0f},
    {2, 3, 300.0f},
    {1, 3, 500.0f},
    {1, 4, 200.0f},
    {3, 4, 282.84f},
    {2, 5, 250.0f},
    {3, 5, 250.0f},
};

static const size_t NUM_FORMATION_EDGES = sizeof(formationEdges) / sizeof(formationEdges[0]);

// ============================================================
//  Robot geometry and controller tuning
// ============================================================
static const float WHEEL_BASE_MM      = 70.0f;
static const float WHEEL_DIAMETER_MM  = 34.0f;

static const float FORMATION_KP                   = 0.75f;
static const float FORMATION_YAW_GAIN             = 1.2f;
static const float LOOKAHEAD_OFFSET_M             = 0.05f;
static const float FORMATION_DISTANCE_DEADBAND_MM = 4.0f;

static const float WHEEL_KP = 9.25f;
static const float WHEEL_KI = 1.0f;
static const float WHEEL_KD = 0.33f;

static const float MAX_LINEAR_ACCEL  = 180.0f;  // mm/s^2
static const float MAX_ANGULAR_ACCEL = 2.5f;    // rad/s^2
static const float MAX_LINEAR_SPEED  = 110.0f;  // mm/s
static const float MAX_ANGULAR_SPEED = 1.4f;    // rad/s
static const float MAX_WHEEL_SPEED   = 120.0f;  // mm/s

// ============================================================
//  Timing
// ============================================================
static const uint32_t STARTUP_DELAY_MS  = 4000;
static const uint32_t COMM_TIMEOUT_MS   = 1000;
static const uint32_t SENSOR_TIMEOUT_MS = 150;
static const TickType_t SENSOR_PERIOD  = pdMS_TO_TICKS(10);   // 100 Hz
static const TickType_t CONTROL_PERIOD = pdMS_TO_TICKS(10);   // 100 Hz
static const TickType_t COMM_PERIOD    = pdMS_TO_TICKS(50);   // 20 Hz
static const TickType_t DEBUG_PERIOD   = pdMS_TO_TICKS(500);  // 2 Hz

// ============================================================
//  Pin map (ESP32 LOLIN D32)
// ============================================================
// I2C bus L: left AS5600 + MPU6050   |  I2C bus R: right AS5600
// (both AS5600s share address 0x36, so each needs its own bus)
static const int I2C_L_SDA = 21, I2C_L_SCL = 22;
static const int I2C_R_SDA = 18, I2C_R_SCL = 19;

// DRV8833 inputs: {IN1, IN2, PWM channel 1, PWM channel 2}
static const int MOTOR_L_IN1 = 26, MOTOR_L_IN2 = 25;
static const int MOTOR_R_IN1 = 32, MOTOR_R_IN2 = 27;
