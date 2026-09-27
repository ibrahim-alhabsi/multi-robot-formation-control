# Firmware

Every robot runs the same sketch. The only thing that changes between robots is `ROBOT_ID` in `config.h`.

## What you need

- **Board:** ESP32 (tested on LOLIN D32)
- **Arduino-ESP32 core 2.0.x.** Core 3.x removed `ledcSetup()` / `ledcAttachPin()`, so this code won't compile on it without changes.
- **Libraries** (Arduino Library Manager):
  - `AS5600` by Rob Tillaart
  - `MPU6050_tockn` by tockn
  - `Kalman Filter Library` by TKJ Electronics

## How to run it

1. Flash any sketch that prints `WiFi.macAddress()` on each robot (or just flash this one: it prints the MAC at boot) and write down every robot's MAC.
2. Fill in `robotMacs`, `initialPoseMm`, and `formationEdges` in `config.h`. The same file goes on every robot.
3. Set `ROBOT_ID`, then flash. Repeat for each robot.
4. Place each robot at its starting position, all facing +x.
5. Switch them on and **don't touch them for ~4 seconds** while the gyro calibrates.
6. They start moving once every robot hears from all of its graph neighbors.

Tip: set `ENABLE_MOTORS false` to test sensors and communication on the bench. The Serial Monitor (115200 baud) shows each robot's pose and how many neighbors it can hear.

## How the code is organized

| File | What it does |
|---|---|
| `formation_robot.ino` | FreeRTOS tasks, ESP-NOW, and the formation controller |
| `config.h` | Robot ID, MACs, graph edges, gains, pins |
| `Odometry.*` | Encoder odometry with Kalman-fused heading |
| `MotorControl.*` | Wheel speed PID and DRV8833 PWM output |

| Task | Rate | Job |
|---|---|---|
| Sensors | 100 Hz | Read encoders + IMU, update pose |
| Control | 100 Hz | Formation law → (v, ω) → wheel PIDs |
| Comms | 20 Hz | Send own position to graph neighbors |
| Debug | 2 Hz | Print status |
