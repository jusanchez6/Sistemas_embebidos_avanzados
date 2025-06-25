| Supported Targets | ESP32-S3 |
| ----------------- | -------- |

# ROBOT-MOV: FreeRTOS-Based Mobile Robot Control System

This project implements a real-time motion control system for a mobile robot using FreeRTOS on the ESP32-S3 platform. It integrates multiple sensors and actuators (BLDC motors, magnetic encoders, ToF distance sensor, and IMU) for high-accuracy odometry and velocity control using PID controllers and sensor fusion techniques.

---

## 📁 Project Structure
```
ROBOT-MOV/
├── drivers/
│   ├── bldc_pwm/
│   │   ├── include/
│   │   └── bldc_pwm.c
│   ├── pid_lib/
│   │   ├── include/
│   │   ├── pid_ext.h
│   │   └── pid_ext.c
│   └── sensor_fusion/
│       ├── include/
│       └── sensor_fusion.c
├── main/
│   ├── include/
│   ├── CMakeLists.txt
│   ├── control_main.c
│   ├── encoder_calibration.c
│   └── main.c
├── platform/
│   ├── include/
│   ├── platform_esp32s3.h
│   └── platform_esp32s3.c
├── sensors/
│   ├── AS5600_HAL_Library/
│   │   ├── include/
│   │   └── as5600_lib.c
│   ├── TM151-driver/
│   │   ├── include/
│   │   ├── EasyObjectDictionary.c
│   │   ├── EasyProfile.c
│   │   ├── EasyProtocol.c
│   │   ├── EasyQueue.c
│   │   └── EasyRetrieve.c
│   └── VL53L1X-driver/
│       ├── VL53L1X.c
├── CMakeLists.txt
├── README.md
└── sdkconfig
```

---

## 🚀 `main.c` – System Initialization and Task Creation

The `main.c` file initializes all hardware components and creates the FreeRTOS tasks required for robot operation.

### Sensors Initialized:
- **VL53L1X** Time-of-Flight Lidar for obstacle detection.
- **AS5600** magnetic encoders for each wheel (Right, Left, Back).
- **TM151 IMU** via UART for inertial measurements.

### Actuators:
- **BLDC Motors (Right, Left, Back):** Controlled via PWM.
- **PID Controllers:** Initialized with external tuning parameters for each wheel.

### FreeRTOS Tasks Created:
| Task Name            | Function        | Purpose                                |
|---------------------|-----------------|----------------------------------------|
| `right_encoder_task`| `vTaskEncoder`  | Read encoder data from AS5600          |
| `left_encoder_task` | `vTaskEncoder`  | Read encoder data                      |
| `back_encoder_task` | `vTaskEncoder`  | Read encoder data                      |
| `imu_task`          | `vTaskIMU`      | Acquire IMU data from TM151            |
| `lidar_task`        | `vTaskLidar`    | Measure distances using VL53L1X        |
| `rwh_control_task`  | `vTaskControl`  | Closed-loop control for right wheel    |
| `lwh_control_task`  | `vTaskControl`  | Closed-loop control for left wheel     |
| `bwh_control_task`  | `vTaskControl`  | Closed-loop control for back wheel     |

### Extern Variables from `control_main.h`:
- Encoder data: `right_encoder_data`, `left_encoder_data`, `back_encoder_data`
- IMU data: `imu_data`
- Lidar data: `lidar_data`
- PID parameters: `pid_paramR`, `pid_paramL`, `pid_paramB`

---

## 🧠 `sensor_fusion.c` – Velocity & Position Estimation

This module implements lightweight sensor fusion algorithms for real-time state estimation.

### Functions Overview:

#### `estimate_velocity_imu(...)`
- Estimates velocity by trapezoidal integration of IMU acceleration.
- Includes exponential smoothing for noise rejection.

#### `estimate_velocity_encoder(...)`
- Computes velocity from AS5600 encoder angle deltas and wheel radius.
- Applies a low-pass filter and ignores micro noise below threshold.

#### `estimate_velocity_lidar(...)`
- Calculates velocity from distance differences measured by the ToF sensor.
- Filters out noisy variations and updates velocity using moving average.

#### `estimate_position(...)`
- Returns the average position between lidar and encoder estimates.

#### `estimate_velocity(...)`
- Fuses IMU, encoder, and lidar velocities:
  - Chooses the pair with the least deviation.
  - Averages the chosen two for robust estimation.

---

## ⚙️ Control Strategy from `pid_ext.h`

Each wheel is controlled through a PID feedback loop based on encoder readings. Velocity commands are generated, and sensor fusion improves accuracy in noisy environments. Tasks are carefully prioritized and assigned appropriate stack sizes for stability and responsiveness.

---

## 📦 Dependencies

- **ESP-IDF**
- **FreeRTOS**
- **VL53L1X driver**
- **TM151 IMU UART driver**
- **AS5600 magnetic encoder library**