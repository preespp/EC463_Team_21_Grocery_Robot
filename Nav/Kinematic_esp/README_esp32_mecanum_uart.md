# ESP32 Mecanum Base Controller (Jetson UART)

This document specifies the architecture, kinematics, and UART protocol for an ESP32-based 4-wheel mecanum chassis using two L298N motor drivers, controlled by a Jetson board over UART.

Target use case:

- Jetson runs ROS2 / SLAM / Nav2 and sends velocity commands.
- ESP32 handles low-level motor control (PWM + direction) for 4 mecanum wheels.
- ESP32 communicates with Jetson through a single UART pair (ESP32 TX/RX ↔ Jetson UART pins).

---

## 1. Hardware Overview

### 1.1 Chassis

- Chassis outer size: **20" × 20" × 1"** (square aluminum plate).
- Coordinate convention (top view):
  - X axis: forward (from center toward the *top* edge of the plate).
  - Y axis: left (from center toward the *left* edge of the plate).
  - Origin `(0, 0)` at chassis geometric center.

Motor mounting (assumption for all 4 wheels, symmetric):

- From the **left/right edge** of the plate to the motor center: **2.5"** (horizontal offset).
- From the **front/back edge** to the motor center: **1.5"** (vertical offset).
- Plate size: 20" per side → plate center at (10", 10") in plate coordinates.

Thus, wheel centers (in plate coordinates, origin at top-left corner):

- Front Left (FL):  (2.5", 1.5")
- Front Right (FR): (20" - 2.5", 1.5") = (17.5", 1.5")
- Rear Left (RL):   (2.5", 20" - 1.5") = (2.5", 18.5")
- Rear Right (RR):  (17.5", 18.5")

Relative to chassis center (10", 10"):

- FL:  x =  2.5"  →  10 - 2.5 =  **7.5"** left of center  
       y =  1.5"  →  10 - 1.5 =  **8.5"** forward of center  
- FR:  x = 17.5"  → 17.5 - 10 = **7.5"** right of center  
       y =  1.5"  →  8.5" forward of center  
- RL:  x =  2.5"  →  7.5" left of center  
       y = 18.5"  → 18.5 - 10 = **8.5"** backward of center  
- RR:  x = 17.5"  →  7.5" right of center  
       y = 18.5"  →  8.5" backward of center  

We define:

- `HALF_WIDTH`  = lateral distance from center to left/right motors ≈ **7.5"**  
- `HALF_LENGTH` = longitudinal distance from center to front/rear motors ≈ **8.5"**  

Converted to meters (1" = 2.54 cm):

- `HALF_WIDTH`  = 7.5"  ≈ 0.1905 m  
- `HALF_LENGTH` = 8.5"  ≈ 0.2159 m  

In code:

```c
#define HALF_WIDTH   0.1905f   // m
#define HALF_LENGTH  0.2159f   // m
#define L            (HALF_WIDTH + HALF_LENGTH) // effective lever arm for rotation
```

Wheel radius `R` (m) should be measured based on your actual mecanum wheels:

```c
#define WHEEL_RADIUS  0.035f   // example: 7 cm diameter → 3.5 cm radius
```

> If the real wheel radius is different, update `WHEEL_RADIUS` accordingly.

---

### 1.2 Mecanum Wheels & Motor Drivers

- 4 × mecanum wheels (FL, FR, RL, RR).
- 4 × DC motors with encoders (optional but recommended).
- 2 × L298N driver boards:
  - Each L298N drives 2 motors.
  - Each motor uses:
    - 1 × PWM pin (ENA/ENB)
    - 2 × direction pins (IN1/IN2)

Example mapping (to be adjusted to the actual wiring):

```c
// Front Left (FL)
#define FL_PWM_GPIO    25
#define FL_IN1_GPIO    26
#define FL_IN2_GPIO    27

// Front Right (FR)
#define FR_PWM_GPIO    14
#define FR_IN1_GPIO    12
#define FR_IN2_GPIO    13

// Rear Left (RL)
#define RL_PWM_GPIO    33
#define RL_IN1_GPIO    32
#define RL_IN2_GPIO    23

// Rear Right (RR)
#define RR_PWM_GPIO    19
#define RR_IN1_GPIO    18
#define RR_IN2_GPIO    5
```

ESP32 uses **LEDC** for PWM:

- One LEDC timer for all 4 motors (e.g., 15–20 kHz).
- One LEDC channel per motor (`FL_PWM`, `FR_PWM`, `RL_PWM`, `RR_PWM`).
- PWM duty controls speed, IN1/IN2 define direction.

---

### 1.3 ESP32 ↔ Jetson UART

- ESP32 UART: `UART_NUM_1` (for example).
- Jetson UART: one UART pair on the 40-pin header.
- Wiring:
  - ESP32 TX → Jetson RX
  - ESP32 RX → Jetson TX
  - GND shared

UART parameters:

- Baud: 115200
- Data bits: 8
- Parity: None
- Stop bits: 1

ESP-IDF side:

- `uart_param_config()`
- `uart_set_pin()`
- `uart_driver_install()`
- Two FreeRTOS tasks:
  - `uart_rx_task`: receive command from Jetson, update `cmd_vel`.
  - (Optional) `uart_tx_task`: send odometry / status back to Jetson.

---

## 2. Coordinate Frames & Conventions

Robot body frame (used by kinematics and command velocities):

- Origin: chassis center.
- **+X**: forward (toward “top” edge).
- **+Y**: left.
- **+Z**: up.
- Angular velocity `w` is yaw rate around +Z (CCW positive when viewed from top).

Command velocity from Jetson:

- `vx` [m/s] – forward speed (+X)
- `vy` [m/s] – leftward speed (+Y)
- `w`  [rad/s] – yaw rate (CCW positive)

---

## 3. Mecanum Kinematics

We use standard mecanum inverse kinematics with wheel centers offset by `HALF_LENGTH` and `HALF_WIDTH`.

Let:

- `vx` – robot forward velocity (m/s, +X)
- `vy` – robot leftward velocity (m/s, +Y)
- `w`  – robot yaw rate (rad/s, CCW)
- `R`  – wheel radius (m)
- `L`  – effective lever arm for rotation:

  ```c
  float L = HALF_LENGTH + HALF_WIDTH; // m
  ```

Wheel angular velocities (`w_fl`, `w_fr`, `w_rl`, `w_rr`) in rad/s:

```c
w_fl = (1.0f / R) * ( vx - vy - L * w );
w_fr = (1.0f / R) * ( vx + vy + L * w );
w_rl = (1.0f / R) * ( vx + vy - L * w );
w_rr = (1.0f / R) * ( vx - vy + L * w );
```

Notes:

- These formulas assume a specific roller orientation (FL/RR rollers aligned one way, FR/RL the opposite).  
- If your chassis moves in the opposite direction to what you expect, you may need to:
  - flip sign of `vy` or `w`, or  
  - define `MOTOR_FL_INVERT`, etc., to invert each wheel.

After computing wheel angular velocities, we map to normalized motor commands in `[-1, 1]` using a max wheel speed:

```c
float max_w = MAX_WHEEL_RAD_PER_SEC; // tuned for your motors
float cmd_fl = w_fl / max_w;         // [-1, 1]
...
```

Then:

```c
bool dir_fl_forward = (cmd_fl >= 0.0f);
float duty_fl = fabsf(cmd_fl); // [0, 1]
```

And we set:

- DIR pins according to `dir_fl_forward`.
- PWM duty via LEDC according to `duty_fl`.

---

## 4. ESP32 Firmware Structure (ESP-IDF)

### 4.1 Data Structures

```c
typedef struct {
    float vx; // m/s
    float vy; // m/s
    float w;  // rad/s
} cmd_vel_t;

typedef struct {
    float w_fl;
    float w_fr;
    float w_rl;
    float w_rr;
} wheel_speeds_t;

// Shared command velocity (updated by UART RX task, read by motor control task)
static cmd_vel_t g_cmd_vel = {0};
```

Optional (later):

- encoder tick counters per wheel
- odometry struct with (x, y, theta)

### 4.2 Initialization

In `app_main()`:

1. Initialize GPIO for direction pins.
2. Initialize LEDC timer and channels for PWM.
3. Initialize UART (`uart_init()`).
4. Create FreeRTOS tasks:
   - `uart_rx_task()`
   - `motor_control_task()`
   - (optional) `uart_tx_task()`
   - (optional) encoder / odometry tasks

### 4.3 `motor_control_task()`

- Runs at fixed rate (e.g. 50–100 Hz).
- Steps:
  1. Copy `g_cmd_vel` into a local variable.
  2. Call `mecanum_inverse_kinematics()` to get wheel speeds.
  3. Clip each wheel speed to ±`MAX_WHEEL_RAD_PER_SEC`.
  4. Convert to motor direction + PWM duty.
  5. Apply via LEDC and GPIO.

Pseudocode:

```c
void motor_control_task(void *arg) {
    const TickType_t period = pdMS_TO_TICKS(20); // 50 Hz
    while (1) {
        cmd_vel_t cmd = g_cmd_vel;
        wheel_speeds_t wheels;
        mecanum_inverse_kinematics(&cmd, &wheels);
        apply_wheel_commands(&wheels);
        vTaskDelay(period);
    }
}
```

---

## 5. UART Protocol (ESP32 ↔ Jetson)

### 5.1 Physical

- Single UART link:
  - ESP32 UART1 TX/RX ↔ Jetson UART pins.
- Baud: 115200 8N1.

### 5.2 Command From Jetson → ESP32

Simple line-based JSON (one command per line, `\n` terminated):

```json
{"vx": 0.20, "vy": 0.00, "w": 0.10}
```

Units:

- `vx` [m/s]
- `vy` [m/s]
- `w`  [rad/s]

Jetson should send at 20–50 Hz (matching or higher than motor control loop).

ESP32 `uart_rx_task` behavior:

1. Read bytes from UART until `"\n"`.
2. Parse JSON (or a simple custom parser).
3. Update `g_cmd_vel`.

### 5.3 Status From ESP32 → Jetson (Optional)

Optional feedback for debugging / odometry:

- Example payload:

```json
{
  "t": 12345678,
  "vx_est": 0.19,
  "vy_est": 0.01,
  "w_est": 0.09,
  "fl_rps": 3.2,
  "fr_rps": 3.1,
  "rl_rps": 3.1,
  "rr_rps": 3.2
}
```

- `t`: ESP32 uptime in ms.
- `*_est`: estimated robot velocities.
- `*_rps`: wheel speeds (rev/s or rad/s).

This can be sent at a lower rate (e.g. 10–20 Hz) to avoid flooding the UART.

---

## 6. Jetson Side Expectations

Jetson is responsible for:

1. Opening the UART device (e.g. `/dev/ttyTHS1`) at 115200 8N1.
2. In a ROS2 node:
   - Subscribe to `/cmd_vel` (from Nav2).
   - Convert messages to JSON lines and send to ESP32 over UART.
3. (Optional) Read status lines from ESP32 and publish:
   - `/wheel_states`
   - `/odom_raw`
   - any additional debug topics.

The high-level ROS2 stack (SLAM + Nav2) only needs to know:

- It can send `(vx, vy, w)` via `/cmd_vel`, and
- There is an odometry source that publishes robot motion.

---

## 7. TODO / Calibration

To complete the system, you still need to:

- Measure or confirm:
  - **Wheel radius `R`** (m)
  - **Maximum wheel angular speed** `MAX_WHEEL_RAD_PER_SEC`
- Check real chassis:
  - If motors are not perfectly symmetric, adjust `HALF_LENGTH`, `HALF_WIDTH`.
- Verify wheel orientation:
  - If the robot moves sideways in the opposite direction, flip sign of `vy` or adjust wheel inversion flags.

Once the above are set, the ESP32 firmware can:

1. Take Jetson’s `(vx, vy, w)` commands over UART,
2. Run mecanum inverse kinematics based on the **20"×20" chassis and the given wheel offsets**,
3. Drive the 4 motors via L298N using PWM (LEDC) + direction GPIO,
4. Optionally provide encoder/odometry feedback back to Jetson.
