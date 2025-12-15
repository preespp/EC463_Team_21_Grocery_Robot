# ESP32 Mecanum Base – Control Modes & Communication Design

This document describes how to structure the ESP32 mecanum-base firmware so that:

- **Motor control + kinematics** are completely independent from
- **Communication method** (local demo, UART, or I2C).

The central idea is that the ESP32 always drives the motors from a single shared velocity command:

```c
typedef struct {
    float vx; // m/s, forward (+X)
    float vy; // m/s, left (+Y)
    float w;  // rad/s, yaw rate (CCW +)
} cmd_vel_t;

static volatile cmd_vel_t g_cmd_vel = {0};
```

If any module can write a valid `g_cmd_vel`, the robot will move accordingly.  
Who writes `g_cmd_vel` (demo, UART, I2C, etc.) is controlled by `g_control_mode`.

---

## 1. Control Modes

We use an enum to define **where** `g_cmd_vel` comes from:

```c
typedef enum {
    CONTROL_MODE_LOCAL_TEST = 0, // Local demo, no Jetson needed
    CONTROL_MODE_I2C       = 1, // Jetson sends cmd_vel via I2C
    CONTROL_MODE_UART      = 2  // Jetson / PC sends commands via UART
} control_mode_t;

static volatile control_mode_t g_control_mode = CONTROL_MODE_LOCAL_TEST;
```

High-level behavior:

- **LOCAL_TEST**: ESP32 generates its own demo trajectory.
- **I2C**: ESP32 receives binary `{vx, vy, w}` from Jetson (I2C master → ESP32 slave).
- **UART**: ESP32 receives textual commands from Jetson or a PC over UART.

The **motor control task never depends on the mode**. It only reads `g_cmd_vel`.

---

## 2. Motor Control & Kinematics (Communication-Independent)

This is the **first block** of logic:

1. Read `cmd_vel = {vx, vy, w}`.
2. Compute 4 wheel angular velocities via mecanum inverse kinematics.
3. Convert wheel rates → direction + PWM.
4. Drive 4 motors through L298N + LEDC.

This block does **not** know or care if the command came from demo / UART / I2C.

### 2.1 Kinematic Parameters

Configure these based on your chassis:

```c
#define HALF_WIDTH    0.1905f   // m  (approx. 7.5")
#define HALF_LENGTH   0.2159f   // m  (approx. 8.5")
#define WHEEL_RADIUS  0.035f    // m  (example, 7 cm diameter wheel)
#define MAX_WHEEL_RAD_PER_SEC  20.0f // example, tune for your motors
```

We define:

```c
typedef struct {
    float w_fl;
    float w_fr;
    float w_rl;
    float w_rr;
} wheel_speeds_t;
```

### 2.2 Mecanum Inverse Kinematics

```c
static void mecanum_inverse_kinematics(const cmd_vel_t *cmd, wheel_speeds_t *ws)
{
    const float R = WHEEL_RADIUS;
    const float L = HALF_LENGTH + HALF_WIDTH;

    const float vx = cmd->vx;
    const float vy = cmd->vy;
    const float w  = cmd->w;

    ws->w_fl = ( vx - vy - L * w ) / R;
    ws->w_fr = ( vx + vy + L * w ) / R;
    ws->w_rl = ( vx + vy - L * w ) / R;
    ws->w_rr = ( vx - vy + L * w ) / R;
}
```

If sideways motion or rotation goes in the opposite direction, you can adjust:

- Sign of `vy` or `w`, or
- Per-wheel inversion flags inside the motor drive functions.

### 2.3 Mapping Wheel Speeds to Motors

```c
static void set_motor_fl(float cmd); // [-1, 1], sign = direction, |value| = speed
static void set_motor_fr(float cmd);
static void set_motor_rl(float cmd);
static void set_motor_rr(float cmd);

static void apply_wheel_commands(const wheel_speeds_t *ws)
{
    const float max_w = MAX_WHEEL_RAD_PER_SEC;

    float w_fl = fminf(fmaxf(ws->w_fl, -max_w), max_w);
    float w_fr = fminf(fmaxf(ws->w_fr, -max_w), max_w);
    float w_rl = fminf(fmaxf(ws->w_rl, -max_w), max_w);
    float w_rr = fminf(fmaxf(ws->w_rr, -max_w), max_w);

    float cmd_fl = w_fl / max_w; // [-1, 1]
    float cmd_fr = w_fr / max_w;
    float cmd_rl = w_rl / max_w;
    float cmd_rr = w_rr / max_w;

    set_motor_fl(cmd_fl);
    set_motor_fr(cmd_fr);
    set_motor_rl(cmd_rl);
    set_motor_rr(cmd_rr);
}
```

In each `set_motor_*` function you:

1. Determine sign (forward/backward).
2. Map `fabs(cmd)` to LEDC PWM duty cycle.
3. Set L298N IN1/IN2 pins for direction.

### 2.4 Motor Control Task

This task runs at a fixed rate and is **completely agnostic** to how `g_cmd_vel` is produced:

```c
static void motor_control_task(void *arg)
{
    const TickType_t period = pdMS_TO_TICKS(20); // 50 Hz

    while (1) {
        cmd_vel_t cmd = g_cmd_vel; // snapshot
        wheel_speeds_t ws;

        mecanum_inverse_kinematics(&cmd, &ws);
        apply_wheel_commands(&ws);

        vTaskDelay(period);
    }
}
```

---

## 3. Local Test Mode (Built-in Demo, No Jetson Needed)

This is the **second block** of logic.

Local test mode lets you bring up and tune the chassis **without any Jetson, I2C, or external PC**.  
The ESP32 simply writes to `g_cmd_vel` in a pattern (forward, stop, strafe, rotate…).

### 3.1 Local Test Task

```c
static void local_test_task(void *arg)
{
    const TickType_t T = pdMS_TO_TICKS(2000); // 2 seconds per motion

    while (1) {
        if (g_control_mode != CONTROL_MODE_LOCAL_TEST) {
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        // 1) Forward 2 s
        g_cmd_vel.vx = 0.2f;
        g_cmd_vel.vy = 0.0f;
        g_cmd_vel.w  = 0.0f;
        vTaskDelay(T);

        // 2) Stop 1 s
        g_cmd_vel.vx = 0.0f;
        g_cmd_vel.vy = 0.0f;
        g_cmd_vel.w  = 0.0f;
        vTaskDelay(pdMS_TO_TICKS(1000));

        // 3) Strafe left 2 s
        g_cmd_vel.vx = 0.0f;
        g_cmd_vel.vy = 0.2f;
        g_cmd_vel.w  = 0.0f;
        vTaskDelay(T);

        // 4) Strafe right 2 s
        g_cmd_vel.vx = 0.0f;
        g_cmd_vel.vy = -0.2f;
        g_cmd_vel.w  = 0.0f;
        vTaskDelay(T);

        // 5) Rotate left 2 s
        g_cmd_vel.vx = 0.0f;
        g_cmd_vel.vy = 0.0f;
        g_cmd_vel.w  = 0.5f;
        vTaskDelay(T);

        // 6) Stop 2 s
        g_cmd_vel.vx = 0.0f;
        g_cmd_vel.vy = 0.0f;
        g_cmd_vel.w  = 0.0f;
        vTaskDelay(T);
    }
}
```

**Usage for bring-up:**

1. Only connect: ESP32 + L298N + motors + battery (USB only for logs, optional).
2. Set `g_control_mode = CONTROL_MODE_LOCAL_TEST`.
3. Flash and run.
4. Observe the robot:
   - Forward/backward correct?
   - Strafe left/right correct?
   - Rotation direction correct?

You tune:

- Wiring, motor inversion,
- `HALF_LENGTH` / `HALF_WIDTH`,
- Sign conventions,

**before** touching Jetson, I2C, or ROS2.

---

## 4. UART Command Mode (Human / PC / Jetson)

UART provides a simple way to manually command the robot from a PC or from Jetson.  
It is also a good debugging tool even if you eventually use I2C for production.

### 4.1 Example UART Command Format

Very simple text-based commands (one line per command):

```text
F0.2\n   → forward  0.2 m/s
B0.2\n   → backward 0.2 m/s
L0.2\n   → left     0.2 m/s
R0.2\n   → right    0.2 m/s
W0.4\n   → rotate   0.4 rad/s
S\n      → stop
```

### 4.2 UART Command Task

```c
static void uart_cmd_task(void *arg)
{
    uint8_t buf[64];

    while (1) {
        int len = uart_read_line(buf, sizeof(buf)); // implement helper

        if (len <= 0) {
            continue;
        }

        if (g_control_mode != CONTROL_MODE_UART) {
            continue;
        }

        char cmd = buf[0];
        float value = 0.0f;

        if (len > 1) {
            value = atof((char *)&buf[1]);
        }

        switch (cmd) {
        case 'F':
            g_cmd_vel.vx = +value;
            g_cmd_vel.vy = 0.0f;
            g_cmd_vel.w  = 0.0f;
            break;
        case 'B':
            g_cmd_vel.vx = -value;
            g_cmd_vel.vy = 0.0f;
            g_cmd_vel.w  = 0.0f;
            break;
        case 'L':
            g_cmd_vel.vx = 0.0f;
            g_cmd_vel.vy = +value;
            g_cmd_vel.w  = 0.0f;
            break;
        case 'R':
            g_cmd_vel.vx = 0.0f;
            g_cmd_vel.vy = -value;
            g_cmd_vel.w  = 0.0f;
            break;
        case 'W':
            g_cmd_vel.vx = 0.0f;
            g_cmd_vel.vy = 0.0f;
            g_cmd_vel.w  = value;
            break;
        case 'S':
        default:
            g_cmd_vel.vx = 0.0f;
            g_cmd_vel.vy = 0.0f;
            g_cmd_vel.w  = 0.0f;
            break;
        }
    }
}
```

**Usage:**

- Set `g_control_mode = CONTROL_MODE_UART`.
- Connect a PC via USB-UART adapter.
- Use `screen /dev/ttyUSBx 115200` or similar.
- Type commands like `F0.2` + Enter and observe.

Jetson can also use the **same UART interface** later (instead of a human).

---

## 5. I2C Command Mode (Optional, Jetson as Master, ESP32 as Slave)

If you later prefer to use I2C between Jetson and ESP32:

- Jetson = I2C master.
- ESP32 = I2C slave.
- Protocol: 12-byte binary struct `{vx, vy, w}`.

### 5.1 Binary Command Format

```c
typedef struct {
    float vx;
    float vy;
    float w;
} __attribute__((packed)) cmd_vel_packet_t;
```

Jetson writes 12 bytes (3 floats, little-endian) to the ESP32 slave address.

### 5.2 I2C Receive Task on ESP32

```c
static void i2c_rx_task(void *arg)
{
    cmd_vel_packet_t local_cmd;
    uint8_t buf[sizeof(cmd_vel_packet_t)];

    while (1) {
        int len = i2c_slave_read_buffer(I2C_PORT,
                                        buf,
                                        sizeof(buf),
                                        portMAX_DELAY);
        if (len == sizeof(buf) &&
            g_control_mode == CONTROL_MODE_I2C) {
            memcpy(&local_cmd, buf, sizeof(local_cmd));
            g_cmd_vel = *(cmd_vel_t *)&local_cmd;
        }
    }
}
```

Again, the motor control task still only uses `g_cmd_vel`.

---

## 6. Task Layout in `app_main()`

A typical setup might look like:

```c
void app_main(void)
{
    // 1. Initialize hardware
    motors_init();  // GPIO + LEDC + L298N
    uart_init();    // For uart_cmd_task (optional but recommended)
    i2c_init();     // Only if you plan to use I2C

    // 2. Start in local test mode for bring-up
    g_control_mode = CONTROL_MODE_LOCAL_TEST;

    // 3. Motor control (always running)
    xTaskCreate(motor_control_task,
                "motor_control_task",
                4096,
                NULL,
                9,
                NULL);

    // 4. Local test demo
    xTaskCreate(local_test_task,
                "local_test_task",
                4096,
                NULL,
                5,
                NULL);

    // 5. UART command task (for human / Jetson over UART)
    xTaskCreate(uart_cmd_task,
                "uart_cmd_task",
                4096,
                NULL,
                6,
                NULL);

    // 6. I2C command task (optional)
    xTaskCreate(i2c_rx_task,
                "i2c_rx_task",
                4096,
                NULL,
                7,
                NULL);

    // 7. (Optional) Add a GPIO-based mode switch:
    //    - button pressed   → CONTROL_MODE_LOCAL_TEST
    //    - button released  → CONTROL_MODE_UART / CONTROL_MODE_I2C
}
```

You can compile-time or run-time select which mode to use.

---

## 7. Recommended Development Phases

### Phase 1 – Chassis + Demo Mode (No Jetson, No I2C)

- Implement:
  - `mecanum_inverse_kinematics`
  - `apply_wheel_commands` (LEDC + L298N)
  - `motor_control_task`
  - `local_test_task`
- Set `g_control_mode = CONTROL_MODE_LOCAL_TEST` (default in current firmware).
- **Usage**
  - Flash the ESP32 with batteries/motors connected.
  - The `local_test_task` cycles through the scripted path (forward, strafe, rotate) without any external commands.
  - Use this mode to verify wheel orientation, PWM scaling, and battery health indoors before connecting Jetson.
- Test and tune:
  - Forward/backward correctness
  - Sideways motion correctness
  - Rotation direction

This phase uses only **ESP32 + L298N + motors (+ battery)**.

---

### Phase 2 – UART Control (Still Jetson-Independent If You Want)

- Enable `uart_cmd_task`.
- Set `g_control_mode = CONTROL_MODE_UART`.
- **Usage**
  - Connect ESP32 UART1 TX/RX to a USB–UART adapter (or Jetson’s UART) at 115200 8N1.
  - From a terminal (e.g., `screen`, `minicom`, `PuTTY`), type single-letter commands followed by a value, then press Enter:
    - `F0.2` → Forward 0.2 m/s
    - `B0.2` → Reverse 0.2 m/s
    - `L0.2` / `R0.2` → Strafe left/right
    - `W0.3` → Rotate CCW at 0.3 rad/s
    - `S` → Stop
  - Any computer capable of opening the serial port can drive the chassis; once tested, Jetson can send the same strings programmatically.
- Confirm that external commands update `g_cmd_vel` and control the chassis.

At this point you have:

- **Local demo** and
- **Manual remote control via UART**.

---

### Phase 3 – Connect Jetson (Preferably via UART First)

Simplest path:

- On Jetson, write a small program or ROS2 node:
  - Subscribe to `/cmd_vel`.
  - Convert to UART commands (`F`, `L`, `R`, `W`, or JSON).
  - Send over UART to ESP32.

ESP32 stays in `CONTROL_MODE_UART`, only the **source** of commands changes (PC → Jetson).

---

### Phase 4 – (Optional) Switch to I2C Command Mode

If you really want I2C:

- Jetson I2C master writes `{vx, vy, w}` struct to ESP32 slave.
- ESP32 `i2c_rx_task` receives packets and, in `CONTROL_MODE_I2C`, updates `g_cmd_vel`.
- Motor control and kinematics remain unchanged.
- **Usage**
  - Hardware: connect ESP32 SDA/SCL (3.3 V) to Jetson Orin Nano J12 pins 3/5, which are on **I2C Bus 7**. Add 2.2 k–4.7 k pull-ups to 3.3 V if the line does not already have them.
  - ESP32 firmware: set `g_control_mode = CONTROL_MODE_I2C`. Our ESP32 I2C slave uses address **0x46** to avoid conflicting with Jetson’s onboard devices (JetsonHacks pinout shows bus-1 devices at 0x25/0x40).
  - Jetson software: open `/dev/i2c-7`, write 12 bytes (little-endian floats `{vx, vy, w}`) at ~20–50 Hz. Example pseudo-code in Python:
    ```python
    import struct, fcntl, os
    I2C_SLAVE = 0x0703
    fd = os.open("/dev/i2c-7", os.O_RDWR)
    fcntl.ioctl(fd, I2C_SLAVE, 0x46)
    payload = struct.pack("<fff", 0.2, 0.0, 0.1)
    os.write(fd, payload)
    ```
  - Validate the bus using `sudo i2cdetect -y -r 7` on Jetson; `0x46` should appear when the ESP32 is powered.

---

## 8. Key Takeaways

- **Motor control & kinematics are one module; communication is another.**
- `g_cmd_vel` is the only interface between them.
- `g_control_mode` selects which communication source is currently active:
  - Internal demo,
  - UART,
  - or I2C.
- You can fully bring up and debug the chassis **without Jetson**, then progressively add UART and I2C as needed.

This structure should make your ESP32 firmware easier to test, extend, and integrate with the Jetson + ROS2 navigation stack.
