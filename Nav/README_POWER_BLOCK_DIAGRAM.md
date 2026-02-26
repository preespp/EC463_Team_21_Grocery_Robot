# Grocery Robot Power Block Diagram (Current Wiring)

This README documents the current power architecture for the Team 21 Grocery Robot based on:

- `Nav/README_TEST_PLAN_SETUP.md`
- `README.md`
- `STM32/Base_Control/INTERFACE_SUMMARY.md`
- Current team wiring notes (Feb 2026)

## 1. Hardware in Scope

Base system:

- NVIDIA Jetson Orin Nano Super Developer Kit (Ubuntu 22.04)
- DJI Development Board A (STM32F427IIH6 base controller)
- GL.iNet AXT-1800 WiFi 6 router
- SICK PicoScan150 LiDAR
- 4x DJI M3508 motors
- 4x DJI C620 motor controllers (CAN bus to STM32 board)
- ESP32 (ultrasonic sensor controller)
- 10 inch touchscreen (GUI)
- USB-to-UART bridge (Jetson <-> DJI board)

Arm system:

- Arm ESP32 base controller
- Servo set
- 12V XT30 output from DJI STM32 board (rated 10A)

Power hardware:

- 4x 25.6V LiFePO4 batteries total, arranged as two battery sets (2 batteries per set)
- 2x voltage balancer/discharger modules (one per battery set)
- OrangeRC XT60 to USB-PD power converter
- USB-C PD to DC barrel converter (for Jetson input)
- Remote E-stop switch (30A pass-through)
- DJI XT60/XT30 center power distribution board

## 2. High-Level Power Topology

Two battery branches are used:

- Branch A (compute/network): powers Jetson and router through PD conversion.
- Branch B (drive/actuation): powers chassis drive stack through E-stop and DJI power distribution.

```mermaid
flowchart LR
  subgraph A[Branch A - Compute and Network]
    BA[Battery Set A\n2x 25.6V LiFePO4] --> BALA[Balancer/Discharger A]
    BALA --> ORC[OrangeRC XT60 to USB-PD Converter]
    ORC --> USBA[USB-A Output]
    ORC --> USBC[USB-C PD Output]
    USBA --> RTR[GL.iNet AXT-1800 Router]
    USBC --> PDDC[PD to DC Barrel Converter]
    PDDC --> JET[NVIDIA Jetson Orin Nano]
  end

  subgraph B[Branch B - Drive and Actuation]
    BB[Battery Set B\n2x 25.6V LiFePO4] --> BALB[Balancer/Discharger B]
    BALB --> ESTOP[Remote E-stop\n30A pass-through]
    ESTOP --> BUS[Center Power Bus]
    BUS --> DJI[DJI XT60/XT30 Power Distribution Board]
    DJI --> STM[DJI Dev Board A (STM32F427)]
    DJI --> C620[4x C620 Motor Controllers]
    C620 --> M3508[4x M3508 Motors]
    STM --> XT30_12V[Onboard XT30 12V 10A Output]
    XT30_12V --> ARM[Arm ESP32 + Servos]
  end
```

## 3. Detailed Path Description

### 3.1 Branch A: Jetson and Router Power

1. Two 25.6V batteries feed Balancer/Discharger A.
2. Balancer output feeds OrangeRC XT60 to USB-PD converter.
3. Converter USB-A output powers the GL.iNet AXT-1800 router.
4. Converter USB-C PD output feeds a PD-to-DC barrel converter.
5. DC barrel output powers the Jetson Orin Nano input jack.

### 3.2 Branch B: Drive Power and E-stop Path

1. The second pair of 25.6V batteries feeds Balancer/Discharger B.
2. Balancer output passes through remote E-stop switch (30A pass-through rating).
3. E-stop output feeds center power bus, then DJI power distribution board.
4. DJI power distribution supplies:
   - DJI Dev Board A (STM32 base controller)
   - 4x C620 motor controllers
5. C620 controllers drive 4x M3508 motors over the motor power path.

### 3.3 Arm Power Path

1. DJI STM32 board provides onboard XT30 12V output (10A rated).
2. This 12V rail powers arm electronics (arm ESP32 + servos).

## 4. Current and Safety Notes

- Remote E-stop is placed on Branch B (drive/actuation branch), so it is expected to cut drive power while compute branch can remain alive.
- Reported battery capability note: each battery can provide about 10A continuous, up to about 18A short burst (<3 s).
- Keep E-stop physically reachable and verify wheel free-space before power-on.
- Confirm polarity and connector orientation before every test session.

## 5. Interfaces Related to This Power Layout

- Jetson <-> STM32 command/telemetry: USB-to-UART (`/dev/ttyUSB0`, 115200 bps in runbooks)
- STM32 <-> C620: CAN1 bus at 1 Mbps (see `STM32/Base_Control/INTERFACE_SUMMARY.md`)
- LiDAR <-> Jetson: Ethernet UDP (network path, not a power path)

## 6. Open Items to Verify on Hardware

- Exact power feed for SICK PicoScan150 and touchscreen is not explicitly documented in repo docs.
- Confirm whether ultrasonic ESP32 is fed from DJI 12V rail (with local regulation) or from a separate regulator path.
- Add inline fuse ratings per branch if available (recommended for final electrical documentation).

## 7. Suggested Power-On Sequence

1. Verify E-stop is engaged and robot wheels are clear.
2. Power Branch A (Jetson + router), verify Jetson boot and network access.
3. Power Branch B, then release E-stop only when software stack is ready.
4. Start base bridge and teleop/nav stack.
5. Re-engage E-stop before shutdown/disconnect.

