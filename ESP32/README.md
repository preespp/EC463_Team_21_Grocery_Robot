# ESP32 Projects

This folder contains various ESP32-based projects and components for the Grocery Robot system.

## Folder Structure

- **arm_servo_I2C/**: ESP32 project for controlling arm servos via I2C interface. Includes Python script and ESP-IDF build files.
- **linear_actuator/**: ESP32 project for linear actuator control using ESP-IDF.
- **servo-component-testing/**: Arduino sketches for testing servo components, including lift tests and repeatability tests.
- **ultrasonic/**: Arduino sketch for ultrasonic sensor functionality.
- **ultrasonic_I2C/**: ESP32 project for ultrasonic sensors with I2C communication using ESP-IDF.

## Getting Started

Each subfolder contains its own build configuration and source code. Refer to the individual README files or project files for setup instructions.

For ESP-IDF projects, ensure you have the ESP-IDF environment set up. For Arduino projects, use the Arduino IDE with ESP32 board support.

## Dependencies

- ESP-IDF (for ESP-IDF projects)
- Arduino IDE with ESP32 core (for .ino files)
- Required hardware: ESP32 microcontroller, servos, ultrasonic sensors, etc.