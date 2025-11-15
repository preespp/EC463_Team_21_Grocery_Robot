// #include <SPI.h>
// #include <ESP32Servo.h>

// volatile byte angle = 90;
// Servo servo;

// void onReceive(byte data) {
//   angle = data;
//   servo.write(angle);
// }

// void setup() {
//   SPI.begin(18, 19, 23, 5);  // SCLK, MISO, MOSI, SS
//   pinMode(5, INPUT);

//   servo.attach(18);
//   servo.write(90);

//   SPI.onReceive(onReceive);
// }

// void loop() {}

// #include <Arduino.h>
// #include <ESP32Servo.h>
// #include "driver/spi_slave.h"

// #define PIN_MISO 19
// #define PIN_MOSI 23
// #define PIN_SCLK 18
// #define PIN_SS   5

// Servo servo;

// spi_slave_transaction_t t;
// uint8_t rx_buffer[1];

// void setup() {
//   Serial.begin(115200);

//   // --- Servo setup ---
//   servo.attach(15);   // Use a safe PWM-capable pin
//   servo.write(90);

//   // --- SPI Slave Config ---
//   spi_bus_config_t buscfg = {
//         .mosi_io_num = PIN_MOSI,
//         .miso_io_num = PIN_MISO,
//         .sclk_io_num = PIN_SCLK,
//         .quadwp_io_num = -1,
//         .quadhd_io_num = -1,
//         .max_transfer_sz = 1
//   };

//   spi_slave_interface_config_t slvcfg = {
//         .spics_io_num = PIN_SS,
//         .flags = 0,
//         .queue_size = 3,
//         .mode = 0,
//         .post_setup_cb = NULL,
//         .post_trans_cb = NULL
//   };

//   // Initialize SPI slave
//   spi_slave_initialize(VSPI_HOST, &buscfg, &slvcfg, SPI_DMA_DISABLED);

//   Serial.println("SPI Slave ready");
// }

// void loop() {
//   memset(&t, 0, sizeof(t));
//   t.length = 8;           // 8 bits
//   t.rx_buffer = rx_buffer;

//   // Wait for packet from master
//   esp_err_t ret = spi_slave_transmit(VSPI_HOST, &t, portMAX_DELAY);

//   if (ret == ESP_OK) {
//     uint8_t angle = rx_buffer[0];
//     Serial.print("Received angle: ");
//     Serial.println(angle);

//     // Move servo
//     servo.write(angle);
//   }
// }

#include <ESP32Servo.h>

Servo myServo;
const int SERVO_PIN = 18;
const int MIN_US = 500;
const int MAX_US = 2500;
int currentAngle = 0;

int mapDegToUs(int deg) {
  deg = constrain(deg, 0, 270);
  return MIN_US + (long)deg * (MAX_US - MIN_US) / 270;
}

void moveSmooth(int target) {
  target = constrain(target, 0, 270);
  int dir = (target > currentAngle) ? 1 : -1;
  int v = 1, vMax = 6, vMin = 1, dt = 12;
  while (currentAngle != target) {
    int rem = abs(target - currentAngle);
    if (rem > 40) v = min(v + 1, vMax); else v = max(v - 1, vMin);
    currentAngle += dir * min(rem, v);
    myServo.writeMicroseconds(mapDegToUs(currentAngle));
    delay(dt);
  }
}

void setup() {
  Serial.begin(115200);
  myServo.attach(SERVO_PIN, MIN_US, MAX_US);
  moveSmooth(0);  // home
  Serial.println("Servo ready: send angle 0–270");
}

void loop() {
  if (!Serial.available()) { delay(10); return; }
  String s = Serial.readStringUntil('\n'); s.trim();
  if (s.equalsIgnoreCase("stop")) { myServo.detach(); return; }
  if (s.equalsIgnoreCase("start")) { if(!myServo.attached()) myServo.attach(SERVO_PIN, MIN_US, MAX_US); return; }
  int deg = s.toInt();
  if (deg >=0 && deg <=270) moveSmooth(deg);
}
