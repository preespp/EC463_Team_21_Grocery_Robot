// #include <Arduino.h>

#define TRIG_PIN 18
#define ECHO_PIN 5

// UART pins for Serial1
#define UART_RX 16
#define UART_TX 17

long duration;
float distance;

void setup() {
  Serial1.begin(115200, SERIAL_8N1, UART_RX, UART_TX); // UART to Jetson

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
}

void loop() {
  // Trigger ultrasonic pulse
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  duration = pulseIn(ECHO_PIN, HIGH);

  // Calculate distance
  distance = (duration / 2.0) / 29.1;

  // Create JSON
  String json = "{\"distance_cm\": ";
  json += String(distance, 2);
  json += "}";

  // Send over UART (Serial1)
  Serial1.println(json);

  delay(50); // ~20 Hz
}
