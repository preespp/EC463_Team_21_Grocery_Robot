#define TRIG_PIN 18
#define ECHO_PIN 5

long duration;
float distance;
unsigned long start_time, end_time;

void setup() {
  Serial.begin(115200);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
}

void loop() {
  // trigger ultrasonic pulse
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  // measure echo
  duration = pulseIn(ECHO_PIN, HIGH);
  distance = (duration / 2.0) / 29.1;  // cm

  // JSON-style serial output
  Serial.print("{\"distance_cm\":");
  Serial.print(distance);
  Serial.println("}");

  delay(50); // ~20Hz
}
