#include <ESP32Servo.h>

Servo myServo;
const int SERVO_PIN = 18;
const int MIN_US = 500;     // 0°
const int MAX_US = 2500;    // 270°
int currentAngle = 0;

int mapDegToUs(int deg) {
  deg = constrain(deg, 0, 270);
  return MIN_US + (long)deg * (MAX_US - MIN_US) / 270;
}

void moveSmooth(int target) {
  target = constrain(target, 0, 270);
  int dir = (target > currentAngle) ? 1 : -1;
  int v = 1, vMax = 10, vMin = 1, dt = 6; 
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
  myServo.setPeriodHertz(50);              // standard servo frequency
  myServo.attach(SERVO_PIN, MIN_US, MAX_US);
  Serial.println("Enter angle 0–270 (500–2500 µs range), or type 'start'/'stop'.");
  moveSmooth(0);                           // home softly
}

void loop() {
  if (!Serial.available()) { delay(10); return; }
  String s = Serial.readStringUntil('\n'); s.trim();

  // --- STOP COMMAND ---
  if (s.equalsIgnoreCase("stop")) {
    myServo.detach();
    Serial.println("🟥 Servo detached (stopped).");
    return;
  }

  // --- START SEQUENCE ---
  if (s.equalsIgnoreCase("start")) {
    if (!myServo.attached()) {
      myServo.attach(SERVO_PIN, MIN_US, MAX_US);
      Serial.println("🟩 Servo reattached.");
    }

    Serial.println("Running timed sequence: 0° → 45° → 90° (20s each)");

    // Move to 0°, wait 20s
    moveSmooth(0);
    Serial.println("Holding 0°...");
    delay(5000);

    // Move to 45°, wait 20s
    moveSmooth(45);
    Serial.println("Holding 45°...");
    delay(5000);

    // Move to 90°, wait 20s
    moveSmooth(90);
    Serial.println("Holding 90°...");
    delay(5000);

    // Move to 0°, wait 20s
    moveSmooth(135);
    Serial.println("Holding 0°...");
    delay(5000);

    // Move to 45°, wait 20s
    moveSmooth(180);
    Serial.println("Holding 45°...");
    delay(5000);

    // Move to 45°, wait 20s
    moveSmooth(225);
    Serial.println("Holding 45°...");
    delay(5000);

    // Move to 45°, wait 20s
    moveSmooth(270);
    Serial.println("Holding 45°...");
    delay(5000);


    // Stop servo
    myServo.detach();
    Serial.println("🟥 Sequence complete. Servo stopped.");
    return;
  }

  // --- MANUAL ANGLE COMMAND ---
  int deg = s.toInt();
  if (deg >= 0 && deg <= 270) {
    Serial.print("Moving to "); Serial.print(deg); Serial.println("°");
    moveSmooth(deg);
    Serial.println("Reached.");
  } else {
    Serial.println("Enter 0–270, or 'start'/'stop'.");
  }
}
