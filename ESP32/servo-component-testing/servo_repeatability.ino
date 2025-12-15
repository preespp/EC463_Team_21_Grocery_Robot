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

// Faster motion profile
void moveSmooth(int target) {
  target = constrain(target, 0, 270);
  int dir = (target > currentAngle) ? 1 : -1;
  int v = 1, vMax = 10, vMin = 1;   // higher vMax = faster
  int dt = 6;                       // smaller dt = faster
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
  myServo.setPeriodHertz(50);
  myServo.attach(SERVO_PIN, MIN_US, MAX_US);
  Serial.println("Enter angle 0–270 (500–2500 µs), or 'start'/'stop'.");
  
  // Always reset to 0° on power-up
  moveSmooth(0);
  Serial.println("Servo initialized at 0°.");
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

    const int HOLD_MS = 500;   // 1-second pause
    const int CYCLES  = 3;

    Serial.println("Resetting to 0°...");
    moveSmooth(0);
    delay(HOLD_MS);

    Serial.println("Running sequence: (0° → 90° → 180° → 270°, 1s each) x5");

    for (int i = 1; i <= CYCLES; ++i) {
      Serial.print("Cycle "); Serial.print(i); Serial.println("/10");

      moveSmooth(0);   delay(HOLD_MS);
      moveSmooth(90);  delay(HOLD_MS);
      moveSmooth(180); delay(HOLD_MS);;
      moveSmooth(270); delay(HOLD_MS);

      // Check for early stop
      if (Serial.available()) {
        String t = Serial.readStringUntil('\n'); t.trim();
        if (t.equalsIgnoreCase("stop")) {
          myServo.detach();
          Serial.println("🟥 Stopped mid-sequence.");
          return;
        }
      }
    }

    // Return to 0° at the end
    Serial.println("Returning to 0°...");
    moveSmooth(0);
    delay(HOLD_MS);

    myServo.detach();
    Serial.println("✅ Sequence complete. Servo stopped at 0°.");
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
