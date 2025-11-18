#include <ESP32Servo.h>

const int SERVO_COUNT = 5;
const int SERVO_PINS[SERVO_COUNT] = {18, 19, 21, 22, 23};   // <-- set your pins
const int MIN_US = 500;
const int MAX_US = 2500;

Servo servos[SERVO_COUNT];
int currentAngle[SERVO_COUNT] = {0, 0, 0, 0, 0};

int mapDegToUs(int deg) {
  deg = constrain(deg, 0, 270);
  return MIN_US + (long)deg * (MAX_US - MIN_US) / 270;
}

void moveSmooth(int idx, int target) {
  target = constrain(target, 0, 270);

  int &cur = currentAngle[idx];
  int dir = (target > cur) ? 1 : -1;

  int v = 1, vMax = 10, vMin = 1;
  int dt = 6;

  while (cur != target) {
    int rem = abs(target - cur);
    if (rem > 40) v = min(v + 1, vMax);
    else v = max(v - 1, vMin);

    cur += dir * min(rem, v);
    servos[idx].writeMicroseconds(mapDegToUs(cur));
    delay(dt);
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("5-Servo Demonstration Starting...");

  for (int i = 0; i < SERVO_COUNT; i++) {
    servos[i].setPeriodHertz(50);
    servos[i].attach(SERVO_PINS[i], MIN_US, MAX_US);
    moveSmooth(i, 0);   // initialize all to 0°
    delay(200);
  }

  Serial.println("All servos initialized at 0°");
}


void loop() {

  for (int i = 0; i < SERVO_COUNT; i++) {
    Serial.print("Moving Servo ");
    Serial.print(i + 1);
    Serial.print(" on pin ");
    Serial.println(SERVO_PINS[i]);

    // 0° → 180°
    moveSmooth(i, 180);
    delay(500);

    // 180° → 0°
    moveSmooth(i, 0);
    delay(500);

    delay(1000); 1 sec pause
  }
}
