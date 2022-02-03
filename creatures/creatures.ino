#include <Adafruit_MotorShield.h>

Adafruit_MotorShield motorShield = Adafruit_MotorShield();
Adafruit_DCMotor *motor3 = motorShield.getMotor(3);

bool isHit = false;
unsigned long lastHitChangeTime = 0;

void setup() {
  Serial.begin(115200);
  if (!motorShield.begin()) {
      Serial.println("Could not find Motor Shield. Check wiring.");
    while (1);
  }

  pinMode(D1, INPUT);
  isHit = digitalRead(D1);
  motor3->setSpeed(0);
  motor3->run(FORWARD);
}

void loop() {
  float c1Light = analogRead(A0) / 1023.0;
//  Serial.println(c1Light);
  motor3->setSpeed(max(0.0, (c1Light - 0.7) * 500.0));

  bool hitDetect = digitalRead(D1);
  if (hitDetect != isHit) {
    unsigned long now = millis();
    if (now - lastHitChangeTime > 50) {
      isHit = hitDetect;
      lastHitChangeTime = now;
      if (isHit) {
        Serial.println("Hit!");
      }
    }
  }

}
