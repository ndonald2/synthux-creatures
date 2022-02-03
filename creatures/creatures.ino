#include <Adafruit_MotorShield.h>

static const float LIGHT_THRESH = 0.5;
static const float RPM_SCALE = 100.0;
static const unsigned long HIT_DEBOUNCE_MS = 10;

class Creature {

public:

  Creature(int hitPin, int lightPin, Adafruit_DCMotor *motor) :
    m_hitPin(hitPin),
    m_lightPin(lightPin),
    m_motor(motor)
  {
    pinMode(hitPin, INPUT);  
    m_isHit = digitalRead(hitPin);

    m_motor->setSpeed(0);
    m_motor->run(FORWARD);
  }

  void Update() {
    updateMotorSpeed(); 
    detectHit();
  }

private:

  int m_hitPin;
  int m_lightPin;
  Adafruit_DCMotor *m_motor;

  bool m_isHit = false;
  unsigned long m_lastHitChangeTime = 0;

  void updateMotorSpeed() {
    // TODO: Generous smoothing/integrator
    float lightAmt = analogRead(m_lightPin) / 1023.0f;
    m_motor->setSpeed(max(0.0f, (lightAmt - LIGHT_THRESH) * RPM_SCALE));
  }

  void detectHit() {
    bool hitDetect = digitalRead(m_hitPin);
    if (hitDetect != m_isHit) {
      unsigned long now = millis();
      if (now - m_lastHitChangeTime > HIT_DEBOUNCE_MS) {
        m_isHit = hitDetect;
        m_lastHitChangeTime = now;
        if (m_isHit) {
          Serial.println("Hit!");
        }
      }
    }
  }
};

Adafruit_MotorShield motorShield = Adafruit_MotorShield();
Creature *creature1;

void setup() {
  Serial.begin(115200);
  if (!motorShield.begin()) {
      Serial.println("Could not find Motor Shield. Check wiring.");
    while (1);
  }

  creature1 = new Creature(D1, A0, motorShield.getMotor(3));
}

void loop() {
  creature1->Update();
}
