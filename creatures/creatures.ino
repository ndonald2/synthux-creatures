#include <Adafruit_MotorShield.h>

static const float LIGHT_THRESH = 0.6;
static const float RPM_SCALE = 150.0;

static const float PRESS_MIN = 650.0;
static const float PRESS_MAX = 850.0;

class Smooth {

public:
  Smooth(float t60Rise, float t60Fall) :
    m_t60Rise(t60Rise),
    m_t60Fall(t60Fall)
  {}

  float Process(float input) {
    
    unsigned long now = millis();
    float interval = (float)(now - m_lastMillis) / 1000.0;
    m_lastMillis = now;

    float t60 = input > m_y ? m_t60Rise : m_t60Fall;
    
    float r = 1.0 - ((6.91 * interval) / t60);
    m_y = input * (1.0 - r) + m_y * r;
    return m_y;
  }

private:
  float m_t60Rise;
  float m_t60Fall;
  float m_y = 0.0;
  unsigned long m_lastMillis = 0;
};

class Creature {

public:

  Creature(
    int lightPin, 
    int pressPin, 
    int vibePin, 
    int ledPin,
    Adafruit_DCMotor *motor
  ) : m_lightPin(lightPin),
      m_pressPin(pressPin),
      m_vibePin(vibePin),
      m_ledPin(ledPin),
      m_motor(motor)
  {
    pinMode(vibePin, OUTPUT);
    pinMode(ledPin, OUTPUT);
    m_motor->setSpeed(0);
    m_motor->run(FORWARD);
  }

  void Update() {
    updateMotorSpeed();
    updatePressure();
  }

private:

  int m_lightPin;
  int m_pressPin;
  int m_vibePin;
  int m_ledPin;
  Adafruit_DCMotor *m_motor;
  Smooth smooth = Smooth(5.0, 60.0);

  void updateMotorSpeed() {
    float lightAmt = analogRead(m_lightPin) / 1023.0f;
    lightAmt = smooth.Process(lightAmt);
    float motorSpeed = ((lightAmt - LIGHT_THRESH) / (1.0 - LIGHT_THRESH)) * RPM_SCALE;
    m_motor->setSpeed(fmax(0.0, motorSpeed));
    digitalWrite(m_ledPin, lightAmt > 0.2 ? HIGH : LOW);
  }

  void updatePressure() {
    float raw = analogRead(m_pressPin);
    float norm = (raw - PRESS_MIN) / (PRESS_MAX - PRESS_MIN);
    norm = fmax(0.0, fmin(1.0, norm));
    analogWrite(m_vibePin, norm * 255);
//    Serial.println(norm);
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
  creature1 = new Creature(A0, A1, A2, D30, motorShield.getMotor(3));
}

void loop() {
  creature1->Update();
}
