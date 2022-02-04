#include "DaisyDuino.h"
#include <Adafruit_MotorShield.h>

static const float LIGHT_THRESH = 0.6;
static const float RPM_SCALE = 100.0;
static const float PRESS_MIN = 650.0;
static const float PRESS_MAX = 850.0;

class Smooth {

public:
  Smooth(float t60Rise, float t60Fall, float sampleRate) :
    m_t60Rise(t60Rise),
    m_t60Fall(t60Fall),
    m_sampleInterval(1.0 / sampleRate)
  {}

  float Process(float input) {
    float t60 = input > m_y ? m_t60Rise : m_t60Fall;
    float r = 1.0 - ((6.91 * m_sampleInterval) / t60);
    m_y = input * (1.0 - r) + m_y * r;
    return m_y;
  }

private:
  float m_t60Rise;
  float m_t60Fall;
  float m_sampleInterval;
  float m_y = 0.0;
};

class Creature {

public:

  Creature(
    int lightPin, 
    int pressPin, 
    int vibePin, 
    Adafruit_DCMotor *motor
  ) : m_lightPin(lightPin),
      m_pressPin(pressPin),
      m_vibePin(vibePin),
      m_motor(motor)
  {
    pinMode(vibePin, OUTPUT);
    m_motor->setSpeed(0);
    m_motor->run(FORWARD);
  }

  float GetPressure() {
    return m_pressure;
  }

  float GetMotorSpeed() {
    return m_motorSpeed;
  }

  void Update() {
    updateMotorSpeed();
    updatePressure();
  }

private:

  int m_lightPin;
  int m_pressPin;
  int m_vibePin;

  float m_pressure = 0.0; // normalized 0-1
  float m_motorSpeed = 0.0; // normalized 0-1
  
  Adafruit_DCMotor *m_motor;
  Smooth smooth = Smooth(10.0, 30.0, 2000.0);

  void updateMotorSpeed() {
    float lightRaw = smooth.Process(analogRead(m_lightPin) / 1023.0f);
    m_motorSpeed = fmax(0.0, (lightRaw - LIGHT_THRESH) / (1.0 - LIGHT_THRESH));
    
    float rpm = m_motorSpeed * RPM_SCALE;
    m_motor->setSpeed(rpm);
  }

  void updatePressure() {
    float raw = analogRead(m_pressPin);
    float norm = (raw - PRESS_MIN) / (PRESS_MAX - PRESS_MIN);
    m_pressure = fmax(0.0, fmin(1.0, norm));
    analogWrite(m_vibePin, m_pressure * 255);
//    Serial.println(m_pressure);
  }
};

class CreatureVoice {

public:

  CreatureVoice(float sampleRate) :
    m_cutoffSmooth(Smooth(0.01, 0.01, sampleRate)) 
  {
    for (size_t i = 0; i < 4; i++) {
      m_osc[i].Init(sampleRate);
      m_osc[i].SetWaveform(Oscillator::WAVE_POLYBLEP_SAW);
      m_osc[i].SetAmp(0.3);
    }

    m_chordEnv.Init(sampleRate);
    m_chordEnv.SetTime(ADSR_SEG_ATTACK, 4.0);
    m_chordEnv.SetTime(ADSR_SEG_RELEASE, 1.0);
    m_chordEnv.SetSustainLevel(1.0);

    m_filter.Init(sampleRate);
    m_filter.SetFreq(100.0);
    m_filter.SetRes(0.3);
    m_filter.SetDrive(0.1);
  }

  void SetChordGate(bool gate) {
    if (gate != m_chordGate) {
      Serial.print("Chord gate ");
      Serial.println(gate ? "on" : "off");
      if (gate) {
         for (size_t i = 0; i < 4; i++) {
           m_osc[i].SetFreq(mtof(chords[m_chordIndex][i]));
         }
      } else {
        m_chordIndex = (m_chordIndex + 1) % 4;
      }
      m_chordGate = gate;
    }
  }

  void SetCutoff(float cutoff) {
    m_filter.SetFreq(m_cutoffSmooth.Process(cutoff));
  }

  float Process() {
    float output = 0.0;
    for (size_t i = 0; i < 4; i++) {
      output += m_osc[i].Process();
    }
    m_filter.Process(output);
    
    output = m_filter.Low() * m_chordEnv.Process(m_chordGate) * 0.5;
    
    return output;
  }

private:

  const int chords[4][4] = {
      {45, 61, 64, 66},
      {45, 56, 61, 64},
      {45, 61, 64, 66},
      {45, 56, 61, 64}
  };

  int m_chordIndex = 0;

  Oscillator m_osc[4];
  Adsr m_chordEnv;
  
  Svf m_filter;
  Smooth m_cutoffSmooth;
  
  bool m_chordGate = false;
};


DaisyHardware dsp;
size_t num_channels;

Adafruit_MotorShield motorShield = Adafruit_MotorShield();
Creature *creature1;
CreatureVoice *creatureVoice;

void AudioCallback(float **in, float **out, size_t size) {
  for (size_t i = 0; i < size; i++) {
    float samp = creatureVoice->Process();
    for (size_t chn = 0; chn < num_channels; chn++) {
      out[chn][i] = samp;
    }
  }
}

void setup() {
  Serial.begin(115200);
  dsp = DAISY.init(DAISY_SEED, AUDIO_SR_48K);
  num_channels = dsp.num_channels;
  
  if (!motorShield.begin()) {
      Serial.println("Could not find Motor Shield. Check wiring.");
    while (1);
  }
  creature1 = new Creature(A0, A1, A2, motorShield.getMotor(3));
  creatureVoice = new CreatureVoice(DAISY.get_samplerate());

  DAISY.begin(AudioCallback);
}

void loop() {
  creature1->Update();

  float motorSpeed = creature1->GetMotorSpeed();
  creatureVoice->SetChordGate(motorSpeed > 0.05);
  creatureVoice->SetCutoff(100.0 + powf(motorSpeed, 1.5) * 5000.0);
}
