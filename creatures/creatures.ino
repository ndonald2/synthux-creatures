#include "DaisyDuino.h"
#include <Adafruit_MotorShield.h>

static const float LIGHT_THRESH = 0.6;
static const float RPM_SCALE = 200.0;
static const float PRESS_MIN = 650.0;
static const float PRESS_MAX = 850.0;

class Smooth {

public:
  Smooth(float t60Rise, float t60Fall) :
    m_t60Rise(t60Rise),
    m_t60Fall(t60Fall)
  {}

  void Init(float sampleRate) {
    m_sampleInterval = 1.0 / sampleRate;
  }

  float Process(float input) {
    if (m_sampleInterval == 0.0) { return input; }
    float t60 = input > m_y ? m_t60Rise : m_t60Fall;
    float r = 1.0 - ((6.91 * m_sampleInterval) / t60);
    m_y = input * (1.0 - r) + m_y * r;
    return m_y;
  }

private:
  float m_t60Rise;
  float m_t60Fall;
  float m_sampleInterval = 0.0;
  float m_y = 0.0;
};

class Creature {

public:

  Creature(int lightPin, int pressPin, int vibePin) : 
    m_lightPin(lightPin),
    m_pressPin(pressPin),
    m_vibePin(vibePin)
  {
    pinMode(vibePin, OUTPUT);
  }

  void Init(Adafruit_DCMotor *motor) {
    m_motor = motor;
    m_motor->run(FORWARD);
    m_motor->setSpeed(0);

    // Sample rate of 500Hz is a (very) gross approximation since
    // Update() loop frequency is variable/indeterminate
    m_smooth.Init(500.0);
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
  Smooth m_smooth = Smooth(2.0, 10.0);

  void updateMotorSpeed() {
    float lightRaw = m_smooth.Process(analogRead(m_lightPin) / 1023.0f);
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

  CreatureVoice() {}

  void Init(float sampleRate) {

    m_pressSmooth.Init(sampleRate);
    
    for (size_t i = 0; i < 4; i++) {
      m_osc[i].Init(sampleRate);
      m_osc[i].SetWaveform(Oscillator::WAVE_POLYBLEP_SAW);
      m_osc[i].SetAmp(0.5);
    }

    m_chordEnv.Init(sampleRate);
    m_chordEnv.SetTime(ADSR_SEG_ATTACK, 4.0);
    m_chordEnv.SetTime(ADSR_SEG_RELEASE, 3.0);
    m_chordEnv.SetSustainLevel(1.0);

    m_filter.Init(sampleRate);
    m_filter.SetFreq(100.0);
    m_filter.SetRes(0.2);
    m_filter.SetDrive(0.1);

    m_chorus.Init(sampleRate);
    m_chorus.SetDelayMs(10, 12);
    m_chorus.SetFeedback(0.3);
    m_chorus.SetLfoDepth(0.2);
    m_chorus.SetLfoFreq(0.2, 0.3);
    m_chorus.SetPan(0.1, 0.9);

    m_metro.Init(1, sampleRate);

    m_pluckEnv.Init(sampleRate);
    m_pluckEnv.SetTime(ADENV_SEG_ATTACK, 0.005);
    m_pluckEnv.SetTime(ADENV_SEG_DECAY, 0.1);

    m_fm.Init(sampleRate);
    m_fm.SetRatio(4.0);
    m_fm.SetIndex(0.0);

    m_del.Init();
    m_del.SetDelay((size_t)12000);
  }

  void SetMotorAmt(float motorAmt) {
    m_motorAmt = motorAmt;
  }

  void SetPressAmt(float pressAmt) {
    // heavy curve
    m_pressure = powf(pressAmt, 4.0);
  }

  void Process() {

    // chords
    float chordSamp = 0.0;
    for (size_t i = 0; i < 4; i++) {
      chordSamp += m_osc[i].Process();
    }

    float smoothPressure = m_pressSmooth.Process(m_pressure);
    bool gate = smoothPressure > 0.01;
    if (gate != m_chordGate) {
      if (gate) {
         for (size_t i = 0; i < 4; i++) {
           m_osc[i].SetFreq(mtof(chords[m_chordIndex][i]));
         }
      } else {
        m_chordIndex = (m_chordIndex + 1) % 4;
      }
      m_chordGate = gate;
    }
    
    m_filter.SetFreq(100.0 + smoothPressure * 5000.0);
    m_filter.Process(chordSamp);
    
    chordSamp = m_filter.Low() * m_chordEnv.Process(m_chordGate);
    m_chorus.Process(chordSamp);

    // -- pluck --

    m_metro.SetFreq(1.0 + 20.0 * m_motorAmt);
    if (m_metro.Process()) { 
      if (m_prob.Process(0.8) > 0.0) {
        int note = chords[m_chordIndex][m_pluckIndex] + 12;
        note += random(-1, 2) * 12;
        m_fm.SetFrequency(mtof(note));
        m_pluckEnv.Trigger();
      }
      m_pluckIndex = (m_pluckIndex + 1) % 4;
    }

    m_fm.SetIndex(m_motorAmt * 0.5);
    float pluckSamp = m_fm.Process() * m_pluckEnv.Process() * m_motorAmt * 0.1;

    float delayOutL = m_del.Read();
    float delayOutR = m_del.Read(6000.0);
    m_del.Write(pluckSamp + 0.3 * delayOutL);

    // -- sum --

    m_outL = m_chorus.GetLeft() + pluckSamp + delayOutL * 0.25;
    m_outR = m_chorus.GetRight() + pluckSamp + delayOutR * 0.25;
  }

  float GetL() { return m_outL; }
  float GetR() { return m_outR; }

private:

  const int chords[4][4] = {
      {45, 61, 64, 66},
      {45, 56, 61, 64},
      {52, 61, 64, 68},
      {45, 56, 61, 64}
  };

  int m_chordIndex = 0;

  Oscillator m_osc[4];
  Adsr m_chordEnv;
  Svf m_filter;
  Chorus m_chorus;

  int m_pluckIndex = 0;
  
  Metro m_metro;
  Maytrig m_prob;
  AdEnv m_pluckEnv;
  Fm2 m_fm;
  DelayLine<float, 12000> m_del;

  bool m_chordGate = false;
  float m_pressure;
  float m_motorAmt;
  
  Smooth m_pressSmooth = Smooth(2.0, 4.0);
  
  float m_outL = 0.0;
  float m_outR = 0.0;
};


DaisyHardware dsp;
size_t num_channels;

static Creature creature1 = Creature(A0, A1, A2);
static CreatureVoice creatureVoice;

Adafruit_MotorShield motorShield = Adafruit_MotorShield();

void AudioCallback(float **in, float **out, size_t size) {
  for (size_t i = 0; i < size; i++) {
    creatureVoice.Process();
    for (size_t chn = 0; chn < num_channels; chn++) {
      out[chn][i] = chn == 0 ? creatureVoice.GetL() : creatureVoice.GetR();
    }
  }
}

void setup() {
//  Serial.begin(115200);
  dsp = DAISY.init(DAISY_SEED, AUDIO_SR_48K);
  num_channels = dsp.num_channels;
  
  if (!motorShield.begin()) {
      Serial.println("Could not find Motor Shield. Check wiring.");
    while (1);
  }
  creature1.Init(motorShield.getMotor(3));
  creatureVoice.Init(DAISY.get_samplerate());

  DAISY.begin(AudioCallback);
}

void loop() {
  creature1.Update();

  float motorSpeed = creature1.GetMotorSpeed();
  float pressure = creature1.GetPressure();
  
  creatureVoice.SetMotorAmt(motorSpeed);
  creatureVoice.SetPressAmt(creature1.GetPressure());
  delay(10);
}
