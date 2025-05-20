#include <Arduino.h>
#include <SPI.h>
/***** Configuration Constants *****/
const uint8_t LASER_PWM_PIN   = 3;    // Laser PWM output (OC2B on Nano)
const uint8_t IR_SENSOR_PIN   = A0;   // Photodiode analog input
const uint8_t LATCH_PIN       = 10;   // TPIC6B595 latch (/SS)
const uint8_t DATA_PIN        = 11;   // TPIC6B595 data (MOSI)
const uint8_t CLOCK_PIN       = 13;   // TPIC6B595 clock (SCK)

const unsigned long DISPLAY_REFRESH_MS   = 100;   // Red display update interval
const unsigned long LAP_DISPLAY_FLASH_MS = 5000;  // Flash duration on lap

const uint8_t STARTUP_THRESHOLD_PERCENT = 80;   // Min % to pass alignment
const float   BREAK_BEAM_FRACTION       = 0.5;  // 50% signal drop = break
const float   BREAK_BEAM_HYSTERESIS     = 0.7;  // 70% to re-acquire

/***** 7-Segment Patterns (0=on, 1=off) *****/
const uint8_t SEGMENT_PATTERNS[10] = {
  0b11000000, 0b11111001, 0b10100100, 0b10110000,
  0b10011001, 0b10010010, 0b10000010, 0b11111000,
  0b10000000, 0b10010000
};
enum DigitIndex { RED0=0, RED1, RED2, GREEN0, GREEN1, GREEN2, GREEN3 };

/***** Globals *****/
volatile long lockinValue = 0;
volatile bool phaseFlag   = false;

unsigned long lastDisplayUpdate = 0;
unsigned long flashStartTime   = 0;
unsigned long lastFlashToggle   = 0;

unsigned long lastCrossTimeMicros = 0;
unsigned int  bestLapCentis       = 0;
bool          bestLapSet         = false;
bool          beamPresent        = true;
bool          timingStarted      = false;

enum Mode { MODE_ALIGN, MODE_WAIT, MODE_RUNNING, MODE_FLASH };
Mode mode = MODE_ALIGN;

long lockinMaxPeak      = 100;
long baseLockinValue    = 0;
long breakThresholdValue= 0;
long breakRestoreValue  = 0;

/***** SPI Shift-Out *****/
#include <SPI.h>
void outputShiftRegisters(const uint8_t *bytes, uint8_t count) {
  digitalWrite(LATCH_PIN, LOW);
  for (uint8_t i = 0; i < count; ++i) {
    SPI.transfer(bytes[i]);
  }
  digitalWrite(LATCH_PIN, HIGH);
}

/***** Display Routines *****/
void setDisplayToStrength(uint8_t pct) {
  uint8_t seg[7];
  for (int i=0; i<7; i++) seg[i]=0xFF;
  if (pct>999) pct=999;
  uint8_t h=pct/100, t=(pct/10)%10, o=pct%10;
  if (h) {
    seg[RED0]=SEGMENT_PATTERNS[h];
    seg[RED1]=SEGMENT_PATTERNS[t];
    seg[RED2]=SEGMENT_PATTERNS[o];
  } else if (t) {
    seg[RED1]=SEGMENT_PATTERNS[t];
    seg[RED2]=SEGMENT_PATTERNS[o];
  } else {
    seg[RED2]=SEGMENT_PATTERNS[o];
  }
  outputShiftRegisters(seg,7);
}

void setDisplayToCurrentTime(unsigned long us) {
  uint8_t seg[7];
  for(int i=0;i<7;i++) seg[i]=0xFF;
  unsigned long tenths = (us+50000)/100000;
  if (tenths>999) tenths=999;
  uint8_t h=tenths/100, t=(tenths/10)%10, o=tenths%10;
  // format S S . T
  if (h) {
    seg[RED0]=SEGMENT_PATTERNS[h];
    seg[RED1]=SEGMENT_PATTERNS[t]&0b01111111; // DP on
    seg[RED2]=SEGMENT_PATTERNS[o];
  } else if (t) {
    seg[RED0]=SEGMENT_PATTERNS[t]&0b01111111;
    seg[RED1]=SEGMENT_PATTERNS[o];
    seg[RED2]=SEGMENT_PATTERNS[0];
  } else {
    seg[RED0]=SEGMENT_PATTERNS[0]&0b01111111;
    seg[RED1]=SEGMENT_PATTERNS[o];
    seg[RED2]=SEGMENT_PATTERNS[0];
  }
  outputShiftRegisters(seg,7);
}

void setDisplayToBestTime(unsigned int centis) {
  uint8_t seg[7];
  for(int i=0;i<7;i++) seg[i]=0xFF;
  if (centis>9999) centis=9999;
  unsigned int s=centis/100, hs=centis%100;
  uint8_t t=s/10, o=s%10, d1=hs/10, d2=hs%10;
  if (t) {
    seg[GREEN0]=SEGMENT_PATTERNS[t];
    seg[GREEN1]=SEGMENT_PATTERNS[o]&0b01111111;
    seg[GREEN2]=SEGMENT_PATTERNS[d1];
    seg[GREEN3]=SEGMENT_PATTERNS[d2];
  } else if (o) {
    seg[GREEN1]=SEGMENT_PATTERNS[o]&0b01111111;
    seg[GREEN2]=SEGMENT_PATTERNS[d1];
    seg[GREEN3]=SEGMENT_PATTERNS[d2];
  } else {
    seg[GREEN1]=SEGMENT_PATTERNS[0]&0b01111111;
    seg[GREEN2]=SEGMENT_PATTERNS[d1];
    seg[GREEN3]=SEGMENT_PATTERNS[d2];
  }
  outputShiftRegisters(seg,7);
}

/***** Lock-in ISR *****/
ISR(TIMER2_COMPB_vect) {
  int v = analogRead(IR_SENSOR_PIN) - 512;
  if (!phaseFlag)      lockinValue += v;
  else                 lockinValue -= v;
  lockinValue -= lockinValue >> 6;  // decay
  phaseFlag = !phaseFlag;
}

/***** Setup *****/
void setup() {
  Serial.begin(9600);
  pinMode(LATCH_PIN, OUTPUT);
  pinMode(DATA_PIN, OUTPUT);
  pinMode(CLOCK_PIN, OUTPUT);
  pinMode(LASER_PWM_PIN, OUTPUT);
  digitalWrite(LATCH_PIN, LOW);
  SPI.begin();
  SPI.beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE0));

  // 490 Hz 50% PWM on D3
  analogWrite(LASER_PWM_PIN, 128);
  bitSet(TIMSK2, OCIE2B);  // enable compare-match B ISR

  delay(10);
  setDisplayToStrength(0);
  Serial.println(">>> System booting, ALIGN mode. Show signal strength % on RED digits.");
}

/***** Main Loop *****/
void loop() {
  // Read lock-in value safely
  noInterrupts();
  long lv = lockinValue;
  interrupts();

  if (lv<0) lv = -lv;
  if (lv > lockinMaxPeak) {
    lockinMaxPeak = lv;
    Serial.print("New peak lock-in: "); Serial.println(lockinMaxPeak);
  }
  uint8_t strength = (lockinMaxPeak>0) ? constrain((lv*100L)/lockinMaxPeak,0,100) : 0;

  /**** ALIGN MODE ****/
  if (mode == MODE_ALIGN) {
    setDisplayToStrength(strength);
    Serial.print("ALIGN: lock-in="); Serial.print(lv);
    Serial.print("  %="); Serial.println(strength);
    if (strength >= STARTUP_THRESHOLD_PERCENT) {
      mode = MODE_WAIT;
      beamPresent = true;
      timingStarted = false;
      baseLockinValue   = lv;
      breakThresholdValue = baseLockinValue * BREAK_BEAM_FRACTION;
      breakRestoreValue   = baseLockinValue * BREAK_BEAM_HYSTERESIS;
      Serial.println(">>> ALIGN OK – switching to WAIT for first crossing");
      setDisplayToBestTime(0);
      setDisplayToCurrentTime(0);
    }
    delay(100);
    return;
  }

  /**** BEAM BREAK LOGIC ****/
  bool currentBeam = (lv > breakThresholdValue);
  if (beamPresent && !currentBeam) {
    beamPresent = false;
    Serial.println("** Beam BROKEN **");
    unsigned long nowUs = micros();
    if (!timingStarted) {
      timingStarted = true;
      lastCrossTimeMicros = nowUs;
      mode = MODE_RUNNING;
      Serial.println(">>> Timing STARTED");
    } else if (mode==MODE_RUNNING) {
      unsigned long lapUs = nowUs - lastCrossTimeMicros;
      lastCrossTimeMicros = nowUs;
      unsigned int lapC  = (lapUs+5000)/10000;
      Serial.print(">>> Lap COMPLETE: "); Serial.print(lapC/100.0,2); Serial.println(" s");
      if (!bestLapSet || lapC < bestLapCentis) {
        bestLapCentis = lapC;
        bestLapSet = true;
        Serial.print("---- New BEST: "); Serial.print(bestLapCentis/100.0,2); Serial.println(" s");
        setDisplayToBestTime(bestLapCentis);
      }
      flashStartTime = millis();
      lastFlashToggle = flashStartTime;
      mode = MODE_FLASH;
      setDisplayToCurrentTime(lapUs); // show last lap immediately
    }
  }
  else if (!beamPresent && currentBeam) {
    beamPresent = true;
    Serial.println("** Beam RESTORED **");
  }

  /**** DISPLAY UPDATE ****/
  unsigned long nowMs = millis();
  if (mode == MODE_RUNNING) {
    if (nowMs - lastDisplayUpdate >= DISPLAY_REFRESH_MS) {
      lastDisplayUpdate = nowMs;
      unsigned long elapsedUs = micros() - lastCrossTimeMicros;
      setDisplayToCurrentTime(elapsedUs);
      Serial.print("RUNNING: "); Serial.print(elapsedUs/1000000.0,2);
      Serial.println(" s");
    }
  }
  else if (mode == MODE_FLASH) {
    unsigned long ef = nowMs - flashStartTime;
    if (ef >= LAP_DISPLAY_FLASH_MS) {
      mode = MODE_RUNNING;
      lastDisplayUpdate = nowMs;
      Serial.println(">>> FLASH over, BACK TO RUNNING");
    } else if (nowMs - lastFlashToggle >= 500) {
      lastFlashToggle = nowMs;
      static bool show = true;
      show = !show;
      if (!show) {
        uint8_t blank[7];
        for (int i=0;i<7;i++) blank[i]=0xFF;
        outputShiftRegisters(blank,7);
      } else {
        // re-latch last lap time (we rely on it still being in the registers)
        Serial.println(">>> FLASH TOGGLE ON");
      }
    }
  }
}
