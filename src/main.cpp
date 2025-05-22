#include <Arduino.h>
#include "display.h"

/* ------------- Hardware pins ------------- */
const uint8_t LASER_PWM_PIN = 3;
const uint8_t IR_SENSOR_PIN = A0;

/* ------------- Timing constants ------------- */
const unsigned long DISPLAY_REFRESH_MS   = 100;    // 10 Hz
const unsigned long LAP_DISPLAY_FLASH_MS = 5000;   // 5 s flash

/* ------------- Thresholds ------------- */
const uint8_t STARTUP_THRESHOLD_PERCENT = 105;
const float   BREAK_DROP_PERCENT        = 30.0;
const float   RESTORE_DROP_PERCENT      = 15.0;

/* ---------- signal-print helpers ---------- */
unsigned long lastStrengthPrintMs = 0;
const float    POWER_EXP          = 0.50f;   // √(linear %)  -> perceptual curve

/* ------------- Globals ------------- */
volatile long lockinValue = 0;
volatile bool phaseFlag   = 0;

unsigned long lockinPeak     = 100;   // **made unsigned long so signed/unsigned compare warning disappears**
unsigned long baselineLockin = 0;
long  breakThresh            = 0;
long  restoreThresh          = 0;

bool  beamPresent   = true;
bool  timingStarted = false;
unsigned long lastCrossMicros = 0;
unsigned long lastLapUs       = 0;
unsigned int  bestLapCentis   = 0;
bool  bestLapSet = false;

enum Mode { ALIGN, READY, WAIT, RUNNING, FLASH };
Mode mode = ALIGN;

unsigned long readyBlinkMs = 0;
bool readyBlinkState = false;

unsigned long lastDisplayMs = 0;
unsigned long flashStartMs  = 0;
unsigned long flashToggleMs = 0;

static unsigned long lastTransitionMs = 0;
const  unsigned long TRANSITION_DEBOUNCE_MS = 50;

/* ---------- Lock-in ISR ---------- */
ISR(TIMER2_COMPB_vect)
{
  int v = analogRead(IR_SENSOR_PIN) - 512;
  lockinValue += phaseFlag ? -v :  v;
  lockinValue -= lockinValue >> 6;      // leakage
  phaseFlag = !phaseFlag;
}

/* ---------- Setup ---------- */
void setup()
{
  Serial.begin(9600);
  DisplayDriver::begin();

  /* test 1.2 s (1200000 µs) with *plain* literal */
  DisplayDriver::showCurrentTime(1200000UL);
  delay(2000);

  DisplayDriver::showStrength(0);

  pinMode(LASER_PWM_PIN, OUTPUT);
  analogWrite(LASER_PWM_PIN, 128);      // 490 Hz, 50 %
  bitSet(TIMSK2, OCIE2B);

  Serial.println(F("ALIGN mode – raise to ≥80 %"));
}

/* ---------- Loop ---------- */
void loop()
{
  unsigned long nowMs = millis();

  /* --- lock-in read --- */
  noInterrupts();
  long lv = lockinValue;
  interrupts();
  if (lv < 0) lv = -lv;
  if (lv > (long)lockinPeak) lockinPeak = lv;           // cast to silence warning
  uint8_t strength = lockinPeak ? (lv * 100UL) / lockinPeak : 0;

  /* --- Serial debug every 200 ms --------------------------------- */
  if (nowMs - lastStrengthPrintMs >= 200) {
      lastStrengthPrintMs = nowMs;

      /* convert lock-in → peak-to-peak ADC counts → volts             */
      float countsPP = lv / 64.0f;                      // see text
      float voltsPP  = countsPP * 5.0f / 1023.0f;       // Vpp

      /* perceptual power-law (sqrt) percentage                       */
      float ratio    = lockinPeak ? (float)lv / (float)lockinPeak : 0.0f;
      float percent  = 100.0f * sqrtf(ratio);           // x^0.5

      Serial.print(F("LockIn="));  Serial.print(lv);
      Serial.print(F("  Vpp="));   Serial.print(voltsPP, 3);
      Serial.print(F(" V  Strength=")); Serial.print(percent, 1);
      Serial.println(F(" % (sqrt)"));
  }

  /* ---------- ALIGN ---------- */
  if(mode == ALIGN){
    DisplayDriver::showStrength(strength);
    delay(50);
    if(strength >= STARTUP_THRESHOLD_PERCENT){
      baselineLockin = lv;
      breakThresh   = baselineLockin*(1.0-BREAK_DROP_PERCENT/100.0);
      restoreThresh = baselineLockin*(1.0-RESTORE_DROP_PERCENT/100.0);
      mode          = READY;                // ← go to READY state
      readyBlinkMs  = nowMs;
      readyBlinkState=false;
      Serial.println(F("Beam OK – READY (waiting for first pass)"));
    }
    return;
  }

  /* ---------- READY (blink  ---  until first true break) ---------- */
if (mode == READY) {

  /* 1) blink “---” every 500 ms */
  if (nowMs - readyBlinkMs >= 500) {
    readyBlinkMs     = nowMs;
    readyBlinkState  = !readyBlinkState;
    DisplayDriver::showReady(readyBlinkState);
  }

  /* 2) Debounced first beam break */
  if (nowMs - lastTransitionMs >= TRANSITION_DEBOUNCE_MS) {

    /* a) look for drop below breakThresh to count as BROKEN */
    if (beamPresent && lv < breakThresh) {
      beamPresent       = false;
      lastTransitionMs  = nowMs;
      /* FIRST real crossing → start timer, go to RUNNING */
      timingStarted     = true;
      lastCrossMicros   = micros();
      mode              = RUNNING;
      DisplayDriver::showCurrentTime(0);   // show 0.0
      Serial.println(F("► READY → RUNNING (first pass)"));
    }

    /* b) if you ever dipped, require restore above restoreThresh
          before another try (typical when driver waves a hand)     */
    else if (!beamPresent && lv > restoreThresh) {
      beamPresent       = true;
      lastTransitionMs  = nowMs;
      Serial.println(F("(beam restored while READY)"));
    }
  }

  return;        // stay inside READY until a real, debounced break
}

  /* ---------- Debounced beam break ---------- */
  if (nowMs - lastTransitionMs >= TRANSITION_DEBOUNCE_MS) {
    if (beamPresent && lv < breakThresh) {
      beamPresent = false;
      lastTransitionMs = nowMs;
      unsigned long nowUs = micros();
      Serial.println(F("Beam BROKEN"));

      if (!timingStarted) {
        timingStarted   = true;
        lastCrossMicros = nowUs;
        mode            = RUNNING;
        Serial.println(F("Timer started"));
      } else if (mode == RUNNING) {
        unsigned long lapUs = nowUs - lastCrossMicros;
        lastCrossMicros = nowUs;
        lastLapUs       = lapUs;
        unsigned int lapCs = (lapUs + 5000) / 10000;       // centiseconds

        Serial.print(F("Lap ")); Serial.print(lapCs / 100.0, 2); Serial.println(F(" s"));

        if (!bestLapSet || lapCs < bestLapCentis) {
          bestLapCentis = lapCs;
          bestLapSet    = true;
          DisplayDriver::showBestTime(bestLapCentis);
          Serial.println(F("↳ NEW BEST"));
        }

        flashStartMs  = nowMs;
        flashToggleMs = nowMs;
        mode          = FLASH;
        DisplayDriver::showCurrentTime(lapUs);
      }
    }
    else if (!beamPresent && lv > restoreThresh) {
      beamPresent = true;
      lastTransitionMs = nowMs;
      Serial.println(F("Beam RESTORED"));
    }
  }

  /* -------------------- WAIT / RUNNING / FLASH -------------------- */
  if (mode == RUNNING) {
    if (nowMs - lastDisplayMs >= DISPLAY_REFRESH_MS) {
      lastDisplayMs = nowMs;
      DisplayDriver::showCurrentTime(micros() - lastCrossMicros);
    }
  }
  else if (mode == FLASH) {
    if (nowMs - flashStartMs >= LAP_DISPLAY_FLASH_MS) {
      mode = RUNNING;
      lastDisplayMs = nowMs;
    } else if (nowMs - flashToggleMs >= 500) {
      flashToggleMs = nowMs;
      static bool on = false;
      on = !on;
      if (on) DisplayDriver::showCurrentTime(lastLapUs);
      else    DisplayDriver::blankRedDigits();
    }
  }
}
