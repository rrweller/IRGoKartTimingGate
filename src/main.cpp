#include <Arduino.h>
#include "display.h"

/* ────────── Hardware ────────── */
constexpr uint8_t LASER_PWM_PIN = 3;     // drives IR laser (Timer-2 PWM)
constexpr uint8_t IR_SENSOR_PIN = A0;    // TIA output into ADC

/* ────────── UI timing ────────── */
constexpr unsigned long DISP_REFRESH_MS   = 100;   // update red every 0.1 s
constexpr unsigned long FLASH_DURATION_MS = 5000;  // freeze last-lap for 5 s

/* ────────── Alignment thresholds ────────── */
constexpr uint8_t STARTUP_MIN_STRENGTH = 50;   // % needed to leave ALIGN
constexpr float   BREAK_DROP_PERCENT   = 10.0; // % drop ⇒ beam broken
constexpr float   RESTORE_HYST_PERCENT = 15.0; // % gap to declare restored
unsigned long tAlignHoldStart = 0;
constexpr unsigned long ALIGN_HOLD_MS = 10000;   // 10-s stable window

/* ────────── Signal-to-percent mapping ──────────
   lv → Vpp = (lv / 64) * 5/1023  (see ISR note)
   then map [DEADZONE_VPP .. FULL_SCALE_VPP] → [0..100] with power law  */
constexpr float DEADZONE_VPP   = 0.0055f;   // 5 mV ≈ no light
constexpr float FULL_SCALE_VPP = 0.20f;    // 0.20 Vpp ≈ strong return
constexpr float POWER_EXP      = 0.20f;    // ¼-power curve emphasises low end


// How many full PWM (490 Hz) cycles to accumulate before a "lock" sample:
constexpr uint8_t INTEGRATION_CYCLES = 5;  // ≃10 ms window (5×2 ms)

/* ────────── Globals ────────── */
volatile long  _lockAcc       = 0;  // accumulating correlator
volatile bool  _phase         = false;  // toggles each half cycle

// Cycle‐counting & sampled lock value
volatile uint8_t  _halfCycleCount = 0;
volatile uint8_t  _cycleCount     = 0;
volatile long     lockSample      = 0;  // snapshot after INTEGRATION_CYCLES

// Dynamic noise floor (OFF‐phase only)
float noiseFloor = 0.002f;

/* Thresholds computed after ALIGN */
long breakThresh = 0, restoreThresh = 0;
bool beamPresent = true;

/* Lap‐timing state */
bool   timingStarted = false;
unsigned long lastCrossUs = 0, lastLapUs = 0;
unsigned int  bestLapCs  = 0; bool bestLapSeen = false;

/* Lap timeout / min‐lap */
constexpr unsigned long MAX_LAP_US     = 100000000UL;
constexpr unsigned long MIN_LAP_US     = 5000000UL;
constexpr unsigned long BROKEN_TIMEOUT_MS = 5000;

/* ────────── State machine ────────── */
enum class State { ALIGN, READY, RUNNING, FLASH, ERROR };
State state = State::ALIGN;

unsigned long tStateEntry   = 0;
unsigned long tLastDebounce = 0;
unsigned long tBrokenStart  = 0;
unsigned long tLastDisp     = 0;
unsigned long tFlashStart   = 0;
unsigned long tFlashToggle  = 0;
unsigned long tReadyBlink   = 0;
bool          readyOn       = false;

/* ────────── ISR: PWM half‐cycle toggler & leak τ≈130 ms ────────── */
ISR(TIMER2_COMPB_vect) {
  // just flip phase every half cycle of 490 Hz → 980 Hz toggles
  _phase = !_phase;

  // apply the 1/128 leak
  _lockAcc -= _lockAcc >> 7;

  // count half cycles → full cycles
  if (++_halfCycleCount >= 2) {
    _halfCycleCount = 0;
    // measure one full cycle
    if (++_cycleCount >= INTEGRATION_CYCLES) {
      lockSample    = _lockAcc;
      _cycleCount   = 0;
      //_lockAcc    = 0; 
    }
  }
}

/* ── ISR: free-running ADC @ ≈9.6 kHz, update lock/ noiseFloor ── */
ISR(ADC_vect) {
  int samp = ADC - 512;             // raw signed sample

  // correlate into lock
  _lockAcc += (_phase ? -samp : samp);

  // dynamic noise floor tracks OFF‐phase only:
  if (!_phase) {
    // simple IIR: noiseFloor ← α·noiseFloor + (1–α)·|samp|
    constexpr float a = 0.995f;
    noiseFloor = a*noiseFloor + (1.0f-a)*(abs(samp)*5.0f/1023.0f);
  }
}

/* ────────── Helpers ────────── */
inline float voltsPP(long lv) {
  return (lv/64.0f)*(5.0f/1023.0f);
}
uint8_t signalPercent(long lv) {
  // map [noiseFloor..FULL_SCALE_VPP] → [0..100] with power law
  float vpp = voltsPP(lv);
  float u   = vpp - noiseFloor - DEADZONE_VPP;
  if (u < 0) u = 0;
  float lin = u / (FULL_SCALE_VPP - DEADZONE_VPP);
  if (lin > 1) lin = 1;
  return uint8_t(100*powf(lin, POWER_EXP) + 0.5f);
}

/* ────────── Setup ────────── */
void setup(){
  Serial.begin(9600);
  DisplayDriver::begin();

  // Demo
  DisplayDriver::showCurrentTime(1200000UL);
  delay(2000);
  DisplayDriver::showStrength(0);

  // Timer2: 490 Hz PWM on D3 & COMPB interrupt
  pinMode(LASER_PWM_PIN, OUTPUT);
  analogWrite(LASER_PWM_PIN, 128);
  TIMSK2 |= _BV(OCIE2B);

  // ADC free‐run @ prescaler 128 → ≈9.6 kHz conversions
  ADMUX  = _BV(REFS0) | (IR_SENSOR_PIN & 0x07);
  ADCSRA = _BV(ADEN) | _BV(ADATE) | _BV(ADIE)
         | _BV(ADPS2)|_BV(ADPS1)|_BV(ADPS0);
  ADCSRB = 0;        // free‐running
  ADCSRA |= _BV(ADSC);  // start

  tStateEntry = millis();
  Serial.println(F("ALIGN mode – aim for ≥50 %"));
}

/* ────────── Main loop ────────── */
void loop(){
  unsigned long nowMs = millis();

  // take snapshot of the most recent integrated lockSample:
  long lv = (lockSample<0 ? -lockSample : lockSample);

  // compute strength
  uint8_t pct = signalPercent(lv);

  /* ─── Serial debug (2 Hz) ─── */
  static unsigned long tPrint=0;
  if (nowMs - tPrint >= 500) {
    tPrint = nowMs;
    Serial.print(F("lv="));   Serial.print(lv);
    Serial.print(F("  Vpp="));Serial.print(voltsPP(lv),3);
    Serial.print(F(" V  %="));Serial.println(pct);
  }

  /* ─── State machine ─── */
  switch (state) {
    /* ---------- ALIGN ---------- */
    case State::ALIGN:
      if (nowMs - tStateEntry < 2000) {
          DisplayDriver::showBanner("ALN");
          return;
      }
      DisplayDriver::showStrength(pct);
      if (pct >= STARTUP_MIN_STRENGTH) {
          if (tAlignHoldStart == 0)          // start 10-s timer
              tAlignHoldStart = nowMs;
          else if (nowMs - tAlignHoldStart >= ALIGN_HOLD_MS) {
              breakThresh   = lv * (1.0 - BREAK_DROP_PERCENT/100.0);
              restoreThresh = lv * (1.0 - RESTORE_HYST_PERCENT/100.0);
              state         = State::READY;
              tStateEntry = nowMs;
              tReadyBlink   = nowMs;
              readyOn       = false;
              beamPresent   = true;
              tBrokenStart = 0;
              Serial.println(F("Beam stable – READY (waiting for first pass)"));
          }
      } else {
          tAlignHoldStart = 0;  // lost strength → restart timer
      }
      return;

    /* ---------- READY ---------- */
    case State::READY:
      if (nowMs - tStateEntry < 2000) {      // one-second “RDY”
          DisplayDriver::showBanner("RDY");
          return;
      }
      if (nowMs - tReadyBlink >= 500) {
        tReadyBlink = nowMs;
        readyOn = !readyOn;
        DisplayDriver::showReady(readyOn);
      }
      /* debounce first break */
      if (nowMs - tLastDebounce >= 50) {
        if (beamPresent && lv < breakThresh) {
          beamPresent = false;
          tLastDebounce = nowMs;
          timingStarted = true;
          lastCrossUs   = micros();
          DisplayDriver::showCurrentTime(0);
          state = State::RUNNING;
          tStateEntry = nowMs;
          Serial.println(F("► Timer START"));
        }
      }
      return;

    /* ---------- ERROR ---------- */
    case State::ERROR:
      delay(5000);
      state = State::ALIGN;
      tStateEntry = nowMs;
      DisplayDriver::blankAll(); 
      bestLapCs = 0; 
      bestLapSeen = false;
      timingStarted = false;
      tAlignHoldStart = 0;
      tBrokenStart = 0;
      return;

    /* ---------- RUNNING / FLASH shared break-detect ---------- */
    default:
      if (nowMs - tLastDebounce >= 50) {
        if (beamPresent && lv < breakThresh) {
          beamPresent     = false;
          tLastDebounce   = nowMs;
          tBrokenStart = nowMs;

          unsigned long nowUs = micros();
          unsigned long lapUs = nowUs - lastCrossUs;

          /*  IGNORE if break happens too soon (e.g., rear wheels)  */
          if (state == State::RUNNING && lapUs < MIN_LAP_US) {
              Serial.println(F("-- spurious short break ignored --"));
              /* do NOT update lastCrossUs — wait for real lap */
          }
          else {
              /* accept lap (or first pass from READY) */
              lastCrossUs = nowUs;

              if (state == State::RUNNING) {                // real new lap
                  lastLapUs = lapUs;
                  unsigned int lapCs = (lapUs + 5000) / 10000;
                  Serial.print(F("Lap ")); Serial.print(lapCs / 100.0, 2);
                  Serial.println(F(" s"));

                  if (!bestLapSeen || lapCs < bestLapCs) {
                      bestLapCs   = lapCs;
                      bestLapSeen = true;
                      DisplayDriver::showBestTime(bestLapCs);
                      Serial.println(F("↳ NEW BEST"));
                  }
                  tFlashStart  = nowMs;
                  tFlashToggle = nowMs;
                  state        = State::FLASH;
                  tStateEntry = nowMs;
                  DisplayDriver::showCurrentTime(lapUs);
              }
              else if (state == State::READY) {             // first pass
                  timingStarted = true;
                  DisplayDriver::showCurrentTime(0);
                  state = State::RUNNING;
                  tStateEntry = nowMs;
                  Serial.println(F("► Timer START"));
              }
          }
      }
        else if (!beamPresent && lv > restoreThresh) {
          beamPresent = true;
          tLastDebounce = nowMs;
          tBrokenStart = 0;
        }
        /* beam lost too long → ERROR banner */
        if (!beamPresent && tBrokenStart &&
            nowMs - tBrokenStart >= BROKEN_TIMEOUT_MS &&
            (state == State::RUNNING || state == State::FLASH)) {
            DisplayDriver::showBanner("ERR");
            state         = State::ERROR;
            Serial.println(F("Beam lost – ERR state"));
        }
      }
      break;
  }

  /* ---------- RUNNING display ---------- */
  if (state == State::RUNNING) {
    /* lap timeout: no break for 100 s */
    if (micros() - lastCrossUs >= MAX_LAP_US) {
        Serial.println(F("‼ 100-s timeout – returning to ALIGN"));
        delay(2500);
        state = State::ALIGN;
        tStateEntry = nowMs;
        DisplayDriver::blankAll(); 
        bestLapCs = 0; 
        bestLapSeen = false;
        timingStarted = false;
        tAlignHoldStart = 0;
        tBrokenStart = 0;
        return;
    }

    /* normal 0.1-s display refresh */
    if (nowMs - tLastDisp >= DISP_REFRESH_MS) {
        tLastDisp = nowMs;
        DisplayDriver::showCurrentTime(micros() - lastCrossUs);
    }
  }

  /* ---------- FLASH state ---------- */
  if (state == State::FLASH) {
    if (nowMs - tFlashStart >= FLASH_DURATION_MS) {
      state = State::RUNNING;
      tStateEntry = nowMs;
      tLastDisp = nowMs;
    } else if (nowMs - tFlashToggle >= 500) {
      tFlashToggle = nowMs;
      static bool on=false;
      on = !on;
      on ? DisplayDriver::showCurrentTime(lastLapUs)
         : DisplayDriver::blankRedDigits();
    }
  }
}
