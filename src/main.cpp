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

/* ────────── Globals ────────── */
volatile long lockIn = 0;          // running correlation accumulator
volatile bool phase  = 0;          // toggles each PWM half-cycle

long   breakThresh  = 0;           // lv below ⇒ beam broken
long   restoreThresh= 0;           // lv above ⇒ beam restored
bool   beamPresent  = true;

bool   timingStarted = false;
unsigned long lastCrossUs = 0;     // micros at previous beam break
unsigned long lastLapUs   = 0;     // for flashing

unsigned int bestLapCs   = 0;      // best lap in centiseconds
bool         bestLapSeen = false;

/* ───────── Lap timeout ───────── */
constexpr unsigned long MAX_LAP_US = 100000000UL; // 100 s → reset
constexpr unsigned long MIN_LAP_US = 5000000UL;
constexpr unsigned long BROKEN_TIMEOUT_MS = 5000;

/* UI state */
enum class State { ALIGN, READY, RUNNING, FLASH, ERROR };
State state = State::ALIGN;

unsigned long tReadyBlink = 0;
bool          readyOn     = false;
unsigned long tStateEntry = 0;
unsigned long tLastDisp   = 0;
unsigned long tFlashStart = 0;
unsigned long tFlashToggle= 0;
unsigned long tLastDebounce=0;
unsigned long tBrokenStart = 0;

/* ────────── ISR: quadrature lock-in ────────── */
ISR(TIMER2_COMPB_vect)
{
  int sample = analogRead(IR_SENSOR_PIN) - 512;     // centre at 0
  lockIn += phase ? -sample : sample;               // + / – correlate
  lockIn -= lockIn >> 6;                            // 1/64 leakage
  phase = !phase;
}

/* ────────── Helpers ────────── */
inline float voltsPP(long lv)  { return (lv/64.0f) * 5.0f / 1023.0f; }

uint8_t signalPercent(long lv)
{
  float vpp = voltsPP(lv);
  float u   = vpp - DEADZONE_VPP;
  if (u < 0) u = 0;
  float lin = u / (FULL_SCALE_VPP - DEADZONE_VPP);
  if (lin > 1) lin = 1;
  return (uint8_t)(100.0f * powf(lin, POWER_EXP) + 0.5f);
}

/* ────────── Setup ────────── */
void setup()
{
  Serial.begin(9600);
  DisplayDriver::begin();

  /* quick power-on demo */
  DisplayDriver::showCurrentTime(888888888UL);
  delay(1500);
  DisplayDriver::showStrength(0);

  /* laser PWM: Timer-2, D3, 490 Hz @ 50 % */
  pinMode(LASER_PWM_PIN, OUTPUT);
  analogWrite(LASER_PWM_PIN, 128);
  bitSet(TIMSK2, OCIE2B);          // enable ISR

  Serial.println(F("ALIGN mode – aim for ≥50 %"));
}

/* ────────── Main loop ────────── */
void loop()
{
  /* snapshot & convert signal */
  noInterrupts();
    long lv = (lockIn < 0) ? -lockIn : lockIn;
  interrupts();
  uint8_t pct = signalPercent(lv);
  unsigned long nowMs = millis();

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
