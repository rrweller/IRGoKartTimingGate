#include <Arduino.h>
#include "display.h"

/* ────────── Hardware ────────── */
constexpr uint8_t LASER_PWM_PIN = 3;     // drives IR laser (Timer-2 PWM)
constexpr uint8_t IR_SENSOR_PIN = A0;    // TIA output into ADC
#define LASER_BIT _BV(PD3)

/* ────────── UI timing ────────── */
constexpr unsigned long DISP_REFRESH_MS   = 100;   // update red every 0.1 s
constexpr unsigned long FLASH_DURATION_MS = 5000;  // freeze last-lap for 5 s

/* ────────── Alignment thresholds ────────── */
constexpr uint8_t STARTUP_MIN_STRENGTH = 50;   // % needed to leave ALIGN
constexpr float   BREAK_DROP_PERCENT   = 10.0; // % drop ⇒ beam broken
constexpr float   RESTORE_HYST_PERCENT = 15.0; // % gap to declare restored
unsigned long tAlignHoldStart = 0;
constexpr unsigned long ALIGN_HOLD_MS = 10000;   // 10-s stable window

/* ────────── Signal-to-percent mapping ────────── */
constexpr float DEADZONE_VPP   = 0.03f;   // 5 mV ≈ no light
constexpr float FULL_SCALE_VPP = 0.20f;    // 0.20 Vpp ≈ strong return
constexpr float POWER_EXP      = 0.20f;    // ¼-power curve emphasises low end

/* ────────── Sample-count configuration ────────── */
// Total reads per full PWM cycle. Must be even.
constexpr uint8_t SAMPLES_PER_PERIOD = 4;
constexpr uint8_t SAMPLES_PER_PHASE = SAMPLES_PER_PERIOD / 2;

/* ────────── Globals ────────── */
volatile long lockIn = 0;          // running correlation accumulator
volatile bool phase  = 0;          // toggles each PWM half-cycle

// raw-ADC accumulators for ON vs OFF
volatile uint32_t onSum      = 0;
volatile uint32_t offSum     = 0;
volatile uint16_t onCount    = 0;
volatile uint16_t offCount   = 0;
volatile bool     cycleReady = false; // set when a full PWM cycle completes

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

/* ────────── ADC acceleration & helpers ────────── */
void configureADC() {
  ADMUX  = (1<<REFS0) | (0 & 0x07); // AVcc, channel A0, right-adjusted
  ADCSRA = (1<<ADEN) | (1<<ADPS2) | (1<<ADPS0); // prescaler=32
}

uint16_t readADC10() {
  ADCSRA |= (1<<ADSC);
  while (ADCSRA & (1<<ADSC));
  return ADC;  // full 10-bit result (0–1023)
}

static inline int8_t readSensor() {
  return (int8_t)readADC10() - 512;
}

constexpr uint8_t SUM_SHIFT =
  (SAMPLES_PER_PHASE == 1)  ? 0 :
  (SAMPLES_PER_PHASE == 2)  ? 1 :
  (SAMPLES_PER_PHASE == 4)  ? 2 :
  (SAMPLES_PER_PHASE == 8)  ? 3 :
  (SAMPLES_PER_PHASE == 16) ? 4 : 5;

/* ────────── Fast quadrature lock-in ISR ────────── */
ISR(TIMER2_COMPB_vect)
{
  long corr = 0;
  uint16_t raw;
  for (uint8_t i = 0; i < SAMPLES_PER_PHASE; ++i) {
    raw = readADC10();
    corr += ((int)raw - 512);  // center around midpoint (512 ≈ 2.5 V midpoint)

    if (PIND & LASER_BIT) {
      onSum += raw;
      onCount++;
    } else {
      offSum += raw;
      offCount++;
    }
  }

  lockIn += phase ? -corr : corr;
  lockIn -= lockIn >> 6;
  phase = !phase;

  if (!phase) cycleReady = true;
}

/* ────────── Helpers ────────── */
inline float voltsPP(long lv)
{
  return (lv/64.0f)*5.0f/1023.0f;
}

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

  /* fast ADC for IR_SENSOR_PIN */
  configureADC();

  /* laser PWM: Timer-2, D3, 490 Hz @ 50% */
  pinMode(LASER_PWM_PIN, OUTPUT);
  analogWrite(LASER_PWM_PIN, 128);
  bitSet(TIMSK2, OCIE2B);          // enable ISR

  Serial.println(F("ALIGN mode – aim for ≥50 %"));
}


/* ────────── Main loop ────────── */
void loop()
{
  // 1) Grab fresh lock-in level
  noInterrupts();
    long lv = (lockIn < 0) ? -lockIn : lockIn;
  interrupts();

  // 2) If a full cycle completed, compute true Vpp
  static float realVpp = 0;
  if (cycleReady) {
    noInterrupts();
      uint32_t os = onSum, ofs = offSum;
      uint16_t oc = onCount, ofc = offCount;
      onSum = offSum = onCount = offCount = 0;
      cycleReady = false;
    interrupts();

    if (oc && ofc) {
      float avgOn  = float(os)/oc;
      float avgOff = float(ofs)/ofc;
      realVpp = fabs(avgOn - avgOff)*(5.0f/1023.0f); // accurate to ADC 10-bit
    } else {
      realVpp = 0;
    }
  }
  uint8_t pct = signalPercent(lv);
  unsigned long nowMs = millis();

  /* ─── Serial debug (2 Hz) ─── */
  static unsigned long tPrint=0;
  if (nowMs - tPrint >= 500) {
    tPrint = nowMs;
    Serial.print(F("rate=")); Serial.print(SAMPLES_PER_PERIOD);
    Serial.print(F("  lv=")); Serial.print(lv);
    Serial.print(F("  Vpp(raw)=")); Serial.print(realVpp, 3);
    Serial.print(F("  %=")); Serial.println(pct);
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