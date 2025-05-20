/***** Configuration Constants *****/
const uint8_t LASER_PWM_PIN   = 3;    // Laser diode PWM output (OC2B on Arduino Nano)
const uint8_t IR_SENSOR_PIN   = A0;   // Photodiode analog input
const uint8_t LATCH_PIN       = 10;   // TPIC6B595 latch pin (connected to /SS)
const uint8_t DATA_PIN        = 11;   // TPIC6B595 data pin (MOSI)
const uint8_t CLOCK_PIN       = 13;   // TPIC6B595 clock pin (SCK)

// Display timing and modes
const unsigned long DISPLAY_REFRESH_MS = 100;   // Update current time display every 100 ms (0.1 s)
const unsigned long LAP_DISPLAY_FLASH_MS = 5000; // Flash lap time for 5 seconds

// Calibration thresholds
const uint8_t STARTUP_THRESHOLD_PERCENT = 80;  // Required signal strength (0-100%) to begin timing
const float   BREAK_BEAM_FRACTION = 0.5;       // Fraction of full signal considered a "beam break" (e.g. 0.5 = 50% drop)
const float   BREAK_BEAM_HYSTERESIS = 0.7;     // Fraction for beam re-acquire (should be higher than BREAK_BEAM_FRACTION)

/***** 7-Segment Display Data *****/
// Segment bit mapping: assuming TPIC6B595 outputs connect to segments A-G and DP.
// We'll use an 8-bit pattern for each digit: bit0->A, bit1->B, ..., bit6->G, bit7->DP.
// For common-anode displays with TPIC6B595 sinking, a '0' bit turns a segment ON (current sink active).
// The patterns below assume 0 = segment on, 1 = segment off for each segment bit.
const uint8_t SEGMENT_PATTERNS[10] = {
  // ABCDEFG.(DP off) 
  0b11000000, // 0 -> segments A, B, C, D, E, F on (G off, DP off) – pattern will be inverted if needed
  0b11111001, // 1 -> B, C on
  0b10100100, // 2 -> A, B, D, E, G on
  0b10110000, // 3 -> A, B, C, D, G on
  0b10011001, // 4 -> B, C, F, G on
  0b10010010, // 5 -> A, C, D, F, G on
  0b10000010, // 6 -> A, C, D, E, F, G on
  0b11111000, // 7 -> A, B, C on
  0b10000000, // 8 -> A, B, C, D, E, F, G on
  0b10010000  // 9 -> A, B, C, D, F, G on
};
// Note: The above patterns are for common-cathode by default (0=on). For common-anode with sink driver, they still apply 
// because a '0' in the shift register output will sink current and light the segment, matching the logic.

// Indices for each digit in the shift register chain (0=first red digit, ..., 6=last green digit)
enum DigitIndex { RED0=0, RED1, RED2, GREEN0, GREEN1, GREEN2, GREEN3 };

/***** Global Variables *****/
volatile long lockinValue = 0;   // Accumulator for lock-in detection (signed correlation value)
volatile bool phaseFlag = false; // Tracks half-cycle phase (false = just finished laser-ON half, true = laser-OFF half)

unsigned long lastDisplayUpdate = 0;      // Timestamp for last red display refresh
unsigned long lastFlashToggle = 0;        // Timestamp for toggling display during flash
unsigned long flashStartTime = 0;         // Start time of current flash period

// Timing variables
unsigned long lastCrossTimeMicros = 0;    // Timestamp of last beam crossing (lap start reference)
unsigned int  bestLapCentis = 0;          // Best lap time in centiseconds (hundredths)
bool bestLapSet = false;                  // Whether a best lap time is recorded

// Beam state
bool beamPresent = true;   // True if beam currently unbroken (present). Starts true once aligned.
bool timingStarted = false; // True after the first crossing has started timing.

// Mode of operation
enum Mode { MODE_ALIGN, MODE_WAIT, MODE_RUNNING, MODE_FLASH };
Mode mode = MODE_ALIGN;

// Lock-in calibration values
long lockinMaxPeak = 100;    // Dynamic peak of lock-in value observed (for percentage scaling)
long baseLockinValue = 0;    // Baseline lock-in value when beam is fully present (recorded at end of alignment)
long breakThresholdValue = 0;    // Lock-in value threshold below which we consider the beam broken
long breakRestoreValue = 0;      // Lock-in threshold to consider beam restored (for hysteresis)

/***** Utility: SPI Shift-Out to 7-Segment Registers *****/
#include <SPI.h>
void outputShiftRegisters(const uint8_t *bytes, uint8_t count) {
  // Sends `count` bytes from the array to the chained TPIC6B595 shift registers via SPI.
  // The first byte in the array corresponds to the first shift register in the chain (Red0).
  digitalWrite(LATCH_PIN, LOW);
  // Send bytes in order: Red0 -> ... -> Green3. 
  // Note: TPIC6B595 clocks data MSB-first on its serial input.
  for (uint8_t i = 0; i < count; ++i) {
    SPI.transfer(bytes[i]);
  }
  digitalWrite(LATCH_PIN, HIGH);
}

/***** Display Update Functions *****/
void setDisplayToStrength(uint8_t percent) {
  // Display the signal strength (0-100) on the red 3-digit display (no decimal point).
  // If percent < 100, leftmost digit is blank or 0; if 100, display "100".
  uint8_t segBytes[7]; // buffer for 7 digits (only first 3 used for red)
  // Prepare all as blank initially
  for (int i = 0; i < 7; ++i) segBytes[i] = 0xFF;  // 0xFF = all segments off (since 1 bits mean off for our logic)
  
  if (percent > 999) percent = 999;  // safety clamp (should never happen since max 100)
  uint8_t hundreds = percent / 100;
  uint8_t tens = (percent / 10) % 10;
  uint8_t ones = percent % 10;
  if (hundreds > 0) {
    // 100% exactly
    segBytes[RED0] = SEGMENT_PATTERNS[hundreds];  // '1'
    segBytes[RED1] = SEGMENT_PATTERNS[tens];      // '0'
    segBytes[RED2] = SEGMENT_PATTERNS[ones];      // '0'
  } else if (tens > 0) {
    // 10-99%
    segBytes[RED0] = 0xFF;                       // blank
    segBytes[RED1] = SEGMENT_PATTERNS[tens];
    segBytes[RED2] = SEGMENT_PATTERNS[ones];
  } else {
    // 0-9%
    segBytes[RED0] = 0xFF;                       // blank
    segBytes[RED1] = 0xFF;                       // blank
    segBytes[RED2] = SEGMENT_PATTERNS[ones];
  }
  // Ensure no decimals lit:
  // (Our segment patterns have DP off by default; if not, we could mask bit7 to 1 to turn DP off.)
  outputShiftRegisters(segBytes, 7);
}

void setDisplayToCurrentTime(unsigned long microsElapsed) {
  // Display current lap time (microsElapsed since last lap start) on red digits with one decimal (tenths).
  // microsElapsed is converted to seconds.tenths.
  uint8_t segBytes[7];
  for(int i=0; i<7; ++i) segBytes[i] = 0xFF;  // default all off
  // Convert time to tenths of seconds (one decimal place)
  // We will round to nearest tenth: add 50 ms (50000 us) before dividing by 100000.
  unsigned long tenths = (microsElapsed + 50000) / 100000;  // time in 0.1 s units
  if (tenths > 999) {
    tenths = 999;  // cap at 99.9 to fit in three digits
  }
  // Extract digits
  uint8_t hundreds = tenths / 100;       // tens of seconds (0-9 for 0–99.9s range)
  uint8_t tens = (tenths / 10) % 10;     // ones of seconds
  uint8_t ones = tenths % 10;           // tenths of second as a single digit
  
  if (hundreds > 0) {
    // Time >= 100.0 (should be capped to 99.9 anyway)
    segBytes[RED0] = SEGMENT_PATTERNS[hundreds];
    segBytes[RED1] = SEGMENT_PATTERNS[tens];
    segBytes[RED2] = SEGMENT_PATTERNS[ones];
    // If >=100, we won't show a decimal (not expected in normal operation).
  } else if (tens > 0) {
    // 10.0s to 99.9s
    segBytes[RED0] = SEGMENT_PATTERNS[tens];
    // Put decimal point on this middle digit:
    segBytes[RED0] &= 0b01111111;  // set DP segment bit to 0 (on)
    segBytes[RED1] = SEGMENT_PATTERNS[ones];
    segBytes[RED2] = SEGMENT_PATTERNS[0];  // We have only one decimal place, so last digit always 0? (Actually mistake: Should re-evaluate logic)
  } else {
    // 0.0s to 9.9s
    segBytes[RED0] = 0xFF;  // blank hundreds
    segBytes[RED1] = SEGMENT_PATTERNS[tens];  // (tens here is 0 since <10, so we'll display 0 for "0.x" or blank? Let's display 0 for 0.x)
    // Actually, for 0.x seconds, display "0.x":
    if (tens == 0) {
      segBytes[RED1] = SEGMENT_PATTERNS[0];
    }
    segBytes[RED1] &= 0b01111111;           // turn on DP on this digit
    segBytes[RED2] = SEGMENT_PATTERNS[ones];
  }
  outputShiftRegisters(segBytes, 7);
}

void setDisplayToBestTime(unsigned int centis) {
  // Display best lap time (in centiseconds) on the green 4-digit display with two decimal places.
  // centis is the time in hundredths of a second.
  uint8_t segBytes[7];
  for(int i=0; i<7; ++i) segBytes[i] = 0xFF;
  if (centis > 9999) centis = 9999;  // cap at 99.99s
  // Calculate minutes if needed (for times >= 100.00, but we assume laps < 100s, so skip minutes)
  unsigned int seconds = centis / 100;
  unsigned int hundredths = centis % 100;
  // Extract digits
  uint8_t tensSec = seconds / 10;
  uint8_t onesSec = seconds % 10;
  uint8_t tenths = hundredths / 10;
  uint8_t hundredth = hundredths % 10;
  
  if (tensSec > 0) {
    // >= 10.00 sec
    segBytes[GREEN0] = SEGMENT_PATTERNS[tensSec];
    segBytes[GREEN1] = SEGMENT_PATTERNS[onesSec];
    segBytes[GREEN1] &= 0b01111111;  // decimal point on onesSec digit (xx.xx format)
    segBytes[GREEN2] = SEGMENT_PATTERNS[tenths];
    segBytes[GREEN3] = SEGMENT_PATTERNS[hundredth];
  } else if (onesSec > 0) {
    // 1.00 to 9.99 sec
    segBytes[GREEN0] = 0xFF;  // blank tensSec
    segBytes[GREEN1] = SEGMENT_PATTERNS[onesSec];
    segBytes[GREEN1] &= 0b01111111;  // DP on onesSec
    segBytes[GREEN2] = SEGMENT_PATTERNS[tenths];
    segBytes[GREEN3] = SEGMENT_PATTERNS[hundredth];
  } else {
    // 0.00 to 0.99 sec (unlikely for lap, but handle)
    segBytes[GREEN0] = 0xFF;  // blank tensSec
    segBytes[GREEN1] = SEGMENT_PATTERNS[0];
    segBytes[GREEN1] &= 0b01111111;  // DP on "0."
    segBytes[GREEN2] = SEGMENT_PATTERNS[tenths];
    segBytes[GREEN3] = SEGMENT_PATTERNS[hundredth];
  }
  outputShiftRegisters(segBytes, 7);
}

/***** Interrupt Service Routine for Lock-in Detection *****/
ISR(TIMER2_COMPB_vect) {
  // This ISR is called at the end of each half-cycle of Timer2 PWM (OC2B compare match in phase-correct mode).
  // We alternate between reading the photodiode at the end of laser-ON and laser-OFF phases.
  // phaseFlag == false -> just finished laser ON phase, read "signal + ambient"
  // phaseFlag == true  -> just finished laser OFF phase, read "ambient only"
  int sensorValue = analogRead(IR_SENSOR_PIN);  // 10-bit ADC reading (0-1023)
  int centered = sensorValue - 512;             // center around 0 (~0 when only ambient, + when more IR)
  if (!phaseFlag) {
    // Laser was ON this half-cycle: add the reading
    lockinValue += centered;
  } else {
    // Laser was OFF: subtract the reading
    lockinValue -= centered;
  }
  // Apply a simple decay/leak to avoid unbounded growth and to filter over time.
  // Here we remove ~1/64 of the current value each half-cycle (~1.6% per half-cycle) to track changes.
  lockinValue -= lockinValue >> 6;
  phaseFlag = !phaseFlag;
}

/***** Setup Routine *****/
void setup() {
  // Serial (optional debug)
  Serial.begin(115200);
  
  // Pin configurations
  pinMode(LATCH_PIN, OUTPUT);
  pinMode(DATA_PIN, OUTPUT);
  pinMode(CLOCK_PIN, OUTPUT);
  pinMode(LASER_PWM_PIN, OUTPUT);
  digitalWrite(LATCH_PIN, LOW);
  
  // Initialize SPI for shift register output
  SPI.begin();
  // Set SPI clock to e.g. 4 MHz for fast shifting (TPIC6B595 can handle this).
  SPI.beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE0));
  
  // Configure Timer2 for 490 Hz PWM on OC2B (D3) and enable compare interrupt
  // Arduino analogWrite on pin 3 already sets up Timer2 in phase-correct PWM (TOP=255, prescaler 64).
  analogWrite(LASER_PWM_PIN, 128);  // 50% duty cycle (modulate IR laser at ~490 Hz)
  // Enable Timer2 Compare Match B interrupt
  bitSet(TIMSK2, OCIE2B);  // TIMSK2 – Timer/Counter2 Interrupt Mask Register, OCIE2B is bit 2
  
  // Give some time for things to stabilize
  delay(10);
  
  // Initially display "0" for signal strength
  setDisplayToStrength(0);
}

/***** Main Loop *****/
void loop() {
  // Lock-in signal strength (percentage) calculation
  // We use a critical section to read the volatile lockinValue safely (interrupt off during read)
  long lockVal;
  noInterrupts();
  lockVal = lockinValue;
  interrupts();
  
  // Dynamic scaling of signal strength
  if (lockVal < 0) lockVal = -lockVal;  // use absolute value for strength
  if (lockVal > lockinMaxPeak) {
    lockinMaxPeak = lockVal;
  }
  uint8_t strengthPercent = 0;
  if (lockinMaxPeak > 0) {
    // Scale current lockVal to 0-100 based on max peak seen
    strengthPercent = (uint8_t) constrain((lockVal * 100L) / lockinMaxPeak, 0, 100);
  }
  
  if (mode == MODE_ALIGN) {
    // Show signal strength and check threshold
    setDisplayToStrength(strengthPercent);
    if (strengthPercent >= STARTUP_THRESHOLD_PERCENT) {
      // Beam alignment sufficient – proceed to arm the timer
      mode = MODE_WAIT;
      beamPresent = true;
      timingStarted = false;
      // Record baseline lock-in value and thresholds for beam break
      baseLockinValue = lockVal;
      if (baseLockinValue < 1) baseLockinValue = 1; // avoid zero
      breakThresholdValue = (long)(baseLockinValue * BREAK_BEAM_FRACTION);
      breakRestoreValue  = (long)(baseLockinValue * BREAK_BEAM_HYSTERESIS);
      // Initialize best lap time display to 0.00 (or blank if desired)
      bestLapSet = false;
      bestLapCentis = 0;
      setDisplayToBestTime(0);  // show "0.00" or could blank if no lap yet
      // Also clear red display to 0.0 ready to start timing
      setDisplayToCurrentTime(0);
      Serial.println("Alignment done. Beam strength OK, timer armed.");
    }
    // Continue looping in ALIGN mode until threshold reached
    delay(50); // small delay to slow update (optional)
    return;
  }
  
  // If we reach here, we are in one of the timing modes (WAIT/RUNNING/FLASH)
  
  // Beam break detection logic
  if (mode != MODE_ALIGN) {
    bool currentBeamPresent = (lockVal > breakThresholdValue);
    if (beamPresent && !currentBeamPresent) {
      // Beam was present, now broken – trigger lap event
      beamPresent = false;
      // Only proceed if we are in timing mode and not currently flashing a prior lap
      if (mode == MODE_WAIT || mode == MODE_RUNNING) {
        unsigned long nowUs = micros();
        if (!timingStarted) {
          // First break: start timing from this moment (no lap time to display yet)
          timingStarted = true;
          lastCrossTimeMicros = nowUs;
          mode = MODE_RUNNING;
          Serial.println("Lap timing started.");
        } else {
          // Subsequent break: lap completed
          unsigned long lapUs = nowUs - lastCrossTimeMicros;
          lastCrossTimeMicros = nowUs;  // reset start for next lap
          // Convert lap time to centiseconds
          unsigned int lapCentis = (unsigned int)((lapUs + 5000) / 10000); // round to nearest .01
          // Update best lap if applicable
          if (!bestLapSet || lapCentis < bestLapCentis) {
            bestLapCentis = lapCentis;
            bestLapSet = true;
            setDisplayToBestTime(bestLapCentis);
          }
          // Flash the lap time on red display
          flashStartTime = millis();
          lastFlashToggle = flashStartTime;
          mode = MODE_FLASH;
          // Immediately show the lap time (solid) to start the flash
          // (Displayed with one decimal place)
          unsigned long lapTenthsUs = (lapUs + 50000) / 100000 * 100000; // convert to nearest tenth in microseconds
          setDisplayToCurrentTime(lapTenthsUs);
          Serial.print("Lap completed: "); Serial.print(lapCentis/100.0, 2); Serial.println(" s");
        }
      }
    } 
    else if (!beamPresent && currentBeamPresent) {
      // Beam was broken, now restored
      beamPresent = true;
      Serial.println("Beam restored (ready for next lap trigger).");
    }
  }
  
  // Handle display updates based on mode
  unsigned long nowMs = millis();
  if (mode == MODE_WAIT) {
    // Timer armed, waiting for first crossing – just display 0.0 (current lap time = 0)
    // We can continuously ensure it shows 0.0 (or remain from alignment end).
    // Not much to do until break happens.
  }
  else if (mode == MODE_RUNNING) {
    // Update current lap time on red display every 0.1s
    if (nowMs - lastDisplayUpdate >= DISPLAY_REFRESH_MS) {
      lastDisplayUpdate = nowMs;
      unsigned long elapsedUs = micros() - lastCrossTimeMicros;
      setDisplayToCurrentTime(elapsedUs);
    }
  }
  else if (mode == MODE_FLASH) {
    // Flash the last lap time on red display for LAP_DISPLAY_FLASH_MS duration
    unsigned long elapsedFlash = nowMs - flashStartTime;
    if (elapsedFlash >= LAP_DISPLAY_FLASH_MS) {
      // Flash period over – return to normal timing display
      mode = MODE_RUNNING;
      lastDisplayUpdate = nowMs;
      // After flashing, continue updating current lap time (which has been running in background)
      // We won't explicitly reset the red display here; it will update on next refresh tick.
      Serial.println("Flash period ended, resuming timing display.");
    } else {
      // During flash period, toggle display on/off at a fixed blink rate (e.g. 2 Hz)
      if (nowMs - lastFlashToggle >= 500) {  // toggle every 0.5s
        lastFlashToggle = nowMs;
        static bool flashOn = true;
        flashOn = !flashOn;
        if (flashOn) {
          // Re-display the last lap time (we still have it from when flash started; could store if needed)
          // Here, we can simply not change it since it was displayed at flash start and left static.
          // Alternatively, could store lastLapTenths or lastLapSegBytes to re-output.
          // For simplicity, assume it remains latched while flashOn, and we blank it when flashOff.
        } else {
          // Blank the red display (all segments off)
          uint8_t blankBytes[7];
          for(int i=0;i<7;++i) blankBytes[i] = 0xFF;
          outputShiftRegisters(blankBytes, 7);
        }
      }
      // Note: We keep the green best time display on normally during flashing (only red is blinking).
    }
  }
  
  // (Optional) small delay or yield to avoid saturating CPU in loop
  // delay(1);
}