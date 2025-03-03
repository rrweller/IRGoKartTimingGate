#include <Arduino.h>

/*** Arduino Nano 1kHz Laser PWM and Synchronous Detector ***/

// Pin assignments (Arduino Nano ATmega328P):
const byte LASER_PIN = 3;      // OC0A pin, but we will toggle manually for precise phase
const byte PHOTO_PIN = A0;     // analog input from photodiode amplifier output

// Detection parameters:
const unsigned long HALF_PERIOD_US = 500;   // half cycle = 500 µs for 1kHz
const int DETECTION_THRESHOLD = 5;          // threshold on averaged difference (in ADC units)
const int AVERAGE_CYCLES = 100;             // number of cycles to average over for confidence

// Variables for running average calculation:
long cumulativeDifference = 0;
int cycleCount = 0;
bool signalDetected = false;
int confidenceValue = 0;

void setup() {
  pinMode(LASER_PIN, OUTPUT);
  digitalWrite(LASER_PIN, LOW);
  analogReference(DEFAULT);  // using 5V as ADC reference
  Serial.begin(115200);
  Serial.println("Starting synchronous detection...");
}

void loop() {
  // 1. Turn laser ON
  digitalWrite(LASER_PIN, HIGH);
  unsigned long tStart = micros();
  // Small delay to allow photodiode/amp to respond (choose ~half of half-period)
  delayMicroseconds(200);  
  // 2. Sample photodiode during laser ON
  int readingOn = analogRead(PHOTO_PIN);
  
  // 3. Wait until half-period is completed (500 µs since tStart)
  while (micros() - tStart < HALF_PERIOD_US) {
    // busy-wait loop (could do nothing or a small NOP) 
  }
  // 4. Turn laser OFF
  digitalWrite(LASER_PIN, LOW);
  // 5. Wait ~200 µs for amp to settle after turning off
  delayMicroseconds(200);
  // 6. Sample photodiode during laser OFF
  int readingOff = analogRead(PHOTO_PIN);
  
  // Calculate difference between ON and OFF readings
  int diff = readingOn - readingOff;
  
  // 7. Accumulate for running average over AVERAGE_CYCLES cycles
  cumulativeDifference += diff;
  cycleCount++;
  if (cycleCount >= AVERAGE_CYCLES) {
    // Compute average difference over the last N cycles
    float avgDiff = float(cumulativeDifference) / AVERAGE_CYCLES;
    // Reset counters for next average window (optional; we could do a sliding window instead)
    cumulativeDifference = 0;
    cycleCount = 0;
    
    // 8. Determine detection flag and confidence
    if (avgDiff > DETECTION_THRESHOLD) {
      signalDetected = true;
    } else {
      signalDetected = false;
    }
    confidenceValue = (int)avgDiff;  // use the average difference as a simple confidence metric
    // Optionally, scale confidenceValue to 0-100 or 0-255 for output, or compute SNR, etc.
    
    // 9. Output results (here we print to serial; in practice this could set a digital output or send to PC)
    Serial.print("avgDiff = ");
    Serial.print(avgDiff, 2);
    Serial.print(" -> Detected: ");
    Serial.print(signalDetected ? "YES" : "no");
    Serial.print(", Confidence = ");
    Serial.println(confidenceValue);
  }
  
  // 10. Complete the second half-period timing (ensure total cycle ~1000 µs)
  // Since we already waited 500 µs for first half, and did some work, calculate remaining time:
  unsigned long tCycle = micros() - tStart;
  if (tCycle < 2 * HALF_PERIOD_US) {
    delayMicroseconds((2 * HALF_PERIOD_US) - tCycle);
  }
  // Loop repeats for next cycle
}
