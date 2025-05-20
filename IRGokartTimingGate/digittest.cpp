#include <Arduino.h>
// Test each segment on 7 TPIC6B595N (7×8 outputs = 56 LEDs)
// hooked in series, driving 7×7-segment displays.
// 
// Pin mapping on the Nano:
const int dataPin  = 11;  // SER_IN
const int clockPin = 13;  // SRCK (shift clock)
const int latchPin = 10;  // RCK  (register clock)

#define NUM_CHIPS      7
#define OUTPUTS_PER_CHIP 8
#define TOTAL_OUTPUTS (NUM_CHIPS * OUTPUTS_PER_CHIP)

void setup() {
  pinMode(dataPin, OUTPUT);
  pinMode(clockPin, OUTPUT);
  pinMode(latchPin, OUTPUT);
}

void loop() {
  // Loop once over every bit in the entire chain
  for (int bitIdx = 0; bitIdx < TOTAL_OUTPUTS; bitIdx++) {
    // Build a zeroed array of bytes, then set the one bit we want
    uint8_t bytes[NUM_CHIPS] = {0};
    int chip    = bitIdx / OUTPUTS_PER_CHIP;       // which chip
    int segment = bitIdx % OUTPUTS_PER_CHIP;       // which output on that chip
    bytes[chip] = (1 << segment);

    // Shift it all out MSB-first, starting with the *last* chip in chain
    digitalWrite(latchPin, LOW);
    for (int i = NUM_CHIPS - 1; i >= 0; i--) {
      shiftOut(dataPin, clockPin, MSBFIRST, bytes[i]);
    }
    digitalWrite(latchPin, HIGH);

    delay(200);  // keep it lit for 100 ms
  }
}
