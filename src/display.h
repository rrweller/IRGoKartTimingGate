#ifndef DISPLAY_H
#define DISPLAY_H

#include <Arduino.h>

namespace DisplayDriver {
  void begin();

  void showStrength(uint8_t percent);               // 0-100  (RED)
  void showCurrentTime(unsigned long microsElapsed); // SS.t   (RED)
  void showBestTime(unsigned int centis);           // SS.hh  (GREEN)

  void showReady(bool on);                          // --- blink on/off (RED)
  void showBanner(const char txt[3]);               // --- show 3 letters
  void blankRedDigits();                            // blanks red only
  void blankAll();                                  // blanks all digits
}

#endif
