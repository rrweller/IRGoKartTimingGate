#ifndef DISPLAY_H
#define DISPLAY_H

#include <Arduino.h>

namespace DisplayDriver {

  void begin();                                 // call from setup()

  void showStrength(uint8_t percent);           // 0-100 on RED
  void showCurrentTime(unsigned long micros);   // SS.t  on RED
  void showBestTime(unsigned int centis);       // SS.hh on GREEN

  void blankRedDigits();                        // blanks only the red 3
  void blankAll();                              // blanks all 7 digits (optional)
}

#endif
