#include <Arduino.h>
const int LED = 13;  // from the circuit we can see that we have connected LED on Pin 13


void setup() 
{                
    pinMode(LED, OUTPUT); // Defining LED pin as OUTPUT Pin.   
}

// Below mentioned code runs for ever(infinite loop)
void loop() {
  digitalWrite(LED, HIGH); // LED gets turned ON (1/HIGH/+5V)
  delay(1000);             // Waiting for one second. 
  digitalWrite(LED, LOW);  // LED gets OFF (0/LOW/0V/GND)
  delay(1000);        // here and above Delay is in mili second (1000 = 1 second)
}