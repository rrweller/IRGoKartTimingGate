#include <Arduino.h>

// ----------------------------------------
// User Settings
// ----------------------------------------
static const uint16_t N = 64;             // Number of samples in each capture
static const double FS = 4000.0;          // Sampling rate (Hz)
static const double FREQ = 490.0;         // Target frequency (Hz)
static const double THRESHOLD = 550.0;   // Amplitude threshold (adjust experimentally)

// Pin assignments
static const int PD_PIN      = A0;        // Photodiode amplifier input
static const int LED_PIN     = 7;
static const int LASER_PIN   = 3;         // Laser / IR LED drive, ~490 Hz by default PWM

// We'll create lookup tables for the reference signals
double cosRef[N];
double sinRef[N];

void setup() {
  Serial.begin(9600);
  pinMode(LED_PIN, OUTPUT);
  pinMode(LASER_PIN, OUTPUT);

  // If we want a ~490 Hz drive, we can just do:
  analogWrite(LASER_PIN, 127); 
  // On many Arduinos, pin 3 uses a default PWM freq ~490 Hz.

  // Precompute our reference sine and cosine for one "analysis window" of length N
  // We want the reference at frequency = FREQ
  //   angle[n] = 2*pi*(FREQ/FS)*n
  for (uint16_t i = 0; i < N; i++) {
    double angle = 2.0 * M_PI * (FREQ / FS) * (double)i;
    cosRef[i] = cos(angle);
    sinRef[i] = sin(angle);
  }

  Serial.println("Single-Frequency Lock-In Example");
  Serial.print("N = "); Serial.println(N);
  Serial.print("FS = "); Serial.println(FS);
  Serial.print("Window Duration = "); 
  Serial.print(1000.0 * (double)N / FS); 
  Serial.println(" ms");
}

void loop() {
  // 1) Collect N samples at FS
  // We'll do a simple blocking approach, reading N samples in one window.
  static const unsigned long microsPerSample = (unsigned long)(1000000.0 / FS);

  double sumI = 0.0;  // In-phase accumulation
  double sumQ = 0.0;  // Quadrature accumulation

  for (uint16_t i = 0; i < N; i++) {
    unsigned long tStart = micros();

    // Read the photodiode signal: raw 0..1023
    int adcVal = analogRead(PD_PIN);

    // Convert to double. 
    // In a real lock-in you might shift/scale, but let's keep it simple.
    double x = (double)adcVal; 

    // 2) Multiply by reference signals and accumulate
    sumI += x * cosRef[i];
    sumQ += x * sinRef[i];

    // Wait until the sample period is done
    while (micros() - tStart < microsPerSample) {
      // do nothing
    }
  }

  // 3) Compute amplitude
  double amplitude = sqrt(sumI * sumI + sumQ * sumQ);

  // 4) Compare to threshold & turn LED on/off
  bool beamPresent = (amplitude > THRESHOLD);
  digitalWrite(LED_PIN, beamPresent ? HIGH : LOW);

  // 5) Print the amplitude for debugging
  Serial.print("Amplitude: ");
  Serial.print(amplitude);
  Serial.print("  => beamPresent: ");
  Serial.println(beamPresent ? "YES" : "NO");

  // Small delay or none, depending on how often you want updates
  delay(10);
}
