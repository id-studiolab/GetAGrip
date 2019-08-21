#include "pressureSensor.h"

// It works like this: The sensor is basically two conductive services separated by a thin layer of foam, forming
// a capacitor. This capacitor is paralleled with a small capacitor of aprox. the same capacitance value. First we
// discharge both capacitors through a digital output set to LOW. Secondly we charge both capacitors through a
// high-valued resistor and check the analog input connected directly to the two capacitor. Both capacitors will
// slowly charge and we keep time on how long it takes. When pressure is applied the capacitance of the sensor will
// increase, thus adding to the charge time. The time it takes to charge to a certain level is the output value of
// pressure().
// Makes sense? No?
// call or e-mail me: 0630397999, richard@electronicsdesign.nl

void PressureSensor::begin()
{
  // Calibrates the sensor at startup

  capSense.begin(); // Moving average
  discharge();

  // Fill average buffer
  for (int i = 0; i < numAvg; ++i) {
    unsigned long start_time = micros(); // Start timer
    digitalWrite(outputpin, HIGH);
    int capval = analogRead(inputpin);
    static int avg;
    while (capval < CAPTHRESHOLD) {
      capval = analogRead(inputpin);
      if (micros() - start_time > TIMEOUT_MICROS) break;
    }
    unsigned long stop_time = micros();
    capChargeTime = capSense.reading(stop_time - start_time);
    discharge();
    zeroPressureTime = capChargeTime;
  }
}

void PressureSensor::discharge()
{
  pinMode(outputpin, OUTPUT);
  digitalWrite(outputpin, LOW);
  pinMode(inputpin, OUTPUT);
  digitalWrite(inputpin, LOW);
  delay(100);
  pinMode(inputpin, INPUT);
}

uint16_t PressureSensor::pressure()
{
  int retval = 0;
  unsigned long start_time = micros(); // Start timer
  digitalWrite(outputpin, HIGH);
  int capval = analogRead(inputpin);
  static int avg;
  while (capval < CAPTHRESHOLD) {
    capval = analogRead(inputpin);
    if (micros() - start_time > TIMEOUT_MICROS) break;
  }
  unsigned long stop_time = micros();
  capChargeTime = capSense.reading(stop_time - start_time);
  discharge();
  retval = capChargeTime - zeroPressureTime;

  if (retval < 0) retval = 0;
  else if (retval > MAXVAL) retval = MAXVAL;

  return retval;
}

void PressureSensor::run()
{
  static bool clench_initiated = false;
  static bool first_clench_detected = false;
  static uint16_t prev_pressure_val = 0;
  static unsigned long clenchStartMillis = 0;
  static unsigned long pressureMillis = 0;
  static unsigned long prev_pressureMillis = 0;

  pressureMillis = millis();
  if (pressureMillis - prev_pressureMillis > PRESSURE_CHECK_DELAY) {
    uint16_t pressure_val = pressure();

    //Serial.println(pressure_val); // For debug purpose

    if (clench_initiated) { // we're clenching but not long enough to count

      if (pressure_val < prev_pressure_val) { // Declining
        if (pressure_val <= clenchThreshold / 4) {
          clench_initiated = false;
        }
      }

    }
    else {

      if (pressure_val > prev_pressure_val) { // Increasing
        if (pressure_val > clenchThreshold) {
          clench_initiated = true;
          first_clench_detected = true;
          clenchStartMillis = millis();
        }
      }

    }

    if (first_clench_detected && clench_initiated && (millis() - clenchStartMillis > CLENCH_HOLD_DELAY)) {
      first_clench_detected = false;
      isClenching = true;
      event_clench(pressure()); // Call clench event handler
    }
    else if (isClenching && !clench_initiated) {
      first_clench_detected = false;
      isClenching = false;
      event_release(pressure()); // Call release event handler
    }

    prev_pressure_val = pressure_val;
  }
}
