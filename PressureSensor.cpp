#include "pressureSense.h"

void PressureSensor::begin()
{
  _capSense.begin(); // Moving average
  discharge();

  // Fill average buffer
  for (int i = 0; i < _numAvg; ++i) {
    unsigned long start_time = micros(); // Start timer
    digitalWrite(_outputpin, HIGH);
    int capval = analogRead(_inputpin);
    static int avg;
    while (capval < 800) {
      capval = analogRead(_inputpin);
      if (micros() - start_time > TIMEOUT_MICROS) break;
    }
    unsigned long stop_time = micros();
    _capChargeTime = _capSense.reading(stop_time - start_time);
    discharge();
    _zeroPressureTime = _capChargeTime;
  }
}

void PressureSensor::discharge()
{
  pinMode(_outputpin, OUTPUT);
  digitalWrite(_outputpin, LOW);
  pinMode(_inputpin, OUTPUT);
  digitalWrite(_inputpin, LOW);
  delay(1);
  pinMode(_inputpin, INPUT);
}

int PressureSensor::getPressure()
{
  int retval = 0;
  unsigned long start_time = micros(); // Start timer
  digitalWrite(_outputpin, HIGH);
  int capval = analogRead(_inputpin);
  static int avg;
  while (capval < 800) {
    capval = analogRead(_inputpin);
    if (micros() - start_time > TIMEOUT_MICROS) break;
  }
  unsigned long stop_time = micros();
  _capChargeTime = _capSense.reading(stop_time - start_time);
  discharge();
  retval = _capChargeTime - _zeroPressureTime;

  if (retval < 0) retval = 0;
  else if (retval > MAXVAL) retval = MAXVAL;

  return retval;
}
