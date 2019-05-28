#include "pressureSensor.h"

void PressureSensor::begin()
{
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
  delay(1);
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
  static unsigned long pressureMillis = 0;
  static unsigned long prev_pressureMillis = 0;

  pressureMillis = millis();
  if (pressureMillis - prev_pressureMillis > PRESSURE_CHECK_DELAY) {
    uint16_t pressure_val = pressure();
    
    if (clench_flag == true) {
      if (pressure_val <= clenchThreshold) {
        clench_flag = false;
        event_release(pressure());
      }
    }
    else {
      if (pressure_val > clenchThreshold) {
        clench_flag = true;
        event_clench(pressure());
      }
    }
  }
}
