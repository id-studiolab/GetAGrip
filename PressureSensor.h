#ifndef _PRESSURESENSE_H_
#define _PRESSURESENSE_H_

#include <Arduino.h>
#include <movingAvg.h>

class PressureSensor {
  public:
    static const long TIMEOUT_MICROS = 4000;
    static const long MAXVAL = 200;

    PressureSensor(int outputpin, int inputpin, int numavg = 20) : _outputpin(outputpin), _inputpin(inputpin), _capSense(numavg), _numAvg(numavg) {}
    void begin();
    int getPressure();
    
  private:
  
    int _outputpin;
    int _inputpin;

    int _pressureValue;
    int _zeroPressureTime;
    int _capChargeTime;
    int _numAvg;
    
    unsigned long _start_time;
    unsigned long _stop_time;
    movingAvg _capSense;

    void discharge();
};

#endif
