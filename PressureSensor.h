// By Richard Bekking
// richard@electronicsdesign.nl

#pragma ONCE

#include <Arduino.h>
#include <movingAvg.h>

class PressureSensor {
  public:

    PressureSensor(int outputpin, int inputpin, uint16_t clenchThreshold, 
                   void (*fpclench)(uint16_t value), void (*fprelease)(uint16_t value), 
                   int numavg = 20)
      : outputpin(outputpin), inputpin(inputpin), capSense(numavg), numAvg(numavg), clench_flag(false), 
        clenchThreshold(clenchThreshold), event_clench(fpclench), event_release(fprelease) {}
    void begin();
    uint16_t pressure();
    void run();

  private:

    static const unsigned long  PRESSURE_CHECK_DELAY = 500; // ms
    static const unsigned long  TIMEOUT_MICROS = 4000;
    static const unsigned long  MAXVAL = 200;
    static const int            CAPTHRESHOLD = 800;
    
    int outputpin;
    int inputpin;

    bool clench_flag;
    uint16_t clenchThreshold;
    uint16_t pressureValue;
    uint16_t zeroPressureTime;
    uint16_t capChargeTime;
    int numAvg;

    unsigned long start_time;
    unsigned long stop_time;
    movingAvg capSense;

    void (*event_clench)(uint16_t pressure);
    void (*event_release)(uint16_t pressure);
    void discharge();
};
