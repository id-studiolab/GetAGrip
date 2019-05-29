// By Richard Bekking
// richard@electronicsdesign.nl
// See PressureSensor.cpp for a explaination of the functionality

#pragma ONCE

#include <Arduino.h>
#include <movingAvg.h>

class PressureSensor {
  public:

    PressureSensor(int outputpin, int inputpin, uint16_t clenchThreshold, 
                   void (*fpclench)(uint16_t value), void (*fprelease)(uint16_t value), 
                   int numavg = 40)
      : outputpin(outputpin), inputpin(inputpin), capSense(numavg), numAvg(numavg), isClenching(false),
        clenchThreshold(clenchThreshold), event_clench(fpclench), event_release(fprelease) {}
    void begin();
    uint16_t pressure();
    void run();

  private:

    static const unsigned long  CLENCH_HOLD_DELAY = 2000;   // ms (This one is usefull to configure, 
                                                            //  how long should one clench before it's counted as a clench?)
    static const unsigned long  PRESSURE_CHECK_DELAY = 500; // ms (This one determines the resolution of the data)
    static const unsigned long  TIMEOUT_MICROS = 4000;      // Don't touch if you don't fully understand why.
    static const unsigned long  MAXVAL = 200;               // Same as above...
    static const int            CAPTHRESHOLD = 800;         // Same as above... here be monsters...
    
    int outputpin;
    int inputpin;

    bool     isClenching;
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
