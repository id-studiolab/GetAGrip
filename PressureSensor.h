// By Richard Bekking
// richard@electronicsdesign.nl
// See PressureSensor.cpp for a explaination of the functionality

#pragma ONCE

#include <Arduino.h>
#include <movingAvg.h>

class PressureSensor {
  public:

    PressureSensor(int outputPin_, int inputPin_, uint16_t clenchThreshold_,
                   void (*fpclench)(uint16_t value), void (*fprelease)(uint16_t value), 
                   int numavg = 20)
      : outputPin_(outputPin_), inputPin_(inputPin_), capSense_(numavg), numAvg_(numavg), isClenching_(false),
        clenchThreshold_(clenchThreshold_), pressureValue_(0), event_clench_(fpclench), event_release_(fprelease) {}
    void begin();
    uint16_t pressure();
    void run();

  private:

    // BEGIN Statemachine stuff
    enum class PressureSensorStates_ {
      Init,                   // Initial state
      DischargeWait,          // Wait untill discharge is finished
    };

    PressureSensorStates_ state_;

    // END Statemachine stuff

    static const unsigned long  CLENCH_HOLD_DELAY = 2000;   // ms (This one is usefull to configure, 
                                                            //  how long should one clench before it's counted as a clench?)
    static const unsigned long  PRESSURE_CHECK_DELAY = 500; // ms (This one determines the resolution of the data)
    static const unsigned long  TIMEOUT_MICROS = 4000;      // Don't touch if you don't fully understand why.
    static const unsigned long  MAXVAL = 200;               // Same as above...
    static const int            CAPTHRESHOLD = 800;         // Same as above... here be monsters...
    
    int outputPin_             = 0;
    int inputPin_              = 0;
    bool     isClenching_      = false;
    uint16_t clenchThreshold_  = 0;
    uint16_t pressureValue_    = 0;
    uint16_t zeroPressureTime_ = 0;
    uint16_t capChargeTime_    = 0;
    int numAvg_                = 0;

    unsigned long start_time_ = 0UL;
    unsigned long stop_time_  = 0UL;
    movingAvg capSense_;

    void (*event_clench_)(uint16_t pressure)  = nullptr;
    void (*event_release_)(uint16_t pressure) = nullptr;
    void discharge_();
};
