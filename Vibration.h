// By Richard Bekking
// richard@electronicsdesign.nl

#pragma ONCE

#include <Arduino.h>

// Make sure to append .0 to the number below:
// e.g. 128 becomes 128.0
#define VIBRATION_MAX 255.0

#define END_OF_SEQUENCE uint16_t(-1)

class VibrationElement  {
  public:

    VibrationElement() : calc_next_value(nullptr), step_time_ms(0) {};
    VibrationElement(uint16_t steps, uint16_t step_delay, uint16_t repeat,  uint16_t (*fp)(uint16_t, uint16_t))
      : step_index(0), step_max(steps), step_time_ms(step_delay), repeat_max(repeat), repeat_count(0), calc_next_value(fp) {};

    uint16_t next();
    void reset();
    uint16_t stepTime();

  private:

    uint16_t step_time_ms;      // Time in ms between each step
    uint16_t step_index;        // Keeps track of our progress
    uint16_t step_max;          // Number of steps
    uint16_t repeat_max;        // Repeat the whole cycle this many times
    uint16_t repeat_count;      // Keeps track of how many repetitions we've done
    // Function pointer to the algorithm
    uint16_t (*calc_next_value)(uint16_t x, uint16_t max_x);
};

class Vibration {
  public:

    Vibration(uint16_t pin) : pin_number(pin), element_index(0), num_elements(0), finished_flag(false) {}
    void append(VibrationElement element);
    void go();          // Do the buisness
    bool finished();    // Signals when buisness is done
    void reset();

  private:

    enum constants {MAX_ELEMENTS = 10};

    uint16_t pin_number;        // Output for the vibration motor
    bool ready();
    uint16_t next();
    // Could be factorised into millitimer library
    unsigned long time_millis;
    unsigned long prev_time_millis;
    // -------------------------------------------
    uint16_t element_index;
    uint16_t num_elements;
    bool finished_flag;
    VibrationElement elements[MAX_ELEMENTS];
};
