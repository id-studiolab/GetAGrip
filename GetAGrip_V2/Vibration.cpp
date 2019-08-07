// By Richard Bekking
// richard@electronicsdesign.nl

#include "Vibration.h"

uint16_t VibrationElement::next()
{
  uint16_t retval = 0;

  if (repeat_count == repeat_max) {
    repeat_count = 0;
    return -1;
  }
  else {
    retval = calc_next_value(step_index++, step_max);
  }

  if (step_index == step_max) {
    step_index = 0;
    ++repeat_count;
  }

  return retval;
}

void VibrationElement::reset()
{
  step_index = 0;
  repeat_count = 0;
}

uint16_t VibrationElement::stepTime()
{
  return step_time_ms;
}

bool Vibration::ready()
{
  time_millis = millis();
  if (time_millis - prev_time_millis >= elements[element_index].stepTime()) {
    prev_time_millis = time_millis;

    return true;
  }

  return false;
}

void Vibration::append(VibrationElement element)
{
  if (num_elements < MAX_ELEMENTS) {
    elements[num_elements++] = element;
  } else {
  }
}

uint16_t Vibration::next()
{
  uint16_t retval = 0;

  if (element_index == num_elements) {
    element_index = 0;
    return -1;
  }

  retval = elements[element_index].next();

  if (retval == uint16_t(-1)) {
    ++element_index;
    retval = next();
  }

  return retval;
}

void Vibration::reset()
{
  analogWrite(pin_number, 0);
  element_index = 0;
  finished_flag = false;
  time_millis = millis();
  prev_time_millis = time_millis;
  for (int i = 0; i < num_elements; ++i) {
    elements[i].reset();
  }
}

void Vibration::go()
{
  uint16_t nextval = 0;

  if (ready()) {
    nextval = next();
    if (nextval == END_OF_SEQUENCE) {
      analogWrite(pin_number, 0);
      finished_flag = true;
    }
    else {
      analogWrite(pin_number, nextval);
      //Serial.println(nextval);
    }
  }
}

bool Vibration::finished()
{
  return finished_flag;
}
