#include "PressureSensor.h"

const int PRESSURE_OUTPUT = 3;
const int PRESSURE_INPUT = A0;

// Pressure
const int PROGMEM CLENCH_THRESHOLD = 60;
void on_clench(uint16_t pressure);
void on_release(uint16_t pressure);
PressureSensor pressureSense(PRESSURE_OUTPUT, PRESSURE_INPUT, CLENCH_THRESHOLD, &on_clench, &on_release);

void setup() {

  Serial.begin(115200);
  while (!Serial);
  Serial.println("Initializing..");
  // put your setup code here, to run once:
  pressureSense.begin();
  Serial.println("Initialize Finish");
}

void loop() {
  // put your main code here, to run repeatedly:
  pressureSense.run();
//  Serial.println(analogRead(A0));
}

void on_clench(uint16_t pressure)
{
  // This will be invoked when we detect that the user is clenching his fist
  // for a (configurable) while. Check PressureSensor.h to configure this.
  Serial.print("CLENCH EVENT: ");
  Serial.println(pressure);
}

void on_release(uint16_t pressure)
{
  // This will be invoked when we detect that the user is releasing his clenched
  // fist.
  Serial.print(F("CLENCH RELEASE EVENT: "));
  Serial.println(pressure);
}
