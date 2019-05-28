// By Richard Bekking
// richard@electronicsdesign.nl

#define USE_ARDUINO_INTERRUPTS true
#include <PulseSensorPlayground.h>
#include <AceButton.h>
#include "Fsm.h"
#include "EventQueue.h"
#include "PressureSensor.h"
#include "Vibration.h"
#include "MillisTimer.h"

using namespace ace_button;

// PIN definitions
const int BUTTONGREEN_PIN = 2;
const int BUTTONBLACK_PIN = 3;
const int BUTTONWHITE_PIN = 6;
const int BUTTONBLUE_PIN  = 7;
const int BUTTONRED_PIN  = A2;
const int MOTOR_PIN = 5;
const int HEARTRATE_PIN = A3;
const int PRESSURE_OUTPUT = 4;
const int PRESSURE_INPUT  = A0;

// Buttons
void handleButtonEvent(AceButton*, uint8_t, uint8_t);
AceButton button_green(BUTTONGREEN_PIN);
AceButton button_black(BUTTONBLACK_PIN);
AceButton button_white(BUTTONWHITE_PIN);
AceButton button_blue(BUTTONBLUE_PIN);
AceButton button_red(BUTTONRED_PIN);

// State machine
const int STRESS_DETECTED = 1;
const int CHALLENGE_BUTTON_ACTIVATED = 2;
const int INACTIVITY_DETECTED = 3;
const int CHALLENGE_DETECTED = 4;
const int CLENCH_ACTIVATED = 5;
const int CLENCH_DEACTIVATED = 6;
const int STRESSALARM_TIMEOUT   = 7;
const int CHALLENGEALARM_TIMEOUT = 8;
const int INACTIVITYALARM_TIMEOUT = 9;

void on_standby();
void on_standby_enter();
void on_challenge_enter();
void on_selfreport();
void on_selfreport_enter();
void on_stressalarm_enter();
void on_stressalarm_exit();
void on_challengealarm_enter();
void on_challengealarm_exit();
void on_inactivityalarm_enter();
void on_inactivityalarm_exit();
void check_triggers();
State state_standby(&on_standby_enter, &on_standby, NULL);
State state_challenge(&on_challenge_enter, &on_challenge, &on_challenge_exit);
State state_selfreport(&on_selfreport_enter, &on_selfreport, NULL);
State state_stressalarm(&on_stressalarm_enter, &on_stressalarm, &on_stressalarm_exit);
State state_challengealarm(&on_challengealarm_enter, &on_challengealarm, &on_challengealarm_exit);
State state_inactivityalarm(&on_inactivityalarm_enter, &on_inactivityalarm, &on_inactivityalarm_exit);
Fsm fsm_main(&state_standby);
EventQueue events;

// Pressure
const int CLENCH_THRESHOLD = 100;
void on_clench(uint16_t pressure);
void on_release(uint16_t pressure);
PressureSensor pressureSense(PRESSURE_OUTPUT, PRESSURE_INPUT, CLENCH_THRESHOLD, &on_clench, &on_release);

// Heartrate
const int HEARTRATE_THRESHOLD = 550;
PulseSensorPlayground pulseSensor;

// Vibration
uint16_t vib_full(uint16_t x, uint16_t max_x);      // Constant vibration
uint16_t vib_rampup(uint16_t x, uint16_t max_x);    // Linear ramp up
uint16_t vib_rampdown(uint16_t x, uint16_t max_x);  // Linear ramp down
VibrationElement constZero    (1,   500, 1,  &vib_zero);     //  _______ <- 0%
VibrationElement constHalf    (1,   1500, 1, &vib_half);     //  ------- <- 50%
VibrationElement constFull    (1,   1500, 1, &vib_full);     //  ------- <- 100%
VibrationElement rampUp       (100, 30,  1,  &vib_rampup);   //
VibrationElement sawtoothUp   (100, 2,   5,  &vib_rampup);   //
VibrationElement rampDown     (100, 10,  1,  &vib_rampdown); //
VibrationElement sawtoothDown (100, 2,   5,  &vib_rampdown); //
VibrationElement piramid      (100, 5,  10,  &vib_piramid);
Vibration vibStress(MOTOR_PIN);
Vibration vibChallenge(MOTOR_PIN);
Vibration vibInactive(MOTOR_PIN);

// Data acquisition
void telemetryTimer_handler(MillisTimer &mt);
MillisTimer telemetryTimer = MillisTimer(5000); // Take a snapshot of all the date once every 5 seconds


void setup() {
  pinMode(BUTTONBLACK_PIN, INPUT);
  pinMode(BUTTONGREEN_PIN, INPUT);
  pinMode(BUTTONWHITE_PIN, INPUT);
  pinMode(BUTTONBLUE_PIN, INPUT);
  pinMode(BUTTONRED_PIN, INPUT);
  pinMode(MOTOR_PIN, OUTPUT);
  digitalWrite(MOTOR_PIN, LOW);

  Serial.begin(115200);
  Serial.println("Arduino started");

  // MOCK signals (should be coming over BlueTooth,
  // being faked by buttons at this moment)
  button_black.init(BUTTONBLACK_PIN, HIGH, 0);
  button_black.setEventHandler(handleButtonEvent);
  button_green.init(BUTTONGREEN_PIN, LOW, 0);
  button_green.setEventHandler(handleButtonEvent);
  button_white.init(BUTTONWHITE_PIN, LOW, 0);
  button_white.setEventHandler(handleButtonEvent);
  button_blue.init(BUTTONBLUE_PIN, LOW, 0);
  button_blue.setEventHandler(handleButtonEvent);
  button_red.init(BUTTONRED_PIN, LOW, 0);
  button_red.setEventHandler(handleButtonEvent);

  // Build vibration patterns out of elements
  vibStress.append(rampUp);
  vibStress.append(rampDown);
  vibStress.append(piramid);
  vibChallenge.append(constFull);
  vibChallenge.append(constHalf);
  vibInactive.append(rampDown);

  // Pressure sensor
  pressureSense.begin();

  // Heartrate sensor
  pulseSensor.analogInput(HEARTRATE_PIN);
  pulseSensor.setThreshold(HEARTRATE_THRESHOLD);
  if (!pulseSensor.begin()) {
    Serial.println("Heartrate sensor failed!");
    delay(2000);
  }

  // Data acquisition
  telemetryTimer.setInterval(5000);
  telemetryTimer.expiredHandler(telemetryTimer_handler);
  telemetryTimer.setRepeats(0); // 0 = Forever
  telemetryTimer.start();


  // Finite State Machine transition table
  fsm_main.add_transition(&state_standby, &state_stressalarm,
                          STRESS_DETECTED,
                          NULL);
  fsm_main.add_transition(&state_stressalarm, &state_standby,
                          STRESSALARM_TIMEOUT,
                          NULL);
  fsm_main.add_transition(&state_stressalarm, &state_selfreport,
                          CLENCH_ACTIVATED,
                          NULL);
  fsm_main.add_transition(&state_standby, &state_selfreport,
                          CLENCH_ACTIVATED,
                          NULL);
  fsm_main.add_transition(&state_selfreport, &state_standby,
                          CLENCH_DEACTIVATED,
                          NULL);
  fsm_main.add_transition(&state_standby, &state_challengealarm,
                          CHALLENGE_DETECTED,
                          NULL);
  fsm_main.add_transition(&state_challengealarm, &state_standby,
                          CHALLENGEALARM_TIMEOUT,
                          NULL);
  fsm_main.add_transition(&state_challengealarm, &state_challenge,
                          CHALLENGE_BUTTON_ACTIVATED,
                          NULL);
  fsm_main.add_transition(&state_standby, &state_challenge,
                          CHALLENGE_BUTTON_ACTIVATED,
                          NULL);
  fsm_main.add_transition(&state_challenge, &state_standby,
                          CHALLENGE_BUTTON_ACTIVATED,
                          NULL);
  fsm_main.add_transition(&state_standby, &state_inactivityalarm,
                          INACTIVITY_DETECTED,
                          NULL);
  fsm_main.add_transition(&state_inactivityalarm, &state_standby,
                          INACTIVITYALARM_TIMEOUT,
                          NULL);
}

void loop() {
  // All objects invoke callbacks so the whole program is event-driven
  fsm_main.run_machine(); 
  telemetryTimer.run();
  pressureSense.run();
  button_black.check();
  button_green.check();
  button_white.check();
  button_blue.check();
  button_red.check();
}

void telemetryTimer_handler(MillisTimer &mt)
{
  int bpm = pulseSensor.getBeatsPerMinute();  // Calls function on our pulseSensor object that returns BPM as an "int".
  Serial.print("Heartbeat: "); Serial.print(bpm); Serial.println(" BPM");

//  if (bpm > HEARTRATE_THRESHOLD) {
//    events.push(STRESS_DETECTED);
//  }
}

void on_clench(uint16_t pressure)
{
  Serial.println("CLENCH EVENT");
  events.push(CLENCH_ACTIVATED);
}

void on_release(uint16_t pressure)
{
  Serial.println("RELEASE EVENT");
  events.push(CLENCH_DEACTIVATED);
}

void handleButtonEvent(AceButton* bt, uint8_t eventType,
                       uint8_t /* buttonState */) {
  static bool toggle = false;

  switch (eventType) {
    case AceButton::kEventPressed:
      switch (bt->getPin()) {
        case BUTTONGREEN_PIN:
          events.push(CHALLENGE_DETECTED);
          break;
        case BUTTONBLACK_PIN:
          if (toggle) {
            Serial.println("CLENCH_DEACTIVATED");
            events.push(CLENCH_DEACTIVATED);
          } else {
            Serial.println("CLENCH_ACTIVATED");
            events.push(CLENCH_ACTIVATED);
          }
          toggle = !toggle;
          break;
        case BUTTONWHITE_PIN:
          events.push(CHALLENGE_BUTTON_ACTIVATED);
          break;
        case BUTTONBLUE_PIN:
          events.push(INACTIVITY_DETECTED);
          break;
        case BUTTONRED_PIN:
          events.push(STRESS_DETECTED);
          break;
      }
      break;
    case AceButton::kEventReleased:
      break;
  }
}

void on_standby() {
  check_triggers();
}

void on_standby_enter() {
  Serial.println("Standby enter");
}

void on_challenge() {
  check_triggers();
}

void on_challenge_enter() {
  Serial.println("Challenge enter");
}

void on_challenge_exit() {
  Serial.println("Challenge exit");
}

void on_selfreport() {
  check_triggers();
}
void on_selfreport_enter() {
  Serial.println("Selfreport enter");
}

void on_stressalarm() {
  vibStress.go();

  if (vibStress.finished()) {
    // Vibration is done
    vibStress.reset(); // Make sure vibration stops and the whole cycle resets
    Serial.println("vibStress is done!");
    events.push(STRESSALARM_TIMEOUT);
  }

  check_triggers();
}

void on_stressalarm_enter() {
  Serial.println("Stressalarm enter");
}

void on_stressalarm_exit() {
  vibStress.reset(); // Stop the vibration in case we were interrupted
  Serial.println("Stressalarm exit");
}

void on_challengealarm()
{
  vibChallenge.go();

  if (vibChallenge.finished()) {
    // Vibration is done
    vibChallenge.reset();
    Serial.println("vibChalenge done!");
    events.push(CHALLENGEALARM_TIMEOUT);
  }

  check_triggers();
}

void on_challengealarm_enter() {
  Serial.println("Challenge alarm enter");
}

void on_challengealarm_exit() {
  vibChallenge.reset(); // Stop the vibration in case we were interrupted
  Serial.println("Challenge alarm exit");
}

void on_inactivityalarm() {
  vibInactive.go();

  if (vibInactive.finished()) {
    // Vibration is done
    vibInactive.reset();
    Serial.println("vibInactive done!");
    events.push(INACTIVITYALARM_TIMEOUT);
  }

  check_triggers();
}

void on_inactivityalarm_enter() {
  Serial.println("Inactivity alarm enter");
}

void on_inactivityalarm_exit() {
  Serial.println("Inactivity alarm exit");
}

void check_triggers()
{
  // Are there any state-changing events?
  // If yes, execute them.
  if (events.state() != EMPTY) {
    fsm_main.trigger(events.pop());
  }
}

uint16_t vib_zero(uint16_t x, uint16_t max_x)
{
  return 0;
}

uint16_t vib_half(uint16_t x, uint16_t max_x)
{
  return VIBRATION_MAX / 2;
}

uint16_t vib_full(uint16_t x, uint16_t max_x)
{
  return VIBRATION_MAX;
}

uint16_t vib_rampup(uint16_t x, uint16_t max_x)
{
  float xstep = VIBRATION_MAX / max_x;
  return xstep * (x + 1);
}

uint16_t vib_rampdown(uint16_t x, uint16_t max_x)
{
  float xstep = VIBRATION_MAX / max_x;
  return VIBRATION_MAX - xstep * x;
}

uint16_t vib_piramid(uint16_t x, uint16_t max_x)
{
  float xstep = VIBRATION_MAX / (max_x / 2);
  if (x < max_x / 2)
  {
    return xstep * (x + 1);
  }
  else {
    return VIBRATION_MAX - xstep * (x - (max_x / 2));
  }
}
