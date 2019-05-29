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
const int VIBRATOR_PIN = 5;
const int HEARTRATE_PIN = A3;
const int PRESSURE_OUTPUT = 8;
const int PRESSURE_INPUT  = A0;

// Buttons
void handleButtonEvent(AceButton*, uint8_t, uint8_t);
AceButton button_green(BUTTONGREEN_PIN);  // MOCK SIGNAL: CHALLENGE_DETECTED
AceButton button_white(BUTTONWHITE_PIN);  // This button is actually in the design: CHALLENGE_BUTTON_ACTIVATED
AceButton button_blue(BUTTONBLUE_PIN);    // MOCK SIGNAL: INACTIVITY_DETECTED
AceButton button_red(BUTTONRED_PIN);      // MOCK SIGNAL: STRESS_DETECTED

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
const int CLENCH_THRESHOLD = 60;
void on_clench(uint16_t pressure);
void on_release(uint16_t pressure);
PressureSensor pressureSense(PRESSURE_OUTPUT, PRESSURE_INPUT, CLENCH_THRESHOLD, &on_clench, &on_release);

// Heartrate
const int HEARTRATE_THRESHOLD = 550;
PulseSensorPlayground pulseSensor;

// Vibration
const unsigned long CHALLENGE_VIB_REPETITIONS = 1000;
VibrationElement constZero    (1,   500, 1,  &vib_zero);     //  _______ <- 0%
VibrationElement constHalf    (1,   1500, 1, &vib_half);     //  ------- <- 50%
VibrationElement constFull    (1,   1500, 1, &vib_full);     //  ------- <- 100%
VibrationElement rampUp       (100, 30,  1,  &vib_rampup);   //  goes from zero to MAX (scaled over time)
VibrationElement sawtoothUp   (100, 2,   5,  &vib_rampup);   //  rampup many times
VibrationElement rampDown     (100, 10,  1,  &vib_rampdown); //  goes from MAX to zero (scaled over time)
VibrationElement sawtoothDown (100, 2,   5,  &vib_rampdown); //  rampdown many times
//  example of a combined ramp-up and ramp-down in one function.
//   It's easier to seperate this behaviour in multiple functions for clarity
VibrationElement piramid      (100, 5,  10,  &vib_piramid);  

Vibration vibStressAlarm(VIBRATOR_PIN);     // These vibration patterns are assembled in setup();
Vibration vibChallengeAlarm(VIBRATOR_PIN);
Vibration vibChallenge(VIBRATOR_PIN);
Vibration vibInactiveAlarm(VIBRATOR_PIN);

// Data acquisition
void telemetryTimer_handler(MillisTimer &mt);
MillisTimer telemetryTimer = MillisTimer(5000); // Take a snapshot of all the date once every 5 seconds

////////////////////////////////////////////////////////////////////////////////
//                                                                            //
// SETUP FUNCTION, All initialisations happen here                            //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////
void setup() {
  pinMode(BUTTONGREEN_PIN, INPUT);
  pinMode(BUTTONWHITE_PIN, INPUT);
  pinMode(BUTTONBLUE_PIN, INPUT);
  pinMode(BUTTONRED_PIN, INPUT);
  pinMode(VIBRATOR_PIN, OUTPUT);
  digitalWrite(VIBRATOR_PIN, LOW);

  Serial.begin(115200);
  Serial.println("Arduino started");

  // MOCK signals (should be coming over BlueTooth,
  // being faked by buttons at this moment)
  button_green.init(BUTTONGREEN_PIN, LOW, 0);
  button_green.setEventHandler(handleButtonEvent);
  button_white.init(BUTTONWHITE_PIN, LOW, 0);
  button_white.setEventHandler(handleButtonEvent);
  button_blue.init(BUTTONBLUE_PIN, LOW, 0);
  button_blue.setEventHandler(handleButtonEvent);
  button_red.init(BUTTONRED_PIN, LOW, 0);
  button_red.setEventHandler(handleButtonEvent);

  // Build vibration patterns out of elements
  vibStressAlarm.append(rampUp);
  vibStressAlarm.append(rampDown);
  vibStressAlarm.append(piramid);
  vibChallengeAlarm.append(constFull);
  vibChallengeAlarm.append(constHalf);
  vibInactiveAlarm.append(rampDown);
  vibChallenge.append(rampUp);
  vibChallenge.append(constZero);
  vibChallenge.append(rampDown);
  vibChallenge.append(constZero);

  // Pressure sensor
  // To calibrate the pressure sensor, whear the glove, relax your hand
  // and reset or power-cycle the glove.
  pressureSense.begin(); 

  // Heartrate sensor
  pulseSensor.analogInput(HEARTRATE_PIN);
  pulseSensor.setThreshold(HEARTRATE_THRESHOLD);
  if (!pulseSensor.begin()) {
    Serial.println("Heartrate sensor failed!");
    delay(2000);
  }

  // Data acquisition setup
  telemetryTimer.setInterval(5000);
  telemetryTimer.expiredHandler(telemetryTimer_handler);
  telemetryTimer.setRepeats(0); // 0 = Forever
  telemetryTimer.stop();

  // Finite State Machine transition table construction
  // It's possible to make fully timed transitions. Check the documentation at Jon Black's website:
  // https://jonblack.me/arduino-multitasking-using-finite-state-machines/
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

////////////////////////////////////////////////////////////////////////////////
//                                                                            //
// LOOP FUNCTION, Main structure of the code is executed here.                //
// To find the actual work being done, look for the state-machine's event     //
// handlers. e.g. on_standby(), on_challenge_enter() etc.                     //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////
void loop() {
  // All objects invoke callbacks so the whole program is event-driven
  fsm_main.run_machine();
  telemetryTimer.run();
  pressureSense.run();
  button_green.check();
  button_white.check();
  button_blue.check();
  button_red.check();
}

////////////////////////////////////////////////////////////////////////////////
//                                                                            //
// GENERAL EVENT HANDLERS. These functions handle non-state-machine events.   //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////
void telemetryTimer_handler(MillisTimer &mt)
{
  // Collect all data we can and store it to SD card / Send it of BTLE

  int bpm = pulseSensor.getBeatsPerMinute();  // Calls function on our pulseSensor object that returns BPM as an "int".
  Serial.print("Heartbeat: "); Serial.print(bpm); Serial.println(" BPM");

  //  if (bpm > HEARTRATE_THRESHOLD) {
  //    events.push(STRESS_DETECTED);
  //  }
}

void on_clench(uint16_t pressure)
{
  // This will be invoked when we detect that the user is clenching his fist 
  // for a (configurable) while. Check PressureSensor.h to configure this.
  Serial.print("CLENCH EVENT: "); Serial.println(pressure);
  events.push(CLENCH_ACTIVATED);
}

void on_release(uint16_t pressure)
{
  // This will be invoked when we detect that the user is releasing his clenched
  // fist.
  Serial.print("CLENCH RELEASE EVENT: "); Serial.println(pressure);
  events.push(CLENCH_DEACTIVATED);
}

void handleButtonEvent(AceButton* bt, uint8_t eventType,
                       uint8_t /* buttonState */) {
  switch (eventType) {
    case AceButton::kEventPressed:
      switch (bt->getPin()) {
        case BUTTONGREEN_PIN:
          events.push(CHALLENGE_DETECTED);
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

////////////////////////////////////////////////////////////////////////////////
//                                                                            //
// STATE MACHINE EVENT HANDLERS. These functions are specifically for the     //
// main state-machine which handles the bulk of the buisness.                 //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////
void on_standby() {
  check_triggers();
}

void on_standby_enter() {
  Serial.println("Standby enter");
}

unsigned long challenge_vib_rep_count = 0;
void on_challenge() {
  vibChallenge.go();

  if (vibChallenge.finished()) {
    // Vibration cycle is done, keep track of repetitions
    // and stop after x repetitions.
    vibChallenge.reset();
    ++challenge_vib_rep_count;

    if (challenge_vib_rep_count >= CHALLENGE_VIB_REPETITIONS) {
      challenge_vib_rep_count = 0;
      // If the user forgets to press the button, we do it for them after
      // many repetitions
      events.push(CHALLENGE_BUTTON_ACTIVATED);
    }
  }

  check_triggers();
}

void on_challenge_enter() {
  Serial.println("Challenge enter");
}

void on_challenge_exit() {
  vibChallenge.reset();
  challenge_vib_rep_count = 0;
  Serial.println("Challenge exit");
}

void on_selfreport() {
  check_triggers();
}
void on_selfreport_enter() {
  Serial.println("Selfreport enter");
}

void on_stressalarm() {
  vibStressAlarm.go();

  if (vibStressAlarm.finished()) {
    // Vibration is done
    vibStressAlarm.reset(); // Make sure vibration stops and the whole cycle resets
    Serial.println("vibStressAlarm is done!");
    events.push(STRESSALARM_TIMEOUT);
  }

  check_triggers();
}

void on_stressalarm_enter() {
  Serial.println("Stressalarm enter");
}

void on_stressalarm_exit() {
  vibStressAlarm.reset(); // Stop the vibration in case we were interrupted
  Serial.println("Stressalarm exit");
}

void on_challengealarm()
{
  static unsigned long repetition_counter = 0;

  vibChallengeAlarm.go();

  if (vibChallengeAlarm.finished()) {
    // Vibration cycle is done
    Serial.println("Challenge alarm done!");
    vibChallengeAlarm.reset();
    events.push(CHALLENGEALARM_TIMEOUT);
  }

  check_triggers();
}

void on_challengealarm_enter() {
  Serial.println("Challenge alarm enter");
}

void on_challengealarm_exit() {
  vibChallengeAlarm.reset(); // Stop the vibration in case we were interrupted
  Serial.println("Challenge alarm exit");
}

void on_inactivityalarm() {
  vibInactiveAlarm.go();

  if (vibInactiveAlarm.finished()) {
    // Vibration is done
    vibInactiveAlarm.reset();
    Serial.println("vibInactiveAlarm done!");
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

////////////////////////////////////////////////////////////////////////////////
//                                                                            //
// VIBRATION ELEMENT ALGORITHMS. These functions contain the calculations     //
// which determine the next value in a vibration pattern. x is the moving     //
// variable, max_x contains the maximum value x will reach. Use VIBRATION_MAX //
// to determine the maximum value you can and should return.                  //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////
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

// Are you still reading this?
