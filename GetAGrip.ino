// By Richard Bekking & Nirav Malsattar
// richard@electronicsdesign.nl
// niravmalsatter@gmail.com

// #define _TASK_TIMECRITICAL      // Enable monitoring scheduling overruns
#define _TASK_SLEEP_ON_IDLE_RUN   // Enable 1 ms SLEEP_IDLE powerdowns between tasks if no callback methods were invoked during the pass
// #define _TASK_STATUS_REQUEST    // Compile with support for StatusRequest functionality - triggering tasks on status change events in addition to time only
// #define _TASK_WDT_IDS           // Compile with support for wdt control points and task ids
// #define _TASK_LTS_POINTER       // Compile with support for local task storage pointer
#define _TASK_PRIORITY            // Support for layered scheduling priority
// #define _TASK_MICRO_RES         // Support for microsecond resolution
// #define _TASK_STD_FUNCTION      // Support for std::function (ESP8266 and ESP32 ONLY)
#define _TASK_DEBUG               // Make all methods and variables public for debug purposes
// #define _TASK_INLINE            // Make all methods "inline" - needed to support some multi-tab, multi-file implementations
#define _TASK_TIMEOUT             // Support for overall task timeout
// #define _TASK_OO_CALLBACKS      // Support for dynamic callback method binding

#define USE_ARDUINO_INTERRUPTS true
#include <PulseSensorPlayground.h>
#include <SparkFun_HM1X_Bluetooth_Arduino_Library.h> // BLE for library for BluetoothMate 4.0  https://github.com/sparkfun/SparkFun_HM1X_Bluetooth_Arduino_Library
#include <TaskScheduler.h>    // Scheduling for arduino based task https://github.com/arkhipenko/TaskScheduler/wiki/API-Task
#include "arduino_bma456.h"  //Step Counter through Accelerometer : https://github.com/Seeed-Studio/Seeed_BMA456
#include <SD.h>
#include <Wire.h>
#include <RTClib.h>
#include "Fsm.h"
#include "EventQueue.h"
#include "PressureSensor.h"
#include "Vibration.h"

#define SerialPort Serial3 // Abstract serial monitor debug port

// Debug and Test options
#define _DEBUG_
//#define _TEST_

#ifdef _DEBUG_
#define _PP(a) Serial.print(a);
#define _PL(a) Serial.println(a);
#else
#define _PP(a)
#define _PL(a)
#endif

// PIN definitions
const int CHIPSELECT_PIN  = 10;
const int VIBRATOR_PIN = 6;
const int HEARTRATE_PIN = 0;
const int PRESSURE_OUTPUT = 8;
const int PRESSURE_INPUT  = A2;

void initBLE();
void initSDCard();
void initClk();
void initHR();
void initAcce();
void intVib ();
void initPressureSens();
void initFSM();

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
void fbleCommCallback();
void bleCallback();
void fsmCallback();
void logToSDcard();

uint16_t vib_zero(uint16_t, uint16_t);
uint16_t vib_half(uint16_t, uint16_t);
uint16_t vib_full(uint16_t, uint16_t);
uint16_t vib_rampup(uint16_t, uint16_t);
uint16_t vib_rampdown(uint16_t, uint16_t);
uint16_t vib_piramid(uint16_t, uint16_t);

// State machine
const int PROGMEM STRESS_DETECTED = 1;
const int PROGMEM CHALLENGE_BUTTON_ACTIVATED = 2;
const int PROGMEM INACTIVITY_DETECTED = 3;
const int PROGMEM CHALLENGE_DETECTED = 4;
const int PROGMEM CLENCH_ACTIVATED = 5;
const int PROGMEM CLENCH_DEACTIVATED = 6;
const int PROGMEM STRESSALARM_TIMEOUT   = 7;
const int PROGMEM CHALLENGEALARM_TIMEOUT = 8;
const int PROGMEM INACTIVITYALARM_TIMEOUT = 9;

//Bluetooth
HM1X_BT bt;

// Realtime Clock
RTC_DS1307 clock; //define a object of DS1307 class
char buf1[10]; //array buffer to store time data in char
char fname[50]; //array buffer to store filename in char

// Heartrate
const int PROGMEM HEARTRATE_THRESHOLD = 550;
PulseSensorPlayground pulseSensor;

//Step Counter (Accelerometer)
uint32_t step = 0;

// Vibration
const int PROGMEM CHALLENGE_VIB_REPETITIONS = 1000;
VibrationElement constZero    (1,   500, 1,  &vib_zero);     //  _______ <- 0%
VibrationElement constHalf    (1,   1500, 1, &vib_half);     //  ------- <- 50%
VibrationElement constFull    (1,   1500, 1, &vib_full);     //  ------- <- 100%
VibrationElement rampUp       (100, 30,  1,  &vib_rampup);   //  goes from zero to MAX (scaled over time)
VibrationElement sawtoothUp   (100, 2,   5,  &vib_rampup);   //  rampup many times
VibrationElement rampDown     (100, 10,  1,  &vib_rampdown); //  goes from MAX to zero (scaled over time)
VibrationElement sawtoothDown (100, 2,   5,  &vib_rampdown); //  rampdown many times
//  Example of a combined ramp-up and ramp-down in one function.
//  It's easier to seperate this behaviour in multiple functions for clarity
VibrationElement piramid      (100, 5,  10,  &vib_piramid);

Vibration vibStressAlarm(VIBRATOR_PIN);     // These vibration patterns are assembled in setup();
Vibration vibChallengeAlarm(VIBRATOR_PIN);
Vibration vibChallenge(VIBRATOR_PIN);
Vibration vibInactiveAlarm(VIBRATOR_PIN);

// Pressure
const int PROGMEM CLENCH_THRESHOLD = 60;
void on_clench(uint16_t pressure);
void on_release(uint16_t pressure);
PressureSensor pressureSense(PRESSURE_OUTPUT, PRESSURE_INPUT, CLENCH_THRESHOLD, &on_clench, &on_release);

Scheduler tsLogData;
Scheduler tsFSM;
/*
  Scheduling defines:
  TASK_MILLISECOND
  TASK_SECOND
  TASK_MINUTE
  TASK_HOUR
  TASK_IMMEDIATE
  TASK_FOREVER
  TASK_ONCE
  TASK_NOTIMEOUT
*/

Task bleCMD(TASK_IMMEDIATE, TASK_FOREVER, &fbleCommCallback, &tsLogData, true);
Task bleLog(5 * TASK_SECOND, TASK_FOREVER, &bleCallback, &tsLogData, true);
Task runFSM(TASK_IMMEDIATE, TASK_FOREVER, &fsmCallback, &tsFSM, true);

State state_standby(&on_standby_enter, &on_standby, NULL);
State state_challenge(&on_challenge_enter, &on_challenge, &on_challenge_exit);
State state_selfreport(&on_selfreport_enter, &on_selfreport, NULL);
State state_stressalarm(&on_stressalarm_enter, &on_stressalarm, &on_stressalarm_exit);
State state_challengealarm(&on_challengealarm_enter, &on_challengealarm, &on_challengealarm_exit);
State state_inactivityalarm(&on_inactivityalarm_enter, &on_inactivityalarm, &on_inactivityalarm_exit);
Fsm fsm_main(&state_standby);
EventQueue events;

//Initialization functions of all components
void initBLE() {
  if (bt.begin(SerialPort, 115200) == false) {
    Serial.println(F("Failed to connect to Bluetooth"));
    while (1) ;
  } else
    Serial.println(F("Bluetooth Initialized!"));
}

void initSDCard() {
  if (!SD.begin(CHIPSELECT_PIN)) {
    Serial.println(F("ERROR: NO SDCARD DETECTED!"));
    while (1) ;
  } else {
    Serial.println(F("SD Card Initialized"));
  }
}

void initClk() {
  //  //Initialize Clock
  clock.begin();
  clock.adjust(DateTime(F(__DATE__), F(__TIME__)));
  Serial.println(F("Clock Initialized!"));
}

void initHR() {
  // Heartrate sensor
  pulseSensor.analogInput(HEARTRATE_PIN);
  pulseSensor.setThreshold(HEARTRATE_THRESHOLD);
  if (!pulseSensor.begin()) {
    Serial.println(F("Heartrate sensor failed!"));
    delay(20);
  }
}
void initPressureSens() {
  // Pressure sensor
  // To calibrate the pressure sensor, whear the glove, relax your hand
  // and reset or power-cycle the glove.
  pressureSense.begin();
}
void initAcce() {
  //Initialize Accelerometer as StepCounter
  bma456.initialize(RANGE_4G, ODR_1600_HZ, NORMAL_AVG4, CONTINUOUS);
  bma456.stepCounterEnable();
  Serial.println(F("Accelerometer/Step counter Initialized!"));
}
void intVib () {

  pinMode(VIBRATOR_PIN, OUTPUT);
  digitalWrite(VIBRATOR_PIN, LOW);

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
}

void initFSM() {
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
// SETUP FUNCTION, All initialisations happen here                            //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////
void setup() {
  Serial.begin(115200);
  while (!Serial);
  Wire.begin();
  Serial.println(F("Arduino started"));

  initBLE();
  initSDCard();
  initClk();
  initHR();
  initPressureSens();
  initAcce();
  initFSM();
  intVib ();

  Serial.println(F("Start Monitoring"));
}

////////////////////////////////////////////////////////////////////////////////
//                                                                            //
// LOOP FUNCTION, Main structure of the code is executed here.                //
// To find the actual work being done, look for the state-machine's event     //
// handlers. e.g. on_standby(), on_challenge_enter() etc.                     //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////
void loop() {
  tsLogData.execute();
  tsFSM.execute();
  // All objects invoke callbacks so the whole program is event-driven
}

void fbleCommCallback() {
  // If data is available from bt module,
  // print it to serial port
  if (bt.available()) {
    handleBLECommand((char) bt.read());
    Serial.write((char) bt.read());
  }
  // If data is available from serial port,
  // print it to bt module.
  if (Serial.available()) {
    bt.write((char) Serial.read());
  }
}

void bleCallback() {
  DateTime now = clock.now();
  sprintf(buf1, "%02d:%02d:%02d %02d/%02d/%02d",  now.hour(), now.minute(), now.second(), now.day(), now.month(), now.year());

  int bpm = pulseSensor.getBeatsPerMinute();  // Calls function on our pulseSensor object that returns BPM as an "int".
  step = bma456.getStepCounterOutput();

  SerialPort.print(F("T: "));
  SerialPort.print(buf1);
  SerialPort.print("\n");
  SerialPort.print(F("HR: "));
  SerialPort.print(bpm);
  SerialPort.print("\n");
  SerialPort.print(F("Step: "));
  SerialPort.print(step);
  SerialPort.println();
  SerialPort.println();
  SerialPort.println();

  bleLog.setCallback(logToSDcard);
  Serial.println ("SD Card Task Disbaled");

}

void fsmCallback() {
  //  telemetryTimer.run();
  fsm_main.run_machine();
  pressureSense.run();
}

////////////////////////////////////////////////////////////////////////////////
//                                                                            //
// GENERAL EVENT HANDLERS. These functions handle non-state-machine events.   //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////

void on_clench(uint16_t pressure)
{
  // This will be invoked when we detect that the user is clenching his fist
  // for a (configurable) while. Check PressureSensor.h to configure this.
  //  Serial.print("CLENCH EVENT: "); Serial.println(pressure);
  events.push(CLENCH_ACTIVATED);
}

void on_release(uint16_t pressure)
{
  // This will be invoked when we detect that the user is releasing his clenched
  // fist.
  //  Serial.print(F("CLENCH RELEASE EVENT: ")); Serial.println(pressure);
  events.push(CLENCH_DEACTIVATED);
}

void handleBLECommand(char cmd) {
  char numb = cmd;
  switch (numb) {
    case '1':
      events.push(CHALLENGE_DETECTED);
      break;
    case '2':
      events.push(CHALLENGE_BUTTON_ACTIVATED);
      break;
    case '3':
      events.push(INACTIVITY_DETECTED);
      break;
    case '4':
      events.push(STRESS_DETECTED);
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
  //  Serial.println(F("On Standby"));
}

void on_standby_enter() {
  Serial.println(F("Standby enter"));
}

unsigned long challenge_vib_rep_count = 0;
void on_challenge() {
  vibChallenge.go();
  Serial.println(F("Vib Chall go"));
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
  Serial.println(F("Challenge enter"));
}

void on_challenge_exit() {
  vibChallenge.reset();
  challenge_vib_rep_count = 0;
  Serial.println(F("Challenge exit"));
}

void on_selfreport() {
  check_triggers();
  //  Serial.println(F("On Selfreport"));
}

void on_selfreport_enter() {
  Serial.println(F("Selfreport enter"));
}

void on_stressalarm() {
  vibStressAlarm.go();

  if (vibStressAlarm.finished()) {
    // Vibration is done
    vibStressAlarm.reset(); // Make sure vibration stops and the whole cycle resets
    Serial.println(F("vibStressAlarm is done!"));
    events.push(STRESSALARM_TIMEOUT);
  }

  check_triggers();
}

void on_stressalarm_enter() {
  Serial.println(F("Stressalarm enter"));
}

void on_stressalarm_exit() {
  vibStressAlarm.reset(); // Stop the vibration in case we were interrupted
  Serial.println(F("Stressalarm exit"));
}

void on_challengealarm()
{
  static unsigned long repetition_counter = 0;

  vibChallengeAlarm.go();

  if (vibChallengeAlarm.finished()) {
    // Vibration cycle is done
    Serial.println(F("Challenge alarm done!"));
    vibChallengeAlarm.reset();
    events.push(CHALLENGEALARM_TIMEOUT);
  }

  check_triggers();
}

void on_challengealarm_enter() {
  Serial.println(F("Challenge alarm enter"));
}

void on_challengealarm_exit() {
  vibChallengeAlarm.reset(); // Stop the vibration in case we were interrupted
  Serial.println(F("Challenge alarm exit"));
}

void on_inactivityalarm() {
  vibInactiveAlarm.go();

  if (vibInactiveAlarm.finished()) {
    // Vibration is done
    vibInactiveAlarm.reset();
    Serial.println(F("vibInactiveAlarm done!"));
    events.push(INACTIVITYALARM_TIMEOUT);
  }

  check_triggers();
}

void on_inactivityalarm_enter() {
  Serial.println(F("Inactivity alarm enter"));
}

void on_inactivityalarm_exit() {
  Serial.println(F("Inactivity alarm exit"));
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

void logToSDcard()
{ 

  // Compile filename for SD card logging (YYYY-MM-DD)
  DateTime now = clock.now();
  sprintf(fname, "%02d%02d%02d.csv",  now.year(), now.month(), now.day());

  int bpm = pulseSensor.getBeatsPerMinute();  // Calls function on our pulseSensor object that returns BPM as an "int".

  // open the file. note that only one file can be open at a time,
  // so you have to close this one before opening another.
  File dataFile = SD.open(fname, FILE_WRITE);
 
    if (dataFile) {
      // Timestamp
      dataFile.print(F("T: "));
      dataFile.print(buf1);
      dataFile.println();
      // Datafields (HR and Steps)
      dataFile.print(F("HR: "));
      dataFile.print(bpm);
      dataFile.println();
      dataFile.print(F("Step: "));
      dataFile.print(step);
  
      // Newline at end of file
      dataFile.println();
      dataFile.close();
    }
    else
    {
      Serial.println ("Error in opening the file!!");
      while(1);
    }
  
  Serial.println ("Data Logged to SD Card and Task Disbaled");
  bleLog.setCallback(bleCallback);
}

///////////////////////////////////////////////////////////////////////////////
//                                                                           //
// Returns a zero padded string based on a two-digit input value             //
//                                                                           //
// Input:  a single- or double digit number                                  //
// Output: if the input was a single digit number, the output will have a    //
//         leading zero. Otherwise the double digit number will be returned  //
//                                                                           //
///////////////////////////////////////////////////////////////////////////////
String ZeroPad(int value)
{
  String returnVal = (value > 9 ? "" : String(F("0"))) + String(value);
  return returnVal;
}
// Are you still reading this?
