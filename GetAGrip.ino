// By Nirav Malsattar
// niravmalsatter@gmail.com

// Debug and Test options
//#define _DEBUG_
#define _TEST_

#ifdef _DEBUG_
#define _PP(a) Serial.print(a);
#define _PL(a) Serial.println(a);
#else
#define _PP(a)
#define _PL(a)
#endif

#define comma ','   // comma ','
#define TCAADDR 0x5A
#define RTCADDR 0x68
#define ACCADDR 0x53
#define SerialPort Serial1 // Abstract serial monitor debug port

#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#define USE_ARDUINO_INTERRUPTS false //Use ths false to tell not to use arduino Interrupt by pulse sensor plaground library
#include <PulseSensorPlayground.h>
#include <SparkFun_HM1X_Bluetooth_Arduino_Library.h> // BLE for library for BluetoothMate 4.0  https://github.com/sparkfun/SparkFun_HM1X_Bluetooth_Arduino_Library
#include "arduino_bma456.h"                          //Step Counter through Accelerometer : https://github.com/Seeed-Studio/Seeed_BMA456
#include <RTClib.h>
#include "Fsm.h" //Finite State machine librarary https://github.com/jonblack/arduino-fsm
#include "EventQueue.h"
#include "MillisTimer.h"
#include "Adafruit_DRV2605.h"

// PIN definitions
const int CHIPSELECT_PIN = 4;
const int CHALLENGE_BUTTON = 10;
const int HEARTRATE_PIN = A1;
int PRESSURE_OUTPUT = 3;
int PRESSURE_INPUT = A0;

void initTimer();
void initBLE();
void initSDCard();
void initClk();
void initHR();
void initAcce();
void intVib();
void initPressureSens();
void intChlngBtn();
void initFSM();

void on_standby();
void on_standby_enter();
void on_challenge_enter();
void on_challenge();
void on_challenge_exit();
void on_selfreport();
void on_selfreport_enter();
void on_selfreport_exit();
void on_stressalarm_enter();
void on_stressalarm();
void on_stressalarm_exit();
void on_challengealarm_enter();
void on_challengealarm();
void on_challengealarm_exit();
void on_inactivityalarm_enter();
void on_inactivityalarm ();
void on_inactivityalarm_exit();
void on_logdata_enter();
void on_logdata();
void on_logdata_exit();
void check_triggers();
uint32_t currTimestamp();
int currHR();
int currSteps ();
void logToSDcard();
void transToBLE();
void checkBLECmd();

// State machine
const int PROGMEM STRESS_DETECTED = 1;
const int PROGMEM CHALLENGE_BUTTON_ACTIVATED = 2;
const int PROGMEM INACTIVITY_DETECTED = 3;
const int PROGMEM CHALLENGE_DETECTED = 4;
const int PROGMEM CLENCH_ACTIVATED = 5;
const int PROGMEM CLENCH_DEACTIVATED = 6;
const int PROGMEM STRESSALARM_TIMEOUT = 7;
const int PROGMEM CHALLENGEALARM_TIMEOUT = 8;
const int PROGMEM INACTIVITYALARM_TIMEOUT = 9;
const int PROGMEM LOG_DATA = 10;
const int PROGMEM LOG_DATA_TIMEOUT = 11;

//Bluetooth
HM1X_BT bt;

// Realtime Clock
RTC_DS1307 clock; //define a object of DS1307 class
uint32_t buf1;    //array buffer to store time data in char
char fname[20];   //array buffer to store filename in char
char date[10];   //array buffer to store filename in char
char timeT[10];   //array buffer to store filename in char

// Heartrate
byte samplesUntilReport;
const byte SAMPLES_PER_SERIAL_SAMPLE = 10;
const int PROGMEM HEARTRATE_THRESHOLD = 555;
PulseSensorPlayground pulseSensor;
int myBPM;

//Step Counter (Accelerometer)
uint32_t step = 0;
float x = 0, y = 0, z = 0;
double sqrtAcce = 0;

// Vibration
Adafruit_DRV2605 drv;

bool fchallengeVib = false;
bool finactivityAlarm = false;
bool fchallengePrompt = false;
bool fstressAlarm = false;
unsigned long chlng_previousMillis = 0;
unsigned long chlng_currentMillis = 0;

bool doneBreathIn = false;
bool doneBreathOut = false;
int long challengeVib_interval = 150000;
int long stressAlarm_interval = 5000;
int long challengePrompt_interval = 5000;
int long inactivityAlarm_interval = 1000;

unsigned long timerChallengeBegin = 0;
unsigned long previousMillis = 0;

// Pressure
int long fsrReading = 0;
int pressureLvl = 0;

//Challenge Button
bool lastButtonState = 1;

// Data acquisition
void telemetryTimer_handler(MillisTimer &mt);
MillisTimer telemetryTimer = MillisTimer(5000); // Take a snapshot of all the date once every 5 seconds

//Defining States for state machine machine to run
State state_standby(&on_standby_enter, &on_standby, NULL);
State state_challenge(&on_challenge_enter, &on_challenge, &on_challenge_exit);
State state_selfreport(&on_selfreport_enter, &on_selfreport, &on_selfreport_exit);
State state_stressalarm(&on_stressalarm_enter, &on_stressalarm, &on_stressalarm_exit);
State state_challengealarm(&on_challengealarm_enter, &on_challengealarm, &on_challengealarm_exit);
State state_inactivityalarm(&on_inactivityalarm_enter, &on_inactivityalarm, &on_inactivityalarm_exit);
State state_log_data(&on_logdata_enter, &on_logdata, &on_logdata_exit);
Fsm fsm_main(&state_standby);
EventQueue events;

//Initialization functions of all components
void initBLE()
{
  if (bt.begin(SerialPort, 115200) == false)
  {
    _PL(F("Failed to connect to Bluetooth"));
    while (1);
  }
  else
    _PL(F("Bluetooth Initialized!"));
}

void initSDCard()
{
  if (!SD.begin(CHIPSELECT_PIN))
  {
    _PL(F("ERROR: NO SDCARD DETECTED!"));
    while (1);
  }
  else
  {
    _PL(F("SD Card Initialized"));
  }
}

void initClk()
{
  //Initialize Clock
  clock.begin();
  clock.adjust(DateTime(F(__DATE__), F(__TIME__)));
  _PL(F("Clock Initialized!"));
}

void initHR()
{
  // Heartrate sensor
  pulseSensor.analogInput(HEARTRATE_PIN);
  pulseSensor.setThreshold(HEARTRATE_THRESHOLD);
  // Skip the first SAMPLES_PER_SERIAL_SAMPLE in the loop().
  samplesUntilReport = SAMPLES_PER_SERIAL_SAMPLE;
  if (pulseSensor.begin())
  {
    _PL(F("Heartrate Initialized"));
    delay(20);
  }
}

void initPressureSens()
{
  pinMode(PRESSURE_OUTPUT, OUTPUT);
  pinMode(PRESSURE_INPUT, INPUT);

  digitalWrite(PRESSURE_OUTPUT, HIGH);
}

void initAcce()
{
  //Initialize Accelerometer as StepCounter
  bma456.initialize(RANGE_4G, ODR_1600_HZ, NORMAL_AVG4, CONTINUOUS);
  bma456.stepCounterEnable();
  _PL(F("Accelerometer/Step counter Initialized!"));
}

void intVib()
{
  drv.begin();
  drv.selectLibrary(6);
  drv.useLRA();
}

void intChlngBtn() {
  pinMode(CHALLENGE_BUTTON, INPUT_PULLUP);
}

void initFSM()
{
  // Finite State Machine transition table construction
  // It's possible to make fully timed transitions. Check the documentation at Jon Black's website:
  // https://jonblack.me/arduino-multitasking-using-finite-state-machines/
  fsm_main.add_transition(&state_standby, &state_stressalarm,
                          STRESS_DETECTED,
                          NULL);
  fsm_main.add_transition(&state_stressalarm, &state_standby,
                          STRESSALARM_TIMEOUT,
                          NULL);
  fsm_main.add_transition(&state_standby, &state_log_data,
                          LOG_DATA,
                          NULL);
  fsm_main.add_transition(&state_log_data, &state_standby,
                          LOG_DATA_TIMEOUT,
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

void initTimer() {
  // Data acquisition setup
  telemetryTimer.setInterval(5000);
  telemetryTimer.expiredHandler(telemetryTimer_handler);
  telemetryTimer.setRepeats(0); // 0 = Forever
  telemetryTimer.start();
}

////////////////////////////////////////////////////////////////////////////////
//                                                                            //
// SETUP FUNCTION, All initialisations happ en here                            //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////
void setup()
{
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000);
  delay (1000);
  _PL(F("Arduino started"));
  _PL(F("Start Initialize Sensors"));

  initBLE();
  initSDCard();
  initClk();
  initHR();
  initPressureSens();
  initAcce();
  intChlngBtn();
  initFSM();
  intVib();
  initTimer();

  _PL(F("Sensors Initialize Successfully Finished"));
  _PL(F("Start StateMachine"));
}

////////////////////////////////////////////////////////////////////////////////
//                                                                            //
// LOOP FUNCTION, Main structure of the code is executed here.                //
// To find the actual work being done, look for the state-machine's event     //
// handlers. e.g. on_standby(), on_challenge_enter() etc.                     //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////
void loop()
{
  // All objects invoke callbacks so the whole program is event-driven
  fsm_main.run_machine();
}

////////////////////////////////////////////////////////////////////////////////
//                                                                            //
// GENERAL EVENT HANDLERS. These functions handle non-state-machine events.   //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////

void telemetryTimer_handler(MillisTimer &mt)
{
  events.push(LOG_DATA);
}

void checkBLECmd()
{
  if (bt.available())
  {
    handleBLECommand((char)bt.read());
  }

  if (SerialPort.available())
  {
    bt.write((char)SerialPort.read());
  }
}

void handleBLECommand(char cmd)
{
  char numb = cmd;
  switch (numb)
  {
    case '0':
      events.push(CHALLENGE_DETECTED);
      break;
    case '1':
      events.push(CHALLENGE_BUTTON_ACTIVATED);
      break;
    case '2':
      events.push(INACTIVITY_DETECTED);
      break;
    case '3':
      events.push(STRESS_DETECTED);
      break;
  }
}

bool handleButton() {
  bool switchState = 0;
  if (!digitalRead(CHALLENGE_BUTTON) && lastButtonState) {
    events.push(CHALLENGE_BUTTON_ACTIVATED);
    switchState = 1;
  }
  lastButtonState = digitalRead(CHALLENGE_BUTTON);
  return switchState;
}

void pressureSense() {
  fsrReading = analogRead(PRESSURE_INPUT);
  if (fsrReading < 700) {
    events.push(CLENCH_ACTIVATED);
  }
  else if (fsrReading > 700) {
    //    _PL(" - No pressure");
    pressureLvl = 0;
  }
}

////////////////////////////////////////////////////////////////////////////////
//                                                                            //
// DIFFERENT VIBRATION PATTERNS FOR EACH ALARM AND TASK                       //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////
void breathIn() {
  drv.setWaveform(0, 1);
  drv.setWaveform(1, 123);
  drv.setWaveform(2, 122);
  drv.setWaveform(3, 122);
  drv.setWaveform(4, 121);
  drv.setWaveform(5, 120);
  drv.setWaveform(6, 119);
  drv.useLRA();
  drv.go();
  doneBreathIn = true;
  _PL(F("Breath In"));
}

void breathOut() {
  drv.setWaveform(0, 119);
  drv.setWaveform(1, 120);
  drv.setWaveform(2, 121);
  drv.setWaveform(3, 121);
  drv.setWaveform(4, 122);
  drv.setWaveform(5, 122);
  drv.setWaveform(6, 123);
  drv.useLRA();
  drv.go();
  doneBreathOut = true;
  _PL(F("Breath Out"));
}
void chlng_vib () {
  chlng_currentMillis = millis();

  if (chlng_currentMillis - chlng_previousMillis >= 5000) {
    if (!doneBreathOut) {
      breathOut();
    }
  } else if (chlng_currentMillis - chlng_previousMillis <= 5000) {
    if (!doneBreathIn) {
      breathIn();
    }
  }
  if (chlng_currentMillis - chlng_previousMillis >= 10000) {
    // save the last time you blinked the LED
    doneBreathIn = false;
    doneBreathOut = false;
    chlng_previousMillis = chlng_currentMillis;
  }
}

void challengePromptVib() {

  drv.setWaveform(0, 1);
  drv.setWaveform(1, 60);
  drv.setWaveform(2, 0);

  unsigned long starttime = millis();
  unsigned long endtime = starttime;

  while ((endtime - starttime) <= challengePrompt_interval) // do this loop for up to 1000mS
  {
    // code here
    drv.go();
    endtime = millis();
  }

  events.push(CHALLENGEALARM_TIMEOUT);
}

void inactivityAlarm() {
  drv.setWaveform(0, 1);
  drv.setWaveform(1, 22);
  drv.setWaveform(2, 0);

  unsigned long starttime = millis();
  unsigned long endtime = starttime;

  while ((endtime - starttime) <= inactivityAlarm_interval) // do this loop for up to 1000mS
  {
    // code here
    drv.go();
    endtime = millis();
  }
  events.push(INACTIVITYALARM_TIMEOUT);
}

void stressAlarm() {
  drv.setWaveform(0, 1);
  drv.setWaveform(1, 45);
  drv.setWaveform(2, 0);

  unsigned long starttime = millis();
  unsigned long endtime = starttime;

  while ((endtime - starttime) <= stressAlarm_interval) // do this loop for up to 1000mS
  {
    // code here
    drv.go();
    endtime = millis();
  }
  events.push(STRESSALARM_TIMEOUT);
}

////////////////////////////////////////////////////////////////////////////////
//                                                                            //
// STATE MACHINE EVENT HANDLERS. These functions are specifically for the     //
// main state-machine which handles the bulk of the buisness.                 //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////

void on_standby_enter()
{
  _PL(F("Standby enter"));
}

void on_standby()
{
  currHR ();
  handleButton();
  currSteps ();
  pressureSense();
  checkBLECmd();
  telemetryTimer.run();
  check_triggers();
}

void on_logdata_enter() {
  _PL(F("LogData enter"));
}

void on_logdata() {
  _PL(F("On LogData"));
  logToSDcard();
  delay (20);
  transToBLE();
  events.push(LOG_DATA_TIMEOUT);
  check_triggers();
}

void on_logdata_exit() {
  myBPM = 0;
  _PL(F("LogData Finished"));
}

void on_challenge_enter()
{
  _PL(F("Challenge enter"));
  fchallengeVib = true;
  telemetryTimer.stop();
  myBPM = 0;
  timerChallengeBegin = millis();
  chlng_previousMillis = millis();
  doneBreathIn = false;
  doneBreathOut = false;
}

void on_challenge()
{
  _PL(F("On Challenge"));
  currHR ();
  currSteps ();
  fsrReading = analogRead(PRESSURE_INPUT);


  if (!handleButton()) {
    if (millis() - timerChallengeBegin < challengeVib_interval) {
      chlng_vib();
      if (fsrReading < 500) {
        _PL(" - Big squeeze");
        pressureLvl = 3;
      } else if (fsrReading < 600) {
        _PL(" - Medium squeeze");
        pressureLvl = 2;
      } else if (fsrReading < 700) {
        _PL(" - Light squeeze");
        pressureLvl = 1;
      }
    } else {
      drv.stop();
      events.push(CHALLENGE_BUTTON_ACTIVATED);
    }
    check_triggers();
  }


  else drv.stop();

  if (millis() - previousMillis >= 1000) {
    previousMillis = millis();
    logToSDcard();
    transToBLE();
  }
}

void on_challenge_exit()
{
  drv.setWaveform (0, 0);
  doneBreathIn = false;
  doneBreathOut = false;
  fchallengeVib = false;
  previousMillis = 0;
  timerChallengeBegin = 0;
  telemetryTimer.start();
  _PL(F("Challenge exit"));
}

void on_challengealarm_enter()
{
  _PL(F("Challenge alarm enter"));
  telemetryTimer.stop();
  fchallengePrompt = true;
}

void on_challengealarm()
{
  _PL(F("On Challenge Alarm"));
  logToSDcard();
  delay (20);
  transToBLE();
  challengePromptVib();
  check_triggers();
}

void on_challengealarm_exit()
{
  _PL(F("Challenge alarm exit"));
  telemetryTimer.start();
  fchallengePrompt = false;
}

void on_selfreport_enter()
{
  _PL(F("Selfreport enter"));
  telemetryTimer.stop();
}

void on_selfreport()
{
  _PL(F("On Self Report!!"));
  fsrReading = analogRead(PRESSURE_INPUT);

  if (fsrReading < 500) {
    _PL(" - Big squeeze");
    pressureLvl = 3;
    drv.setWaveform (0, 12);
    drv.setWaveform (1, 0);
  } else if (fsrReading < 600) {
    _PL(" - Medium squeeze");
    pressureLvl = 2;
    drv.setWaveform (0, 10);
    drv.setWaveform (1, 0);
  } else if (fsrReading < 700) {
    _PL(" - Light squeeze");
    pressureLvl = 1;
    drv.setWaveform (0, 1);
    drv.setWaveform (1, 0);
  } else {
    drv.stop();
    events.push (CLENCH_DEACTIVATED);
  }

  if (millis() - previousMillis >= 500) {
    drv.useLRA();
    drv.go();
    logToSDcard();
    transToBLE();
    previousMillis = millis();
  }
  check_triggers();
}

void on_selfreport_exit()
{
  _PL(F("Self Report Succeed!!"));
  drv.setWaveform (0, 0);
  pressureLvl = 0;
  previousMillis = 0;
  telemetryTimer.start();
}

void on_stressalarm_enter()
{
  _PL(F("Stressalarm enter"));
  fstressAlarm = true;
  telemetryTimer.stop();
}

void on_stressalarm()
{
  _PL(F("On Stress Alarm"));
  logToSDcard();
  delay (20);
  transToBLE();
  stressAlarm();
  check_triggers();
}

void on_stressalarm_exit()
{
  _PL(F("Stressalarm exit"));
  fstressAlarm = false;
  telemetryTimer.start();
}

void on_inactivityalarm_enter()
{
  _PL(F("Inactivity alarm enter"));
  finactivityAlarm = true;
  telemetryTimer.stop();
}

void on_inactivityalarm()
{
  _PL(F("On Inactivity Alarm"));
  logToSDcard();
  delay (20);
  transToBLE();
  inactivityAlarm();
  check_triggers();
}

void on_inactivityalarm_exit()
{
  _PL(F("Inactivity alarm exit"));
  finactivityAlarm = false;
  telemetryTimer.start();
}

void check_triggers()
{
  // Are there any state-changing events?
  // If yes, execute them.
  if (events.state() != EMPTY)
  {
    fsm_main.trigger(events.pop());
  }
}

////////////////////////////////////////////////////////////////////////////////
//                                                                            //
// Data Collection Functions. All the functions related to activate sensors   //
// and return each of value                                                   //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////

uint32_t currTimestamp() {
  DateTime now = clock.now();
  buf1 = now.unixtime();
  sprintf(fname, "%02d%02d%02d.csv",  now.year(), now.month(), now.day());

  return buf1;
}

int currHR () {
  pulseSensor.resume();
  if (pulseSensor.sawNewSample()) {
    if (--samplesUntilReport == (byte) 0) {
      samplesUntilReport = SAMPLES_PER_SERIAL_SAMPLE;

      if (pulseSensor.sawStartOfBeat()) {
        myBPM = pulseSensor.getBeatsPerMinute();
        _PP("HR: ");
        _PL(myBPM);
        return true;
      }
    }
    /*******
      Here is a good place to add code that could take up
      to a millisecond or so to run.
    *******/
  }
  pulseSensor.pause();
  return false;
}

int currSteps () {
  step = bma456.getStepCounterOutput();
  bma456.getAcceleration(&x, &y, &z);
  sqrtAcce = sqrt(sq(x) + sq(y) + sq(z));
  if (sqrtAcce < 1010) {
    sqrtAcce = 0;
  }
  else if (sqrtAcce > 1010) {
    sqrtAcce = sqrtAcce - 1000;
  }
  //  _PP("AcceSqrt: ");
  //  _PL(sqrtAcce);
  return step;
}

////////////////////////////////////////////////////////////////////////////////
//  Log Data: Functuon to log data into SD Card                               //
//  and Transmit the data over BLE                                            //                                                                            //
////////////////////////////////////////////////////////////////////////////////

void transToBLE() {
  uint32_t ts = currTimestamp();

  SerialPort.print("s");
  SerialPort.print(F("tm"));
  SerialPort.print(ts);
  SerialPort.print(comma);
  SerialPort.print(F("hr"));
  SerialPort.print(myBPM);
  SerialPort.print(comma);
  SerialPort.print(F("sl"));
  SerialPort.print (pressureLvl);
  SerialPort.print(comma);
  SerialPort.print(F("st"));
  SerialPort.print(step);
  SerialPort.print(comma);
  SerialPort.print(F("ac"));
  SerialPort.print(sqrtAcce);
  SerialPort.print(comma);
  SerialPort.print(F("sa"));
  SerialPort.print(fstressAlarm);
  SerialPort.print(comma);
  SerialPort.print(F("ir"));
  SerialPort.print(finactivityAlarm);
  SerialPort.print(comma);
  SerialPort.print(F("cp"));
  SerialPort.print(fchallengePrompt);
  SerialPort.print(comma);
  SerialPort.print("cs");
  SerialPort.print(fchallengeVib);
  SerialPort.print(comma);
  SerialPort.print("e");
  SerialPort.println("\r\n");

  _PL(F("BLE Transmission Succeed"));
  //  return true;
}

void logToSDcard()
{
  DateTime now = clock.now();
  sprintf(fname, "%02d%02d%02d.csv",  now.day(), now.month(), now.year());
  sprintf(date, "%02d.%02d.%02d",  now.day(), now.month(), now.year());
  sprintf(timeT, "%02d:%02d:%02d",  now.hour(), now.minute(), now.second());

  // open the file. note that only one file can be open at a time,
  // so you have to close this one before opening another.
  File dataFile = SD.open(fname, FILE_WRITE);

  if (dataFile)
  {
    dataFile.print(date);
    dataFile.print(comma);
    dataFile.print(timeT);
    dataFile.print(comma);
    dataFile.print(myBPM);
    dataFile.print(comma);
    dataFile.print(pressureLvl);
    dataFile.print(comma);
    dataFile.print(step);
    dataFile.print(comma);
    dataFile.print(sqrtAcce);
    dataFile.print(comma);
    dataFile.print (false);
    dataFile.print(comma);
    dataFile.print(fstressAlarm);
    dataFile.print(comma);
    dataFile.print(finactivityAlarm);
    dataFile.print(comma);
    dataFile.print(fchallengePrompt);
    dataFile.print(comma);
    dataFile.print(fchallengeVib);
    dataFile.print(comma);
    // Newline at end of file
    dataFile.println();
    dataFile.close();
    _PL(F("SDCard Log Succeed"));
  }
  else
  {
    _PL(F("Error in opening the file on SD card!!"));
  }
}
