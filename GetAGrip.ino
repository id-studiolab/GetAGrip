// By Nirav Malsattar
// niravmalsatter@gmail.com

#define comma ','   // comma ','

#define TCAADDR 0x5A
#define RTCADDR 0x68
#define ACCADDR 0x53
#define SerialPort Serial1 // Abstract serial monitor debug port

#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#define USE_ARDUINO_INTERRUPTS false
#include <PulseSensorPlayground.h>
#include <SparkFun_HM1X_Bluetooth_Arduino_Library.h> // BLE for library for BluetoothMate 4.0  https://github.com/sparkfun/SparkFun_HM1X_Bluetooth_Arduino_Library
#include "arduino_bma456.h"                          //Step Counter through Accelerometer : https://github.com/Seeed-Studio/Seeed_BMA456
#include <RTClib.h>
#include "Fsm.h"
#include "EventQueue.h"
#include "PressureSensor.h"
#include "MillisTimer.h"
#include "Adafruit_DRV2605.h"
#include <AceButton.h>
using namespace ace_button;

// PIN definitions
const int CHIPSELECT_PIN = 4;
const int CHALLENGE_BUTTON = 10;
const int HEARTRATE_PIN = A1;
constexpr int PRESSURE_OUTPUT = 3;
constexpr int PRESSURE_INPUT = A0;

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
void on_selfreport();
void on_selfreport_enter();
void on_selfreport_exit();
void on_stressalarm_enter();
void on_stressalarm_exit();
void on_challengealarm_enter();
void on_challengealarm_exit();
void on_inactivityalarm_enter();
void on_inactivityalarm_exit();
void on_logdata_enter();
void on_logdata();
void on_logdata_exit();
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

// Heartrate
byte samplesUntilReport;
const byte SAMPLES_PER_SERIAL_SAMPLE = 10;
const int PROGMEM HEARTRATE_THRESHOLD = 555;
PulseSensorPlayground pulseSensor;
int myBPM;

//Step Counter (Accelerometer)
uint32_t step = 0;

// Vibration
Adafruit_DRV2605 drv;
unsigned long vib_count = 0;
unsigned long chlng_vib_rep = 100;
unsigned long chlng_alarm_rep = 5;
unsigned long inactivity_alarm_rep = 5;
unsigned long stress_alarm_rep = 5;

// Pressure
constexpr int PROGMEM CLENCH_THRESHOLD = 60;
void on_clench(uint16_t pressure);
void on_release(uint16_t pressure);
PressureSensor pressureSense(PRESSURE_OUTPUT, PRESSURE_INPUT, CLENCH_THRESHOLD, &on_clench, &on_release);

//Challenge Button
AceButton button(CHALLENGE_BUTTON);
int buttonState = 0;         // variable for reading the pushbutton status
void handleBtnEvent(AceButton*, uint8_t, uint8_t);

// Data acquisition
void telemetryTimer_handler(MillisTimer &mt);
MillisTimer telemetryTimer = MillisTimer(5000); // Take a snapshot of all the date once every 5 seconds

//Defining States for state machine machine to run
State state_standby(&on_standby_enter, &on_standby, NULL);
State state_challenge(&on_challenge_enter, &on_challenge, &on_challenge_exit);
State state_selfreport(&on_selfreport_enter, &on_selfreport, NULL);
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
    Serial.println(F("Failed to connect to Bluetooth"));
    while (1)
      ;
  }
  else
    Serial.println(F("Bluetooth Initialized!"));
}

void initSDCard()
{
  if (!SD.begin(CHIPSELECT_PIN))
  {
    Serial.println(F("ERROR: NO SDCARD DETECTED!"));
    while (1)
      ;
  }
  else
  {
    Serial.println(F("SD Card Initialized"));
  }
}

void initClk()
{
  //Initialize Clock
  clock.begin();
  clock.adjust(DateTime(F(__DATE__), F(__TIME__)));
  Serial.println(F("Clock Initialized!"));
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
    Serial.println(F("Heartrate Initialized"));
    delay(20);
  }
}

void initPressureSens()
{
  // Pressure sensor
  // To calibrate the pressure sensor, whear the glove, relax your hand
  // and reset or power-cycle the glove.
  pressureSense.begin();
}

void initAcce()
{
  //Initialize Accelerometer as StepCounter
  bma456.initialize(RANGE_4G, ODR_1600_HZ, NORMAL_AVG4, CONTINUOUS);
  bma456.stepCounterEnable();
  Serial.println(F("Accelerometer/Step counter Initialized!"));
}

void intVib()
{
  drv.begin();
  drv.selectLibrary(6);
  drv.useLRA();
}

void intChlngBtn() {
  pinMode(CHALLENGE_BUTTON, INPUT_PULLUP);
  ButtonConfig* buttonConfig = button.getButtonConfig();
  buttonConfig->setEventHandler(handleBtnEvent);
  buttonConfig->setFeature(ButtonConfig::kFeatureClick);
  buttonConfig->setFeature(ButtonConfig::kFeatureLongPress);
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
// SETUP FUNCTION, All initialisations happen here                            //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////
void setup()
{
  Serial.begin(115200);
  Wire.begin();
  delay (1000);
  Serial.println(F("Arduino started"));
  Serial.println(F("Start Initialize Sensors"));

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

  Serial.println(F("Sensors Initialize Successfully Finished"));
  Serial.println(F("Start StateMachine"));
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
  currHR ();
  currSteps ();
  fsm_main.run_machine();
  telemetryTimer.run();
  //  pressureSense.run();
  checkBLECmd();
  button.check();
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

void on_clench(uint16_t pressure)
{
  // This will be invoked when we detect that the user is clenching his fist
  // for a (configurable) while. Check PressureSensor.h to configure this.
  Serial.print("CLENCH EVENT: ");
  Serial.println(pressure);

  events.push(CLENCH_ACTIVATED);
}

void on_release(uint16_t pressure)
{
  // This will be invoked when we detect that the user is releasing his clenched
  // fist.
  Serial.print(F("CLENCH RELEASE EVENT: "));
  Serial.println(pressure);

  events.push(CLENCH_DEACTIVATED);
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

void handleBtnEvent(AceButton* /* button */, uint8_t eventType, uint8_t buttonState) {
  switch (eventType) {
    case AceButton::kEventClicked:
      Serial.println("Click");
      events.push(CHALLENGE_BUTTON_ACTIVATED);
      break;
    case AceButton::kEventLongPressed:
      Serial.println("Long_pressed");
      events.push(CHALLENGE_BUTTON_ACTIVATED);
      break;
  }
}

////////////////////////////////////////////////////////////////////////////////
//                                                                            //
// DIFFERENT VIBRATION PATTERNS FOR EACH ALARM AND TASK                       //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////
void chlngVibPatternUP() {
  drv.setWaveform(0, 1);
  drv.setWaveform(1, 123);
  drv.setWaveform(2, 122);
  drv.setWaveform(3, 122);
  drv.setWaveform(4, 121);
  drv.setWaveform(5, 120);
  drv.setWaveform(6, 119);
  drv.setWaveform(7, 0);
  Serial.println("Breath In");
  drv.go();
}

void chlngVibPatternDown() {
  drv.setWaveform(0, 119);
  drv.setWaveform(1, 120);
  drv.setWaveform(2, 121);
  drv.setWaveform(3, 121);
  drv.setWaveform(4, 122);
  drv.setWaveform(5, 122);
  drv.setWaveform(6, 123);
  drv.setWaveform(7, 0);
  Serial.println("Breath Out");
  drv.go();
}
void chlng_vib () {
  unsigned long starttime = millis();
  unsigned long endtime = starttime;

  while ((endtime - starttime) <= 30000) // do this loop for up to 1000mS
  {
    // code here
    chlngVibPatternUP();
    chlngVibPatternDown();
    endtime = millis();
  }
  events.push(CHALLENGE_BUTTON_ACTIVATED);
}

void challengeAlarm() {
  drv.setWaveform(0, 1);
  drv.setWaveform(1, 60);
  drv.setWaveform(2, 0);

  unsigned long starttime = millis();
  unsigned long endtime = starttime;

  while ((endtime - starttime) <= 5000) // do this loop for up to 1000mS
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

  while ((endtime - starttime) <= 1000) // do this loop for up to 1000mS
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

  while ((endtime - starttime) <= 5000) // do this loop for up to 1000mS
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
  Serial.println(F("Standby enter"));
}

void on_standby()
{
  check_triggers();
}

void on_logdata_enter() {
  Serial.println(F("LogData enter"));
}

void on_logdata() {
  Serial.println(F("On LogData"));
  logToSDcard();
  delay (20);
  transToBLE();
  events.push(LOG_DATA_TIMEOUT);
  check_triggers();
}

void on_logdata_exit() {
  myBPM = 0;
  Serial.println(F("LogData Finished"));
}

void on_challenge_enter()
{
  Serial.println(F("Challenge enter"));
  telemetryTimer.stop();
  myBPM = 0;
}

void on_challenge()
{
  //  Serial.println(F("On Challenge"));
  currHR();
  logToSDcard();
  transToBLE();
  chlng_vib();
  check_triggers();
}

void on_challenge_exit()
{
  vib_count = 0;
  telemetryTimer.start();
  Serial.println(F("Challenge exit"));
}

void on_challengealarm_enter()
{
  Serial.println(F("Challenge alarm enter"));
}

void on_challengealarm()
{
  Serial.println(F("On Challenge Alarm"));
  challengeAlarm();
  check_triggers();
}

void on_challengealarm_exit()
{
  vib_count = 0;
  Serial.println(F("Challenge alarm exit"));
}

void on_selfreport_enter()
{
  Serial.println(F("Selfreport enter"));
}

void on_selfreport()
{
  Serial.println(F("On Self Report!!"));
  events.push(LOG_DATA);
  check_triggers();
}

void on_selfreport_exit()
{
  Serial.println(F("Self Report Succeed!!"));
}

void on_stressalarm_enter()
{
  Serial.println(F("Stressalarm enter"));
}

void on_stressalarm()
{
  Serial.println(F("On Stress Alarm"));
  stressAlarm();
  check_triggers();
}

void on_stressalarm_exit()
{
  vib_count = 0;
  Serial.println(F("Stressalarm exit"));
}

void on_inactivityalarm_enter()
{
  Serial.println(F("Inactivity alarm enter"));
}

void on_inactivityalarm()
{
  Serial.println(F("On Inactivity Alarm"));
  inactivityAlarm();
  check_triggers();
}

void on_inactivityalarm_exit()
{
  vib_count = 0;
  Serial.println(F("Inactivity alarm exit"));
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
        Serial.println(myBPM);
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
  return step;
}

////////////////////////////////////////////////////////////////////////////////
//  Log Data: Functuon to log data into SD Card                               //
//  and Transmit the data over BLE                                            //                                                                            //
////////////////////////////////////////////////////////////////////////////////

void transToBLE() {

  Serial.println(F("Transmit Data to BLE Begin"));
  uint32_t ts = currTimestamp();

  SerialPort.print(F("t"));
  SerialPort.print(ts);
  SerialPort.print(comma);
  SerialPort.print(F("h"));
  SerialPort.print(myBPM);
  SerialPort.print(comma);
  SerialPort.print(F("s"));
  SerialPort.print(step);
  SerialPort.println();

  Serial.println(F("BLE Transmission Succeed"));
  //  return true;
}

void logToSDcard()
{

  uint32_t ts = currTimestamp();
  // open the file. note that only one file can be open at a time,
  // so you have to close this one before opening another.
  File dataFile = SD.open(fname, FILE_WRITE);

  if (dataFile)
  {
    // Timestamp
    dataFile.print(F("t"));
    dataFile.print(ts);
    dataFile.print(comma);
    // Datafields (HR and Steps)
    dataFile.print(F("h"));
    dataFile.print(myBPM);
    dataFile.print(comma);
    dataFile.print(F("s"));
    dataFile.print(step);
    // Newline at end of file
    dataFile.println();
    dataFile.close();
    Serial.println(F("SDCard Log Succeed"));
  }
  else
  {
    Serial.println(F("Error in opening the file on SD card!!"));
  }
}
