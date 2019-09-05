// By Nirav Malsattar
// niravmalsatter@gmail.com

#define comma ','   // comma ','
#define USE_ARDUINO_INTERRUPTS false
#define TCAADDR 0x5A
#define RTCADDR 0x68
#define SerialPort Serial1 // Abstract serial monitor debug port

#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#include <PulseSensorPlayground.h>
#include <SparkFun_HM1X_Bluetooth_Arduino_Library.h> // BLE for library for BluetoothMate 4.0  https://github.com/sparkfun/SparkFun_HM1X_Bluetooth_Arduino_Library
#include "arduino_bma456.h"                          //Step Counter through Accelerometer : https://github.com/Seeed-Studio/Seeed_BMA456
#include <RTClib.h>
#include "Fsm.h"
#include "EventQueue.h"
#include "PressureSensor.h"
#include "MillisTimer.h"
#include "Adafruit_DRV2605.h"

// PIN definitions
const int CHIPSELECT_PIN = 4;
const int CHALLENGE_BUTTON = 6;
const int HEARTRATE_PIN = 10;
const int PRESSURE_OUTPUT = 8;
const int PRESSURE_INPUT = A2;

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
void check_triggers();
void logDataCallback();
void contLoopCallback();
void logToSDcard();
void transToBLE();
void checkBLECmd();
void ChlngBtn();

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
const int PROGMEM HEARTRATE_THRESHOLD = 550;
PulseSensorPlayground pulseSensor;

//Step Counter (Accelerometer)
uint32_t step = 0;

// Vibration
Adafruit_DRV2605 drv;

// Pressure
const int PROGMEM CLENCH_THRESHOLD = 60;
void on_clench(uint16_t pressure);
void on_release(uint16_t pressure);
PressureSensor pressureSense(PRESSURE_OUTPUT, PRESSURE_INPUT, CLENCH_THRESHOLD, &on_clench, &on_release);

//Challenge Button
int buttonState = 0;

// Data acquisition
void telemetryTimer_handler(MillisTimer &mt);
MillisTimer telemetryTimer = MillisTimer(1000); // Take a snapshot of all the date once every 5 seconds

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
  //  //Initialize Clock
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
  pinMode(CHALLENGE_BUTTON, INPUT);
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
  fsm_main.run_machine();
  telemetryTimer.run();
  checkBLECmd();
//  ChlngBtn();
  pressureSense.run();
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

////////////////////////////////////////////////////////////////////////////////
//                                                                            //
// GENERAL I2C HANDLERS for getting value from each sensor with I2C bus.      //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////

void tcaselect(uint8_t i) {
  if (i < 7) return;
  Wire.beginTransmission(TCAADDR);
  Wire.write (1 << i);
  Wire.endTransmission();
  Wire.begin();
}

void tcaselect2(uint8_t i) {
  if (i < 7) return;
  Wire.beginTransmission(RTCADDR);
  Wire.write (1 << i);
  Wire.endTransmission();
  Wire.begin();
}

////////////////////////////////////////////////////////////////////////////////
//                                                                            //
// STATE MACHINE EVENT HANDLERS. These functions are specifically for the     //
// main state-machine which handles the bulk of the buisness.                 //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////
void on_standby()
{
  check_triggers();
}

void on_standby_enter()
{
  Serial.println(F("Standby enter"));
}

void on_logdata_enter() {
  Serial.println(F("LogData enter"));
}

void on_logdata() {
  logToSDcard();
  delay (100);
  transToBLE();
  events.push(LOG_DATA_TIMEOUT);
  check_triggers();
}

void on_logdata_exit() {
  Serial.println(F("LogData Finished"));
}

void on_challenge_enter()
{
  Serial.println(F("Challenge enter"));
}

void on_challenge()
{
  Serial.println(F("On Challenge"));

  for (int i = 0; i < 10;) {
    tcaselect(0);
    // put your main code here, to run repeatedly:
    drv.useLRA();
    drv.setWaveform(0, 52);
    drv.setWaveform(1, 0);
    drv.go();
    i++;
    delay(1000);
  }

  events.push(CHALLENGE_BUTTON_ACTIVATED);
  check_triggers();
}

void on_challenge_exit()
{
  Serial.println(F("Challenge exit"));
}

void ChlngBtn() {
  buttonState = digitalRead(CHALLENGE_BUTTON);
    if (buttonState == HIGH) {
    // turn challenge vibration on/off:
     events.push(CHALLENGE_BUTTON_ACTIVATED);
  }
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

  for (int i = 0; i < 3;) {
    tcaselect(0);
    // put your main code here, to run repeatedly:
    drv.useLRA();
    drv.setWaveform(0, 7);
    drv.setWaveform(1, 0);
    drv.go();
    i++;
    delay(1000);
  }

  events.push(STRESSALARM_TIMEOUT);
  check_triggers();
}

void on_stressalarm_exit()
{
  Serial.println(F("Stressalarm exit"));
}

void on_challengealarm_enter()
{
  Serial.println(F("Challenge alarm enter"));
}

void on_challengealarm()
{
  Serial.println(F("On Challenge Alarm"));

  for (int i = 0; i < 3;) {
    tcaselect(0);
    // put your main code here, to run repeatedly:
    drv.useLRA();
    drv.setWaveform(0, 47);
    drv.setWaveform(1, 0);
    drv.go();
    i++;
    delay(1000);
  }

  events.push(CHALLENGEALARM_TIMEOUT);
  check_triggers();
}

void on_challengealarm_exit()
{
  Serial.println(F("Challenge alarm exit"));
}

void on_inactivityalarm_enter()
{
  Serial.println(F("Inactivity alarm enter"));
}

void on_inactivityalarm()
{
  Serial.println(F("On Inactivity Alarm"));
  for (int i = 0; i < 3;) {
    tcaselect(0);
    // put your main code here, to run repeatedly:
    drv.useLRA();
    drv.setWaveform(0, 119);
    drv.setWaveform(1, 0);
    drv.go();
    i++;
    delay(1000);
  }
  events.push(INACTIVITYALARM_TIMEOUT);
  check_triggers();
}

void on_inactivityalarm_exit()
{
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
// and return each sensor value                                               //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////

uint32_t currTimestamp() {
  tcaselect2(0);
  clock.adjust(DateTime(F(__DATE__), F(__TIME__)));
  DateTime now = clock.now();
  buf1 = now.unixtime();

  sprintf(fname, "%02d%02d%02d.csv",  now.year(), now.month(), now.day());

  return buf1;
}

int currHR () {

  unsigned long starttime = millis();
  unsigned long endtime = starttime;

  int myBPM;
  //
  while ((endtime - starttime) <= 1000) // do this loop for up to 1000mS
  {
    if (pulseSensor.sawNewSample()) {

      if (--samplesUntilReport == (byte) 0) {
        samplesUntilReport = SAMPLES_PER_SERIAL_SAMPLE;

        myBPM = pulseSensor.getBeatsPerMinute();
        if (pulseSensor.sawStartOfBeat()) {
          Serial.print (myBPM);
          return myBPM;
        }
        else {
          Serial.println ("HR Failed");
          return 0;
        }
      }
    }
  }
}

uint32_t currSteps () {
  step = bma456.getStepCounterOutput();
  return step;
}

////////////////////////////////////////////////////////////////////////////////
//  Log Data: Functuon to log data into SD Card                               //
//  and Transmit the data over BLE                                            //                                                                            //
////////////////////////////////////////////////////////////////////////////////

void transToBLE() {
  //  Serial.println(F("Transmit Data to BLE Begin"));
  uint32_t ts = currTimestamp();
  int hr = currHR ();
  uint32_t steps = currSteps();

  SerialPort.print(F("t"));
  SerialPort.print(ts);
  SerialPort.print(comma);
  SerialPort.print(F("h"));
  SerialPort.print(hr);
  SerialPort.print(comma);
  SerialPort.print(F("s"));
  SerialPort.print(steps);
  SerialPort.println();

  Serial.println(F("BLE Transmission Succeed"));

  //  return true;
}

void logToSDcard()
{
  uint32_t ts = currTimestamp();
  int hr = currHR ();
  uint32_t steps = currSteps();

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
    dataFile.print(hr);
    dataFile.print(comma);
    dataFile.print(F("s"));
    dataFile.print(steps);
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
