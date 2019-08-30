// By Richard Bekking & Nirav Malsattar
// richard@electronicsdesign.nl
// niravmalsatter@gmail.com

// #define _TASK_TIMECRITICAL      // Enable monitoring scheduling overruns
#define _TASK_SLEEP_ON_IDLE_RUN // Enable 1 ms SLEEP_IDLE powerdowns between tasks if no callback methods were invoked during the pass
// #define _TASK_STATUS_REQUEST    // Compile with support for StatusRequest functionality - triggering tasks on status change events in addition to time only
// #define _TASK_WDT_IDS           // Compile with support for wdt control points and task ids
// #define _TASK_LTS_POINTER       // Compile with support for local task storage pointer
#define _TASK_PRIORITY          // Support for layered scheduling priority
// #define _TASK_MICRO_RES         // Support for microsecond resolution
// #define _TASK_STD_FUNCTION      // Support for std::function (ESP8266 and ESP32 ONLY)
#define _TASK_DEBUG             // Make all methods and variables public for debug purposes
// #define _TASK_INLINE            // Make all methods "inline" - needed to support some multi-tab, multi-file implementations
#define _TASK_TIMEOUT           // Support for overall task timeout
// #define _TASK_OO_CALLBACKS      // Support for dynamic callback method binding

#define SerialPort Serial1 // Abstract serial monitor debug port

#define USE_ARDUINO_INTERRUPTS true
#include <PulseSensorPlayground.h>
#include <SparkFun_HM1X_Bluetooth_Arduino_Library.h> // BLE for library for BluetoothMate 4.0  https://github.com/sparkfun/SparkFun_HM1X_Bluetooth_Arduino_Library
#include <TaskScheduler.h> // Scheduling for arduino based task https://github.com/arkhipenko/TaskScheduler/wiki/API-Task
#include "arduino_bma456.h"  //Step Counter through Accelerometer : https://github.com/Seeed-Studio/Seeed_BMA456
//#include <SD.h>
#include <Wire.h>
#include <RTClib.h>

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
const int CHIPSELECT_PIN  = 4;
const int HEARTRATE_PIN = 10;
const int PRESSURE_OUTPUT = 8;
const int PRESSURE_INPUT  = A2;

void initBLE();
void initClk();
void initHR();
void initAcce();

void fbleCommCallback();
void bleCallback();

//Bluetooth
HM1X_BT bt;

// Realtime Clock
RTC_DS1307 clock; //define a object of DS1307 class
char buf1[10]; //array buffer to store time data in char

// Heartrate
const int PROGMEM HEARTRATE_THRESHOLD = 550;
PulseSensorPlayground pulseSensor;

//Step Counter (Accelerometer)
uint32_t step = 0;

Scheduler tsLogData;

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

//Initialization functions of all components
void initBLE() {
  if (bt.begin(SerialPort, 115200) == false) {
    Serial.println(F("Failed to connect to Bluetooth"));
    while (1) ;
  } else
    Serial.println(F("Bluetooth Initialized!"));
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

void initAcce() {
  //Initialize Accelerometer as StepCounter
  bma456.initialize(RANGE_4G, ODR_1600_HZ, NORMAL_AVG4, CONTINUOUS);
  bma456.stepCounterEnable();
  Serial.println(F("Accelerometer/Step counter Initialized!"));
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
  initHR();
  initClk();
  initAcce();
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
}

void fbleCommCallback() {
  // If data is available from bt module,
  // print it to serial port
  if (bt.available()) {
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
}
