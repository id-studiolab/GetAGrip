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

#define USE_ARDUINO_INTERRUPTS true
#include <PulseSensorPlayground.h> //Heartbeat Sensor: https://github.com/WorldFamousElectronics/PulseSensorPlayground/blob/master/resources/PulseSensor%20Playground%20Tools.md
#include <TaskScheduler.h> // Scheduling for arduino based task https://github.com/arkhipenko/TaskScheduler/wiki/API-Task
#include "arduino_bma456.h"  //Step Counter through Accelerometer : https://github.com/Seeed-Studio/Seeed_BMA456
#include <RTClib.h>
#include <Wire.h>
#include "PressureSensor.h"
#include <SparkFun_HM1X_Bluetooth_Arduino_Library.h> // BLE for library for BluetoothMate 4.0  https://github.com/sparkfun/SparkFun_HM1X_Bluetooth_Arduino_Library
#include <SoftwareSerial.h> //We are going to use software serial to communicate over Bluetooth through UART protocol


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

//Defining All the functions
void initializeAllSensor();
void hrCallback();
void acceCallback();
void bleCallback();
void vibCallback();
void on_clench(uint16_t pressure);
void on_release(uint16_t pressure);

//Bluetooth
HM1X_BT bt;
SoftwareSerial btSerial(3, 4);

// Heartrate
const int HEARTRATE_PIN = 3;
const int HEARTRATE_THRESHOLD = 550;
PulseSensorPlayground pulseSensor;

//Step Counter
uint32_t step = 0;
unsigned long prevStep = 0;

// Pressure
const int PRESSURE_OUTPUT = 8;
const int PRESSURE_INPUT  = A0;
const int CLENCH_THRESHOLD = 60;
uint16_t pressureval;
PressureSensor pressureSense(PRESSURE_OUTPUT, PRESSURE_INPUT, CLENCH_THRESHOLD, &on_clench, &on_release);

// Vibrator Pin
const int VIBRATOR_PIN = 5;

//Clock
RTC_DS1307 clock; //define a object of DS1307 class
char buf1[20]; //array buffer to store time data in char

// Scheduler
Scheduler tsMonitoring;
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

Task hrMonitoring(2.5 * TASK_SECOND, TASK_FOREVER, &hrCallback, &tsMonitoring, true);
Task acceMonitoring(TASK_IMMEDIATE, TASK_FOREVER, &acceCallback, &tsMonitoring, true);
Task pressMonitoring(TASK_IMMEDIATE, TASK_FOREVER, &on_clench, &tsMonitoring, true);

Task bleLog(2.5 * TASK_SECOND, TASK_FOREVER, &bleCallback, &tsLogData, true);

void setup() {
  // put your setup code here, to run once:

  Serial.begin(115200);
  Wire.begin();
  delay(500);

  initializeAllSensor(); //Activate all sensor
}

void loop() {
  // put your main code here, to run repeatedly:
  tsMonitoring.execute();
  tsLogData.execute();
}

void initializeAllSensor() {
  Serial.println("Initializing all Sensors....");

  //Initialize Bluetooth
  if (bt.begin(btSerial, 115200) == false) {
    Serial.println(F("Failed to connect to Bluetooth"));
    while (1) ;
  } else
    Serial.println("Bluetooth Initialized!");

  //Initialize Clock
  clock.begin();
  clock.adjust(DateTime(F(__DATE__), F(__TIME__)));
  Serial.println("Clock Initialized!");

  //Initialize HeartRate Sensor
  pulseSensor.analogInput(HEARTRATE_PIN);
  pulseSensor.setThreshold(HEARTRATE_THRESHOLD);
  if (pulseSensor.begin()) {
    Serial.println("HR Sensor Initialized!");  //This prints one time at Arduino power-up,  or on Arduino reset.
  }

  //Initialize Pressure Sensor
  pressureSense.begin();
  Serial.println("Pressure Sensor Initialized!");

  //Initialize Accelerometer as StepCounter
  bma456.initialize(RANGE_4G, ODR_1600_HZ, NORMAL_AVG4, CONTINUOUS);
  bma456.stepCounterEnable();
  Serial.println("Accelerometer/Step counter Initialized!");

  //Initialize Vibration Motor
  pinMode(VIBRATOR_PIN, OUTPUT);
  Serial.println("Vibration motor Initialized!");

  Serial.println("Initialization Complete");
  Serial.println("Start monitoring State");

  delay(100);
}

void hrCallback() {
  //Getting HR Value on every 10 seconds
  pulseSensor.resume();
  int bpm = pulseSensor.getBeatsPerMinute();
  if (pulseSensor.sawStartOfBeat()) {
    btSerial.print("BPM: ");
    btSerial.print(bpm);
    btSerial.print("\n");
    hrMonitoring.delay(200);
    pulseSensor.pause();
  }
//  pulseSensor.pause();
}

void acceCallback() {

}

void bleCallback() {
}

void vibCallback() {
}

void on_clench(uint16_t pressure) {
}

void on_release(uint16_t pressure) {
}
