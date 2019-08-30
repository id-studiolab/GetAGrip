//Nirav Malsattar
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
#define USE_ARDUINO_INTERRUPTS false
#define comma ','   // comma ','

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
uint32_t buf1;    //array buffer to store time data in char
char fname[10];   //array buffer to store filename in char

// Heartrate
byte samplesUntilReport;
const byte SAMPLES_PER_SERIAL_SAMPLE = 10;
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
  // Skip the first SAMPLES_PER_SERIAL_SAMPLE in the loop().
  samplesUntilReport = SAMPLES_PER_SERIAL_SAMPLE;
  if (pulseSensor.begin())
  {
    Serial.println(F("Heartrate Initialized"));
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

  Serial.println(F("Sensor srtup finished"));
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
  if (bt.available())
  {
    handleBLECommand((char)bt.read());
  }
  // If data is available from serial port,
  // print it to bt module.
  if (SerialPort.available())
  {
    Serial.println(F("BLE write"));
    bt.write((char)SerialPort.read());
  }
}

void handleBLECommand(char cmd)
{
  char numb = cmd;
  switch (numb)
  {
    case '1':
      //      events.push(CHALLENGE_DETECTED);
      Serial.print(F("Vib Challenge Detected"));
      break;
    case '2':
      //      events.push(CHALLENGE_BUTTON_ACTIVATED);
      Serial.print(F("Vib Challenge Activated"));
      break;
    case '3':
      //      events.push(INACTIVITY_DETECTED);
      Serial.print(F("Vib Inactivity Detected"));
      break;
    case '4':
      //      events.push(STRESS_DETECTED);
      Serial.print(F("Vib Stress Detected"));
      break;
  }
}

uint32_t currTimestamp() {
  DateTime now = clock.now();
  buf1 = now.unixtime();
  return buf1;
}

int currHR () {

  unsigned long starttime = millis();
  unsigned long endtime = starttime;

  while ((endtime - starttime) <= 100) // do this loop for up to 1000mS
  {
    if (pulseSensor.sawNewSample()) {

      if (--samplesUntilReport == (byte) 0) {
        samplesUntilReport = SAMPLES_PER_SERIAL_SAMPLE;

        int myBPM = pulseSensor.getBeatsPerMinute();
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

void bleCallback() {
  Serial.println(F("Transmit Data to BLE Begin"));
  uint32_t ts = currTimestamp();
  int hr = currHR();
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

  Serial.println(F("Transmit Data to BLE Finished"));
}
