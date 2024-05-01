/* 
BOSTON UNIVERSITY ENG-ME692
HEATERBOARD CONTROL PROGRAM
AL LEVINE & ZEWEI WU

Designed for Arduino Nano 33 IOT
*/

//====================================================================================================
//LIBRARIES
#include <Wire.h>             // i2c library
#include <ArduinoLowPower.h>  // low power library
//#include <AHTxx.h>            // temp sensor library https://github.com/enjoyneering/AHTxx

//PIN ASSIGNMENTS
const int pinSDA = A4;             // i2c sda pin
const int pinSCL = A5;             // i2c scl pin
const int pinHeat = 9;             // mosfet control pin
const int pinBatt = A6;            // battery monitor pin
const int pinRelay = A7;           // relay control pin
const int pinRelayInterrupt = 21;  // interrupt pin number for relay, see https://github.com/arduino/ArduinoCore-samd/blob/master/variants/nano_33_iot/variant.cpp
const int pinSheetV = A2;          // sheet voltage
const int pinSheetI = A1;          // sheet current

//VARIABLE ASSIGNMENTS
const int tempMax = 50;                      // celsius, max safe temperature for heater
const int tempMin = 20;                      // celsius, desired operating temperature for heater
float tempCur = 0;                           // store temp result
const float tcr = -0.00055;                  // temperature coefficient of resistance for ito film, (Ω/°C)
const int vMin = 12;                         // volts, minimum safe voltage for battery operation
unsigned long timeStart = 0;                 // store time
unsigned long timerMax = 0.5 * (60 * 1000);  // max time for heater, [min]*[s/min]*[ms/s]=[min]

float vDiv = 0.1658;  // voltage divider value

bool running = false;

//Temp sensor setup
//AHTxx aht21(AHTXX_ADDRESS_X38, AHT2x_SENSOR);  //sensor address, sensor type

//====================================================================================================
//SETUP
void setup() {
  pinMode(pinHeat, INPUT);
  pinMode(pinBatt, INPUT);
  pinMode(pinRelay, INPUT);
  pinMode(pinSheetV, INPUT);
  pinMode(pinSheetI, INPUT);

  Wire.begin();        // join i2c bus
  Serial.begin(9600);  // start serial for output
  while (!Serial) {
    ;  // wait for serial port to connect. Needed for native USB port only
  }
  Serial.println("Serial begin");

  LowPower.attachInterruptWakeup(pinRelayInterrupt, callback, RISING);  // attaches relay pin to wake board and run interrupt function

  timeStart = millis();
}

//====================================================================================================
//LOOP
void loop() {
  thermostat();
}

//==================================================
//FUNCTIONS
void callback() {  //interrupt function
  Serial.println("remote button registered");
  if (running == false) {
    Serial.println("Heater ON");
    digitalWrite(LED_BUILTIN, HIGH);
    running = true;
  }
  else
    Serial.println("Heater OFF");
    digitalWrite(LED_BUILTIN, LOW);
    running = false;
}

void thermostat() {
  if (running == true) {
    analogWrite(pinHeat, 200);
  } else {
    analogWrite(pinHeat, 0);
  }
}