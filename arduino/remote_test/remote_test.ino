/* 
BOSTON UNIVERSITY ENG-ME692
HEATERBOARD CONTROL PROGRAM
AL LEVINE & ZEWEI WU

Designed for Arduino Nano 33 IOT
*/

//====================================================================================================
//LIBRARIES
#include <Wire.h>             //i2c library
#include <ArduinoLowPower.h>  //low power library
//#include <AHTxx.h>            //temp sensor library

//PIN ASSIGNMENTS
const int pinHeat = 9;             //mosfet control pin
const int pinBatt = A6;            //battery monitor pin
const int pinRelay = A7;           //relay control pin
const int pinRelayInterrupt = 21;  //interrupt pin number for relay, see https://github.com/arduino/ArduinoCore-samd/blob/master/variants/nano_33_iot/variant.cpp
const int pinSDA = A4;             //i2c sda pin
const int pinSCL = A5;             //i2c scl pin
const int sheetA = A2;             //sheet voltage
const int sheetV = A1;             //sheet current
//#define i2cTemp x  //i2c address of temp sensor, https://www.amazon.com/HiLetgo-Precision-Temperature-Humidity-2-0-5-5V/dp/B09KGW1G41

//VARIABLE ASSIGNMENTS
const float Kp = 1;
const int tempMax = 200;                    //celsius, max safe temperature for heater
const int tempDes = 50;                     //celsius, desired operating temperature for heater
const int vMin = 12;                        //volts, minimum safe voltage for battery operation
float tempCur = 0;                          //store temp result
unsigned long timeStart = 0;                //store time
unsigned long timerMax = 15 * (60 * 1000);  //max time for heater, [min]*[s/min]*[ms/s]

//Temp sensor setup
//AHTxx aht21(AHTXX_ADDRESS_X38, AHT2x_SENSOR);  //sensor address, sensor type

//====================================================================================================
//SETUP

bool toggle = false;

void setup() {

  pinMode(pinHeat, INPUT);
  pinMode(pinBatt, INPUT);
  pinMode(pinRelay, INPUT);
  pinMode(sheetV, INPUT);
  pinMode(sheetA, INPUT);
  pinMode(13, OUTPUT);

  Wire.begin();          //join i2c bus
  Serial.begin(115200);  //start serial for output
  Serial.println("Serial begin");

  LowPower.attachInterruptWakeup(pinRelayInterrupt, callback, RISING);  // attaches relay pin to wake board and run interrupt function
}

//====================================================================================================
//LOOP
void loop() {
  if (toggle == true) {
    digitalWrite(13, HIGH);
  }
  if (toggle == false) {
    digitalWrite(13, LOW);
  }
}

//==================================================
//FUNCTIONS
void callback() {  //interrupt function
  timeStart = millis();
  if (toggle == false) {
    toggle = 1;
  } else {
    toggle = 0;
  }
}

// void pid() {
//   float tempCur = readTemp();     //read temperature from sensor
//   float err = tempDes - tempCur;  //find error between current and desired temp
//   Serial.print("Temp difference: ");
//   Serial.println(err);
//   float P = Kp * err + tempDes;                      //proportional control equation
//   analogWrite(pinHeat, map(P, 0, tempDes, 0, 255));  //map proportional control to pwm signal
// }

void voltCheck() {                       //checks voltage of battery from 1/3 voltage divider
  float vCur = analogRead(pinBatt) * 3;  // voltage reading from (1/3) divider
  if (vCur <= vMin) {
    Serial.println("Low voltage. Going to sleep.");
    LowPower.sleep();
  } else {
    Serial.print("Voltage good. ");
    Serial.print(vCur);
    Serial.println(" volts.");
  }
}


float timer(unsigned long start) {  //function to find time passed since start time
  unsigned long startTime = start;
  unsigned long curTime = millis();
  return start - curTime;
}