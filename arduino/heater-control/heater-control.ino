/* 
BOSTON UNIVERSITY ENG-ME692
HEATERBOARD CONTROL PROGRAM
AL LEVINE & ZEWEI WU

Designed for Arduino Nano 33 IOT
*/

//====================================================================================================
//LIBRARIES
#include <Wire.h>             //i2c library
#include "ArduinoLowPower.h"  //low power library
#include <AHTxx.h>            //temp sensor library

//PIN ASSIGNMENTS
const int pinHeat = 9;             //mosfet control pin
const int pinBatt = A6;            //battery monitor pin
const int pinRelay = A7;           //relay control pin
const int pinRelayInterrupt = 21;  //interrupt pin number for relay, see https://github.com/arduino/ArduinoCore-samd/blob/master/variants/nano_33_iot/variant.cpp
const int pinSDA = A4;             //i2c sda pin
const int pinSCL = A5;             //i2c scl pin
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
AHTxx aht21(AHTXX_ADDRESS_X38, AHT2x_SENSOR);  //sensor address, sensor type

//====================================================================================================
//SETUP
void setup() {

  pinMode(pinHeat, INPUT);
  pinMode(pinBatt, INPUT);
  pinMode(pinRelay, INPUT);

  Wire.begin();          //join i2c bus
  Serial.begin(115200);  //start serial for output
  Serial.println("Serial begin");

  while (aht21.begin() != true) {                                                      //initialize temp sensor
    Serial.println(F("AHT21 not connected or fail to load calibration coefficient"));  //(F()) save string to flash & keeps dynamic memory free
    delay(5000);
  }
  Serial.println(F("AHT21 OK"));

  LowPower.attachInterruptWakeup(pinRelayInterrupt, callback, FALLING);  // attaches relay pin to wake board and run interrupt function
}

//====================================================================================================
//LOOP
void loop() {
  voltCheck();
  if (timer(timeStart) <= timerMax) {
    pid();
  } else {
    LowPower.sleep();
  }
}

//==================================================
//FUNCTIONS
void callback() {  //interrupt function
  voltCheck();
  timeStart = millis();
}

void pid() {
  float tempCur = readTemp();     //read temperature from sensor
  float err = tempDes - tempCur;  //find error between current and desired temp
  Serial.print("Temp difference: ");
  Serial.println(err);
  float P = Kp * err + tempDes;                      //proportional control equation
  analogWrite(pinHeat, map(P, 0, tempDes, 0, 255));  //map proportional control to pwm signal
}

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

float readTemp() {
  float temp = aht21.readTemperature();  //read 6-bytes via I2C, takes 80 milliseconds
  if (temp != AHTXX_ERROR) {             //AHTXX_ERROR = 255, library returns 255 if error occurs
    Serial.print(temp);
    Serial.println(F(" +-0.3C"));
    return temp;  //return the temp value
  } else {
    printAhtStatus();                                                   //print temperature command status
    if (aht21.softReset() == true) Serial.println(F("reset success"));  //as the last chance to make it alive
    else Serial.println(F("reset failed"));
  }
}

void printAhtStatus()  // Print last command status for aht21, from library documentation
{
  switch (aht21.getStatus()) {
    case AHTXX_NO_ERROR:
      Serial.println(F("no error"));
      break;

    case AHTXX_BUSY_ERROR:
      Serial.println(F("sensor busy, increase polling time"));
      break;

    case AHTXX_ACK_ERROR:
      Serial.println(F("sensor didn't return ACK, not connected, broken, long wires (reduce speed), bus locked by slave (increase stretch limit)"));
      break;

    case AHTXX_DATA_ERROR:
      Serial.println(F("received data smaller than expected, not connected, broken, long wires (reduce speed), bus locked by slave (increase stretch limit)"));
      break;

    case AHTXX_CRC8_ERROR:
      Serial.println(F("computed CRC8 not match received CRC8, this feature supported only by AHT2x sensors"));
      break;

    default:
      Serial.println(F("unknown status"));
      break;
  }
}
