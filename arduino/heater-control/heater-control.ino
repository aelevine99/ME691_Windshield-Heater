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
const int pinSDA = A4;             //i2c sda pin
const int pinSCL = A5;             //i2c scl pin
const int pinHeat = 9;             //mosfet control pin
const int pinBatt = A6;            //battery monitor pin
const int pinRelay = A7;           //relay control pin
const int pinRelayInterrupt = 21;  //interrupt pin number for relay, see https://github.com/arduino/ArduinoCore-samd/blob/master/variants/nano_33_iot/variant.cpp
const int pinSheetV = A2;          //sheet voltage
const int pinSheetI = A1;          //sheet current


//VARIABLE ASSIGNMENTS
const int tempMax = 50;                     //celsius, max safe temperature for heater
const int tempMin = 20;                     //celsius, desired operating temperature for heater
float tempCur = 0;                          //store temp result
const float tcr = -0.00055;                 //temperature coefficient of resistance for ito film, (Ω/°C)
const int vMin = 12;                        //volts, minimum safe voltage for battery operation
unsigned long timeStart = 0;                //store time
unsigned long timerMax = 15 * (60 * 1000);  //max time for heater, [min]*[s/min]*[ms/s]=[min]

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

  Wire.begin();          //join i2c bus
  Serial.begin(115200);  //start serial for output
  Serial.println("Serial begin");

  // while (aht21.begin() != true) {                                                      //initialize temp sensor
  //   Serial.println(F("AHT21 not connected or fail to load calibration coefficient"));  //(F()) save string to flash & keeps dynamic memory free
  //   delay(5000);
  // }
  // Serial.println(F("AHT21 OK"));

  LowPower.attachInterruptWakeup(pinRelayInterrupt, callback, RISING);  // attaches relay pin to wake board and run interrupt function
}

//====================================================================================================
//LOOP
void loop() {
  voltCheck();
  if (timer(timeStart) <= timerMax) {
    thermostat();
  } else {
    digitalWrite(LED_BUILTIN, LOW);
    LowPower.sleep();
  }
}

//==================================================
//FUNCTIONS
void callback() {  //interrupt function
  voltCheck();
  timeStart = millis();
  digitalWrite(LED_BUILTIN, HIGH);
}

void thermostat() {
  float tempCur = readTemp();  //read temperature from sensor
  if (tempCur > tempMax) {
    analogWrite(pinHeat, 0);
  } else if (tempCur < tempMin) {
    analogWrite(pinHeat, 255);
  } else {
    analogWrite(pinHeat, 100);
  }
}

void voltCheck() {                       //checks voltage of battery from 1/3 voltage divider
  float vCur = analogRead(pinBatt) * 3;  // voltage reading from (1/3) divider
  Serial.print("Input V: ");
  Serial.println(vCur);
  if (vCur <= vMin) {
    Serial.println("Low voltage. Going to sleep.");
    LowPower.sleep();
  };
}

float readTemp() {
  float sheetI = analogRead(pinSheetI) * 2;  //v_cc = v_iout * 2
  float sheetV = analogRead(pinSheetV) * 3;  // (3.32) / (3.32+6.65) voltage divider
  float sheetR = sheetV / sheetI;            // ohm's law V=IR
  float Tcur = 20 + tcr * (sheetR - 4.8);    // res at room temp (20 °C) is 4.8 ohms
  Serial.print("V: ");
  Serial.print(sheetV);
  Serial.print("\t I: ");
  Serial.print(sheetI);
  Serial.print("\t R: ");
  Serial.print(sheetR);
  Serial.print("\t Temp: ");
  Serial.println(Tcur);
  return Tcur;
}

float timer(unsigned long start) {  //function to find time passed since start time
  unsigned long startTime = start;
  unsigned long curTime = millis();
  return start - curTime;
}


// float readTemp() {
//   float temp = aht21.readTemperature();  //read 6-bytes via I2C, takes 80 milliseconds
//   if (temp != AHTXX_ERROR) {             //AHTXX_ERROR = 255, library returns 255 if error occurs
//     Serial.print(temp);
//     Serial.println(F(" +-0.3C"));
//     return temp;  //return the temp value
//   } else {
//     printAhtStatus();                                                   //print temperature command status
//     if (aht21.softReset() == true) Serial.println(F("reset success"));  //as the last chance to make it alive
//     else Serial.println(F("reset failed"));
//   }
//}

// void printAhtStatus()  // Print last command status for aht21, from library documentation
// {
//   switch (aht21.getStatus()) {
//     case AHTXX_NO_ERROR:
//       Serial.println(F("no error"));
//       break;

//     case AHTXX_BUSY_ERROR:
//       Serial.println(F("sensor busy, increase polling time"));
//       break;

//     case AHTXX_ACK_ERROR:
//       Serial.println(F("sensor didn't return ACK, not connected, broken, long wires (reduce speed), bus locked by slave (increase stretch limit)"));
//       break;

//     case AHTXX_DATA_ERROR:
//       Serial.println(F("received data smaller than expected, not connected, broken, long wires (reduce speed), bus locked by slave (increase stretch limit)"));
//       break;

//     case AHTXX_CRC8_ERROR:
//       Serial.println(F("computed CRC8 not match received CRC8, this feature supported only by AHT2x sensors"));
//       break;

//     default:
//       Serial.println(F("unknown status"));
//       break;
//   }
// }
