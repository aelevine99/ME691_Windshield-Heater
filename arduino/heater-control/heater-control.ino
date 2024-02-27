/* 
BOSTON UNIVERSITY ENG-ME692
HEATERBOARD CONTROL PROGRAM
AL LEVINE & ZEWEI WU
*/

//==================================================
//LIBRARIES
#include <Wire.h>   //i2c library
#include <AHTxx.h>  //temp sensor library

//PINS
#define heatPwr D9  //mosfet control pin
#define batt A6     //battery monitor pin
#define relay A7    //relay control pin
#define pinSDA A4   //i2c
#define pinSCL A5   //i2c
#define i2cTemp x   //i2c address of temp sensor, https://www.amazon.com/HiLetgo-Precision-Temperature-Humidity-2-0-5-5V/dp/B09KGW1G41

//VARS
int tempMax 200;  //celsius, max safe temperature for heater
int tempDes 50;   //celsius, desired operating temperature for heater
int battMin 12;   //volts, minimum safe voltage for battery operation
float ahtVal;     //store temp result

//Temp sensor setup
AHTxx aht21(AHTXX_ADDRESS_X38, AHT2x_SENSOR);  //sensor address, sensor type

//==================================================
//SETUP
void setup() {
  Wire.begin();          //join i2c bus
  Serial.begin(115200);  //start serial for output

  while (aht20.begin() != true) {                                                      //initialize temp sensor
    Serial.println(F("AHT21 not connected or fail to load calibration coefficient"));  //(F()) save string to flash & keeps dynamic memory free
    delay(5000);
  }
  Serial.println(F("AHT21 OK"));
}

//==================================================
//LOOP
void loop() {
  readTemp();
}

//==================================================
//FUNCTIONS
float readTemp() {
  ahtVal = aht21.readTemperature();  //read 6-bytes via I2C, takes 80 milliseconds
  if (ahtValue != AHTXX_ERROR) {     //AHTXX_ERROR = 255, library returns 255 if error occurs
    Serial.print(ahtValue);
    Serial.println(F(" +-0.3C"));
  } else {
    printStatus();                                                      //print temperature command status
    if (aht20.softReset() == true) Serial.println(F("reset success"));  //as the last chance to make it alive
    else Serial.println(F("reset failed"));
  }
}

void printStatus() // Print last command status for aht21
{
  switch (aht20.getStatus())
  {
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
