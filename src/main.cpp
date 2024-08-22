#include <Arduino.h>

#include <Wire.h>
#include <AHT10.h>
#include "PIDController.h"


uint8_t readStatus = 0;

#define fan_Pin A0            // Fan pin for heat dissipation;
#define heat_Resistor_Pin A1  // Heating resistor for heat;

//Set Control Assignment Variables
double Setpoint, TempValue, Output;

// The PID arguments
float kP, kI, kD; 

// Objects
PIDController pid; // Create an instance of the PID controller class, called "pid"

AHT10 myAHT10(AHT10_ADDRESS_0X38);

void setup()
{
  Serial.begin(115200);
  Serial.println();
  
  pinMode(fan_Pin, OUTPUT);
  pinMode(heat_Resistor_Pin, OUTPUT );
  digitalWrite(fan_Pin, HIGH);

  Setpoint = 37.5; //This is the process setpoint
  pid.begin();          // initialize the PID instance
  pid.setpoint(Setpoint);    // The "goal" the PID controller tries to "reach"
  pid.tune(kP, kI, kD);    // Tune the PID, arguments: kP, kI, kD
  pid.limit(0, 255);    // Limit the PID output between 0 and 255, this is important to get rid of integral windup!

  while (myAHT10.begin() != true)
  {
    //(F()) save string to flash & keeps dynamic memory free
    Serial.println(F("AHT10 not connected or fail to load calibration coefficient")); 
    delay(5000);
  }
  Serial.println(F("AHT10 OK"));

}

void loop()
{
  readStatus = myAHT10.readRawData(); // read 6 bytes from AHT10 over I2C

  if (readStatus != AHT10_ERROR)
  {
    TempValue = myAHT10.readTemperature(AHT10_USE_READ_DATA);
    Serial.print(F("Temperature: "));
    Serial.print(TempValue);
    Serial.println(F(" +-0.3C"));

    Output = pid.compute(TempValue);
    analogWrite(heat_Resistor_Pin, Output);

    Serial.print("Setpoint: "), Serial.println(Setpoint);
    Serial.print("Output: "), Serial.println(Output);
    
  }
  else
  {
    Serial.print(F("Failed to read - reset: "));
    Serial.println(myAHT10.softReset()); // reset 1-success, 0-failed
  }

  delay(10000); // recomended polling frequency 8sec..30sec
}