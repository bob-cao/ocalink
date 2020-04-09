#include <Arduino.h>
#include <Wire.h>
#include <AllSensors_DLHR.h>
#include <Servo.h>
#include <PID_v1.h>

AllSensors_DLHR_L60D_8 gagePressure(&Wire);
float pressure_cmH20;

byte blower_pin = 5;
byte blower_speed;
byte blower_speed_percentage;
Servo blower;

// PID Pressure_PID(&Input, &Output, &Setpoint,2,5,1, DIRECT);

void setup()
{
  blower.attach(blower_pin);
  blower.write(10);
  delay(10000);

  Serial.begin(115200);

  Wire.begin();
  gagePressure.setPressureUnit(AllSensors_DLHR::PressureUnit::IN_H2O);

  gagePressure.startMeasurement();

  // myPID.SetMode(AUTOMATIC);
}

void loop()
{
  if(!gagePressure.readData(true))
  {
    gagePressure.startMeasurement();
    pressure_cmH20 = gagePressure.pressure * 2.53746;
    Serial.print("Pressure: ");
    Serial.print(pressure_cmH20);
    Serial.println();
  }

  delay(50);

//  if(pressure_cmH20 <= 20)
//  {
//    blower_speed_percentage = 2;  //~20cmH2o
//  }
//
//  else if(pressure_cmH20 > 20 && pressure_cmH20 <= 30)
//  {
//    blower_speed_percentage = 7;  //~30cmH2o
//  }

  blower_speed = map(blower_speed_percentage, 0, 100, 18, 180);
  blower.write(blower_speed);

  // Input = gagePressure.pressure * 2.53746;
  // Pressure_PID.Compute();
  // Setpoint = 20;
  // blower.write(Output);
}
