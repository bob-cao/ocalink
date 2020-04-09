#include <Arduino.h>
#include <Wire.h>
#include <AllSensors_DLHR.h>

byte pressure_sensor_01 = 41;
AllSensors_DLHR_L60D_8 gagePressure(&Wire);

void setup()
{
  // put your setup code here, to run once:
  pinMode(LED_BUILTIN, OUTPUT);

  Serial.begin(115200);
  
  Wire.begin();
  gagePressure.setPressureUnit(AllSensors_DLHR::PressureUnit::PASCAL);

  gagePressure.startMeasurement();
}

void loop()
{
//  gagePressure.startMeasurement();
  gagePressure.readData(true);
  Serial.print("Pressure: ");
  Serial.print(gagePressure.pressure);
  Serial.print(" Temperature: ");
  Serial.println(gagePressure.temperature);

  delay(250);
}
