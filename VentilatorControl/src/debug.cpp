#include "includes.h"

void debugPrintCurrentValues (void)
{
    if( millis() % 50 == 0 )
    {
      Serial.print(pressure_system_input);
      Serial.print(" ");
      Serial.print(CurrPressureSetpointCentimetersH2O);
      Serial.print(" ");
      Serial.print(blower_speed);
      Serial.print(" ");
      Serial.print(venturiDifferentialPressureReading);
      Serial.println();
    }
}
