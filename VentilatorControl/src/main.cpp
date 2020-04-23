// ---------------------------VENTILATOR CONTROL MAIN-------------------------------- //

#include "includes.h"

// void printPidSetpointAndCurrentValues (void)
// {
//   if( CurrCycleStep != IDLE )
//   {
//     if( millis() % 50 == 0 )
//     {
//       Serial.print(pressure_system_input);
//       Serial.print(" ");
//       switch(CurrCycleStep)
//       {
//         case INHALE_RAMP:
//           Serial.print(CurrPressureSetpointCentimetersH2O);
//         break;
//         case INHALE_HOLD:
//           Serial.print(PipPressureCentimetersH2O);
//         break;
//         case EXHALE_RAMP:
//         case EXHALE_HOLD:
//         case IDLE:
//         default:
//           Serial.print(PeepPressureCentimetersH2O);
//         break;
//       }
//       Serial.print(" ");
//       Serial.print(blower_output_speed_in_percentage);
//       Serial.print(" ");
//       Serial.print(venturiDifferentialPressureReading);
//       Serial.println();
//     }
//   }
// }

void setup()
{
  inits();

  breath_cycle_timer_reset(true);
}

void loop()
{
  pressure_system_input = getPressureReadings();

  cycleStateHandler();

  cycleStateSetpointHandler();

  pinchValveControl();

  writeBlowerSpeed();

  // printPidSetpointAndCurrentValues();

  alarmsHandler();

  alarmsVisualAudioHandler();

  computeSerialReceive();

  computeSerialSend();
}
