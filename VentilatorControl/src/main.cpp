// ---------------------------VENTILATOR CONTROL MAIN-------------------------------- //

#include "includes.h"

void inits (void)
{
  // Initializations
  blowerEscInit();
  alarmLedInit();
  alarmsInit();
  pinchValveInit();
  pressureSensorsInit();
  pidInit();

  // Start cycle state in IDLE state
  CurrCycleStep = IDLE;

  // Serial initialization
  #if SYSTEM__SERIAL_DEBUG__STATEMACHINE
  Serial.begin(DEFAULT_BAUD_RATE);
  #endif
}

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

void cycleStateHandler (void)
{
  if( CurrCycleStep != IDLE )
  {
    CurrTimeInCycleMilliseconds = millis()-CycleStartTimeFromSysClockMilliseconds;
    if(millis()-ControlLoopStartTimeMilliseconds > ControlLoopInitialStabilizationTimeMilliseconds)
    {
      if(CurrTimeInCycleMilliseconds <= InhaleRampDurationMilliseconds)
      {
        CurrCycleStep = INHALE_RAMP;
      }
      else if((InhaleRampDurationMilliseconds < CurrTimeInCycleMilliseconds) &&
              (CurrTimeInCycleMilliseconds <= InhaleDurationMilliseconds))
      { 
        CurrCycleStep = INHALE_HOLD;
      }
      else if((InhaleDurationMilliseconds < CurrTimeInCycleMilliseconds) &&
            (CurrTimeInCycleMilliseconds <= BreathCycleDurationMilliseconds))
      {
        if( pressure_system_input > (PeepPressureCentimetersH2O + 5) )
        {
          CurrCycleStep = EXHALE_RAMP;
        }
        else
        {
          CurrCycleStep = EXHALE_HOLD;
        }
      }
      else if(CurrTimeInCycleMilliseconds > BreathCycleDurationMilliseconds)
      {
        CurrCycleStep = INHALE_RAMP;
        breath_cycle_timer_reset();
      }
    }
    else // if idle == true
    {
      CurrCycleStep = EXHALE_HOLD;
    }
  }
}

void cycleStateSetpointHandler(void)
{
  // TODO: HIGH breakout into separate file and rewrite for readability
  // TODO: MEDIUM put gains in a header, not in the source

  //Recompute Setpoints
  switch(CurrCycleStep)
  {
    case INHALE_RAMP:
      CurrPressureSetpointCentimetersH2O = (((float)CurrTimeInCycleMilliseconds/(float)InhaleRampDurationMilliseconds)*(PipPressureCentimetersH2O-PeepPressureCentimetersH2O))+PeepPressureCentimetersH2O;
    break;
    case INHALE_HOLD:
      CurrPressureSetpointCentimetersH2O = PipPressureCentimetersH2O;
    break;
    case EXHALE_RAMP:
    case EXHALE_HOLD:
    case IDLE:
    default:
      CurrPressureSetpointCentimetersH2O = PeepPressureCentimetersH2O;
    break;
  }

  CurrPressureSetpointCentimetersH2O = linearRemapSetpointCompensation(CurrPressureSetpointCentimetersH2O);
}

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
