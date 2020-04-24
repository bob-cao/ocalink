#include "includes.h"

// TODO: HIGH! FIX STATES, not using ramp times anymore
void cycleStateHandler (void)
{
  if( CurrCycleStep != IDLE )
  {
    pinMode(BATTERY_SHUTDOWN_PIN, INPUT);
    CurrTimeInCycleMilliseconds = millis()-CycleStartTimeFromSysClockMilliseconds;
    if(millis()-ControlLoopStartTimeMilliseconds > ControlLoopInitialStabilizationTimeMilliseconds)
    {
      // if(CurrTimeInCycleMilliseconds <= InhaleRampDurationMilliseconds)
      // {
      //   CurrCycleStep = INHALE_RAMP;
      // }
      if(CurrTimeInCycleMilliseconds <= InhaleDurationMilliseconds)
      { 
        CurrCycleStep = INHALE_HOLD;
      }
      else if((InhaleDurationMilliseconds < CurrTimeInCycleMilliseconds) &&
            (CurrTimeInCycleMilliseconds <= BreathCycleDurationMilliseconds))
      {
        // if( pressure_system_input > (PeepPressureCentimetersH2O + 5) )
        // {
        //   CurrCycleStep = EXHALE_RAMP;
        // }
        // else
        // {
          CurrCycleStep = EXHALE_HOLD;
        // }
      }
       else if(CurrTimeInCycleMilliseconds > BreathCycleDurationMilliseconds)
       {
         CurrCycleStep = INHALE_HOLD;
         breathCycleTimerReset();
       }
    }
    else // if idle == true
    {
      CurrCycleStep = EXHALE_HOLD;
      pinMode(BATTERY_SHUTDOWN_PIN, OUTPUT);
      digitalWrite(BATTERY_SHUTDOWN_PIN, LOW);
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
      //CurrPressureSetpointCentimetersH2O = (((float)CurrTimeInCycleMilliseconds/(float)InhaleRampDurationMilliseconds)*(PipPressureCentimetersH2O-PeepPressureCentimetersH2O))+PeepPressureCentimetersH2O;
   // break;
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
}
