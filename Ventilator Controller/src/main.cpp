#include "includes.h"

BreathCycleState currBreathCycleState;
BreathCycleSettings currBreathCycleSettings;

void setup() {
  //TODO: setup pressure transducers
  //TODO: setup venturis
  //TODO: setup fan driver
  //TODO: setup PID loop
  //TODO: setup expiration valve driver
  
  //TODO: wait 5 seconds for sensor readings to stabilize
  delay(SYSTEM__STARTUP_DELAY__MILLISECONDS);
  
  // init current breath cycle settings
   currBreathCycleSettings.PeepPressureCentimetersH2O =  BREATHCYCLE__DEFAULT_PEEP__CENTIMETERSH2O;
   currBreathCycleSettings.PipPressureCentimetersH2O = BREATHCYCLE__DEFAULT_PIP__CENTIMETERSH2O;
   currBreathCycleSettings.InhaleDurationMilliseconds = BREATHCYCLE__DEFAULT_INHALE_DURATION__MILLISECONDS;
   currBreathCycleSettings.BreathCycleDurationMilliseconds = BREATHCYCLE__DEFAULT_CYCLE_DURATION__MILLISECONDS;
   currBreathCycleSettings.InhaleRampDurationMilliseconds = BREATHCYCLE__DEFAULT_INHALE_RAMP_DURATION__MILLISECONDS;
   currBreathCycleSettings.ExhaleDurationMilliseconds = BREATHCYCLE__DEFAULT_EXHALE_DURATION__MILLISECONDS;

  // Init state machine variable
  currBreathCycleState.CurrCycleStep = EXHALE;
  //TODO: set pressure setpoint to BREATHCYCLE__MINIMUM_PEEP__CENTIMETERSH2O
  //TODO: run PID control loop until things stabilize at min PEEP, then let the actual loop start

  currBreathCycleState.CurrCycleStep = INHALE_RAMP;
  currBreathCycleState.CurrTimeInCycleMilliseconds = 0;
  currBreathCycleState.CycleStartTimeFromSysClockMilliseconds = millis();

  // init serial debug 
  #if SYSTEM__SERIAL_DEBUG__STATEMACHINE
    Serial.begin(SYSTEM__SERIAL_DEBUG__BAUD_RATE); 
  #endif
}

void loop() {

  //TODO: get sensor readings

  //TODO: Check Alarms

  //Update Breath Cycle State
  currBreathCycleState.CurrTimeInCycleMilliseconds = millis()-currBreathCycleState.CycleStartTimeFromSysClockMilliseconds;
  if(currBreathCycleState.CurrTimeInCycleMilliseconds <= currBreathCycleSettings.InhaleRampDurationMilliseconds)
  {
    currBreathCycleState.CurrCycleStep = INHALE_RAMP;
  }
  else if((currBreathCycleSettings.InhaleRampDurationMilliseconds < currBreathCycleState.CurrTimeInCycleMilliseconds) &&
          (currBreathCycleState.CurrTimeInCycleMilliseconds <= currBreathCycleSettings.InhaleDurationMilliseconds))
  { 
    currBreathCycleState.CurrCycleStep = INHALE_HOLD;
  }
  else if((currBreathCycleSettings.InhaleDurationMilliseconds < currBreathCycleState.CurrTimeInCycleMilliseconds) &&
         (currBreathCycleState.CurrTimeInCycleMilliseconds <= currBreathCycleSettings.BreathCycleDurationMilliseconds))
  {
    currBreathCycleState.CurrCycleStep = EXHALE;
  }
  else if(currBreathCycleState.CurrTimeInCycleMilliseconds > currBreathCycleSettings.BreathCycleDurationMilliseconds)
  {
    currBreathCycleState.CurrCycleStep = INHALE_RAMP;
    currBreathCycleState.CurrTimeInCycleMilliseconds = 0;
    currBreathCycleState.CycleStartTimeFromSysClockMilliseconds = millis();
  }

  #if SYSTEM__SERIAL_DEBUG__STATEMACHINE
      Serial.print("Breath Cycle State Changed:");
      Serial.println(BreathCycleStepNames[currBreathCycleState.CurrCycleStep]);
  #endif

}