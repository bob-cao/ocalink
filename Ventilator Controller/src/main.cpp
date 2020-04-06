#include "includes.h"

// Control loop State variables
#pragma region
BreathCycleState currBreathCycleState;
BreathCycleSettings currBreathCycleSettings;
SensorReading currSensorStates;
ActuatorState currActuatorStates;
#pragma endregion

// PID Loop Object instantiations
#pragma region
PID blowerPid(&currSensorStates.PatientCircuitPressureCentimetersH2O,
              &currActuatorStates.blowerPowerLevelPercent,  
              &currBreathCycleState.CurrPressureSetpointCentimetersH2O,
              BLOWER_PID__DEFAULT_PROPORTIONAL_GAIN,
              BLOWER_PID__DEFAULT_INTEGRAL_GAIN,
              BLOWER_PID__DEFAULT_DERIVATIVE_GAIN,
              DIRECT);

PID expirationValvePid(&currSensorStates.PatientCircuitPressureCentimetersH2O,
              &currActuatorStates.blowerPowerLevelPercent,  
              &currBreathCycleState.CurrPressureSetpointCentimetersH2O,
              BLOWER_PID__DEFAULT_PROPORTIONAL_GAIN,
              BLOWER_PID__DEFAULT_INTEGRAL_GAIN,
              BLOWER_PID__DEFAULT_DERIVATIVE_GAIN,
              DIRECT);
#pragma endregion

// Pressure Transducer Object instantiation
#pragma region
I2cMultiplexer i2cMultiplexer(I2C_MULTIPLEXER__ADDRESS);

PressureTransducer oxygenVenturiDifferentialPressure;
PressureTransducer totalFlowVenturiDifferentialPressure;
PressureTransducer blowerOutletGagePressure;
PressureTransducer patientCircuitGagePressure;

#pragma endregion

void setup() {
  // initialize communications
  #if SYSTEM__SERIAL_DEBUG__STATEMACHINE
    Serial.begin(SYSTEM__SERIAL_DEBUG__BAUD_RATE); 
  #endif
    Wire.begin();
  
  //Setup pressure transducers
  oxygenVenturiDifferentialPressure = PressureTransducer(&Wire, &i2cMultiplexer, DLHR_OXYGEN_VENTURI, &currSensorStates.OxygenVenturiDifferentialPressureCentimetersH20);
  totalFlowVenturiDifferentialPressure = PressureTransducer(&Wire, &i2cMultiplexer, DLHR_TOTAL_VENTURI, &currSensorStates.TotalFlowVenturiDifferentialPressureCentimetersH20);
  blowerOutletGagePressure = PressureTransducer(&Wire, &i2cMultiplexer, DLHR_BLOWER_OUTLET_GAGE, &currSensorStates.BlowerOutletPressureCentimetersH20);
  patientCircuitGagePressure = PressureTransducer(&Wire, &i2cMultiplexer, DLHR_PATIENT_CIRCUIT_GAGE, &currSensorStates.PatientCircuitPressureCentimetersH2O);

  //Setup venturis
  

  //TODO: setup fan driver
  //TODO: setup expiration valve driver
  
  // setup PID loop parameters

    blowerPid.SetOutputLimits(BLOWER_PID__MIN_CONTROL_OUTPUT,BLOWER_PID__MAX_CONTROL_OUTPUT);
    blowerPid.SetControllerDirection(BLOWER_PID__CONTROL_DIRECTION);
    blowerPid.SetSampleTime(BLOWER_PID__SAMPLE_TIME);

    expirationValvePid.SetOutputLimits(EXPIRATION_VALVE_PID__MIN_CONTROL_OUTPUT,EXPIRATION_VALVE_PID__MAX_CONTROL_OUTPUT);
    expirationValvePid.SetControllerDirection(EXPIRATION_VALVE_PID__CONTROL_DIRECTION);
    expirationValvePid.SetSampleTime(EXPIRATION_VALVE_PID__SAMPLE_TIME);
  


  //TODO: wait 5 seconds for sensor readings to stabilize
  delay(SYSTEM_TIMINGS__STARTUP_DELAY__MILLISECONDS);
  
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

}

void loop() {

  //TODO: get sensor readings

  //Update Breath Cycle State
  #if SYSTEM__SERIAL_DEBUG__STATEMACHINE
  BreathCycleStep prevCycleStep = currBreathCycleState.CurrCycleStep;
  #endif

  //TODO: Make sure clock overflow is handled gracefully
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
    if(prevCycleStep != currBreathCycleState.CurrCycleStep)
    {
      Serial.print("Breath Cycle State Changed to:");
      Serial.println(BreathCycleStepNames[currBreathCycleState.CurrCycleStep]);
    }
  #endif

  //Recompute Setpoints
  switch(currBreathCycleState.CurrCycleStep)
  {
    case INHALE_RAMP:
      // calculate new setpoint based on linear ramp from PEEP pressure to PIP pressure over set duration
      // PRESSURE_SETPOINT(t) = t*(PIP/RAMP_DURATION)+PEEP
      currBreathCycleState.CurrPressureSetpointCentimetersH2O = (((float)currBreathCycleState.CurrTimeInCycleMilliseconds/(float)currBreathCycleSettings.InhaleRampDurationMilliseconds)*currBreathCycleSettings.PipPressureCentimetersH2O)+currBreathCycleSettings.PeepPressureCentimetersH2O;
    break;
    case INHALE_HOLD:
      currBreathCycleState.CurrPressureSetpointCentimetersH2O = currBreathCycleSettings.PipPressureCentimetersH2O;
    break;
    case EXHALE:
      currBreathCycleState.CurrPressureSetpointCentimetersH2O = currBreathCycleSettings.PeepPressureCentimetersH2O;
    break;
  }

  //Recompute PID Loop outputs
  blowerPid.Compute();
  expirationValvePid.Compute();

  //TODO: Push results of PID recompute to Blower, expiration valve

  //TODO: Check Alarms

  //TODO: Push Alarm States to GUI as soon as they occur


  //Push sensor states to GUI and check for user input every SYSTEM__GUI_UPDATE_PERIOD__MILLISECONDSms
  //Trigger if millis() is within a window since there's no guarantee we run the loop on the exact millisecond where millis%x = 0
  if( millis()%SYSTEM_TIMINGS__GUI_UPDATE_PERIOD__MILLISECONDS <= SYSTEM_TIMINGS__GUI_UPDATE_WINDOW__MILLISECONDS )
  {
    //TODO: Update GUI here
    //TODO: Check GUI task to see if anything's happened
  }

}