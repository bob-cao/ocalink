#include "includes.h"

void buzzerToggle (void)
{
  static unsigned long lastBuzzerToggle = 0;
  if( millis()-lastBuzzerToggle > 500 )
  {
    digitalWrite(BUZZER_PIN, buzzer_state);
    buzzer_state = !buzzer_state;
    lastBuzzerToggle = millis();
  }
}

// Found via regression analysis from static pressure testing of sample blower
// r^2 = 0.99 for this regression on the blower tested.
// see data at https://docs.google.com/spreadsheets/d/1GVmF7gMihPsArEmjk8MefS8RsMpl3gKHL-qg1a0rInc/edit?usp=sharing
double blowerPressureToBlowerSpeed (double pressure)
{
    double clampedpressure = clampf(pressure, 0, PIP_MAX_RECEIVE+MAX_BLOWER_PRESSURE_OFFSET);
    return clampf(sqrt(clampedpressure)*99.28637634 + clampedpressure*1.908460447 + -51.83083825,ADC_MIN_OUTPUT, ADC_MAX_OUTPUT);
}

bool incrementIsPositive = true;
uint32_t timeofLastIncrement = 0;
static BreathCycleStep blowerPreviousBreathCycleStep;
static BreathCycleStep previousBreathCycleStep;
static bool PipPressureReached = false;
static bool PeepPressureReached = false;

void writeBlowerSpeed(void)
{
  //debug only
  //Blower_PID.SetTunings(PinchValve_Kp,PinchValve_Ki,PinchValve_Kd);

  if((blowerPreviousBreathCycleStep == EXHALE_HOLD) && ((CurrCycleStep == INHALE_RAMP)||(CurrCycleStep == INHALE_HOLD)))
  {
    PipPressureReached = false;
  }

  double blowerOffsetLimit = CurrPressureSetpointCentimetersH2O*0.5;
  Blower_PID.SetOutputLimits(-blowerOffsetLimit,blowerOffsetLimit);
  Blower_PID.Compute();

  blower_speed = blowerPressureToBlowerSpeed(CurrPressureSetpointCentimetersH2O + blower_pressure_offset);

  if(pressure_reading>(CurrPressureSetpointCentimetersH2O*0.95))
  {
    PipPressureReached = true;
  }

  if(!PipPressureReached && ((CurrCycleStep == INHALE_RAMP)||(CurrCycleStep == INHALE_HOLD)))
  {
    analogWrite(BLOWER_SPEED_PIN,ADC_MAX_OUTPUT);
  }
  else
  {
    analogWrite(BLOWER_SPEED_PIN,(uint32_t)blower_speed);
  }
}

// Write percent openness to the pinch valve
// 0% = closed
// 100% = wide open
void writePinchValveOpenness (double opennessPercentage)
{
  long pinchValvePulseLengthMicroseconds = 
  map(opennessPercentage,
      MIN_PERCENTAGE,
      MAX_PERCENTAGE,
      PINCH_VALVE_DRIVER_FULL_CLOSE_PULSE_MICROSECONDS,
      PINCH_VALVE_DRIVER_FULL_OPEN_PULSE_MICROSECONDS);
  PinchValve.writeMicroseconds(pinchValvePulseLengthMicroseconds);
}






void pinchValveControl (void)
{
  // debug only
  //PinchValve_PID.SetTunings(PinchValve_Kp,PinchValve_Ki,PinchValve_Kd);
  /*
  if((millis()-timeofLastIncrement) >100)
  {
    pinch_valve_output_openness_in_percentage += incrementIsPositive?1:-1;
    timeofLastIncrement = millis();
  }
  if(pinch_valve_output_openness_in_percentage>=100.0)
  {
    pinch_valve_output_openness_in_percentage = 100.0;
    incrementIsPositive = false;
  }
  if(pinch_valve_output_openness_in_percentage<=0.0)
  {
    pinch_valve_output_openness_in_percentage = 0.0;
    incrementIsPositive = true;
  }
*/
  
  // reset the integrator at the beginning of each exhale cycle
  if((previousBreathCycleStep == INHALE_HOLD) && ((CurrCycleStep == EXHALE_RAMP)||(CurrCycleStep == EXHALE_HOLD)))
  {
    PeepPressureReached = false;
  }
  switch (CurrCycleStep)
  {
  case EXHALE_RAMP:
  //  pinch_valve_output_openness_in_percentage = 100.0;
  //  break;
  case EXHALE_HOLD:
    if(CurrPressureSetpointCentimetersH2O>pressure_reading)
    {
      PeepPressureReached = true;
    }
    if(PeepPressureReached)
    {
      pinch_valve_output_openness_in_percentage = 5.0;
    }
    else
    {
      pinch_valve_output_openness_in_percentage = 100.0;
    }
    
    break;
  case INHALE_HOLD:
  case INHALE_RAMP:
    pinch_valve_output_openness_in_percentage = 0.0;
    break;
  default:
    pinch_valve_output_openness_in_percentage = 100.0;
    // pinch_valve_output_openness_in_percentage = 0.0;
    break;
  }
  
  previousBreathCycleStep = CurrCycleStep;
  writePinchValveOpenness(pinch_valve_output_openness_in_percentage);  
}
