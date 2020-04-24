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
    return clampf(sqrt(clampedpressure)*105.0738528 + clampedpressure*1.042164472 + -32.32704615,ADC_MIN_OUTPUT, ADC_MAX_OUTPUT);
}

bool incrementIsPositive = true;
uint32_t timeofLastIncrement = 0;

void writeBlowerSpeed(void)
{
  // debug only
  Blower_PID.SetTunings(Blower_Kp,Blower_Ki,Blower_Kd);

  double blowerOffsetLimit = CurrPressureSetpointCentimetersH2O*0.5;
  Blower_PID.SetOutputLimits(-blowerOffsetLimit,blowerOffsetLimit);
  Blower_PID.Compute();

  blower_speed = blowerPressureToBlowerSpeed(CurrPressureSetpointCentimetersH2O + blower_pressure_offset);

  analogWrite(BLOWER_SPEED_PIN,(uint32_t)blower_speed);
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
  switch (CurrCycleStep)
  {
  case EXHALE_RAMP:
    pinch_valve_output_openness_in_percentage = 100.0;
    break;
  case EXHALE_HOLD:
    PinchValve_PID.SetTunings(PinchValve_Kp,PinchValve_Ki,PinchValve_Kd);
    PinchValve_PID.Compute();
    break;
  case INHALE_HOLD:
  case INHALE_RAMP:
    pinch_valve_output_openness_in_percentage = 0.0;
    break;
  default:
    pinch_valve_output_openness_in_percentage = 100.0;
    break;
  }

  writePinchValveOpenness(pinch_valve_output_openness_in_percentage);  
}
