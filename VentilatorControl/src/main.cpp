//TODO: Refactor to have all includes in one include file (later)
// Libraries
#include <Arduino.h>
#include <Wire.h>
#include <AllSensors_DLHR.h>
#include <Servo.h>
#include <PID_v1.h>

// ESC Pulse Widths (using OTS hobby ESCs)
// TODO: Use later instead of Servo library
#define BLOWER_DRIVER__MIN_PULSE__MICROSECONDS 1000
#define BLOWER_DRIVER__MAX_PULSE__MICROSECONDS 2000

//TODO: Refactor to have all variables extremely modular and not hard coded (later)

// Pressure Variables
AllSensors_DLHR_L60D_8 gagePressure(&Wire);
float pressure_cmH20;

// Blower Variables
byte blower_pin = 5;
byte blower_speed;
Servo blower;

// Pressure Controlled Blower PID
double pressure_input, blower_output;
double CurrPressureSetpointCentimetersH2O;
// double Kp=0.0000011, Ki=0.00000005, Kd=0.0000000;
double Kp=1.0000000, Ki=0.0000000, Kd=0.0000000;
PID Pressure_PID(&pressure_input, &blower_output, &CurrPressureSetpointCentimetersH2O, Kp, Ki, Kd, DIRECT);

uint32_t CycleStartTimeFromSysClockMilliseconds;
uint32_t CurrTimeInCycleMilliseconds;
uint32_t InhaleRampDurationMilliseconds = 500;
uint32_t InhaleDurationMilliseconds = 1000;
uint32_t ExhaleDurationMilliseconds = 3000;
uint32_t BreathCycleDurationMilliseconds = InhaleDurationMilliseconds + ExhaleDurationMilliseconds;

double PipPressureCentimetersH2O = 5.0;
double PeepPressureCentimetersH2O = 35.0;

typedef enum{
    INHALE_RAMP,
    INHALE_HOLD,
    EXHALE
}BreathCycleStep;

BreathCycleStep CurrCycleStep;

void setup()
{
  // Need a simulated throttle LOW for at least 1 second delay for ESC to start properly
  blower.attach(blower_pin);
  blower.write(10);
  delay(10000);

  //TODO: Make for debugging ONLY
  Serial.begin(115200);

  //TODO: Change from I2C to SPI
  Wire.begin();
  gagePressure.setPressureUnit(AllSensors_DLHR::PressureUnit::IN_H2O);
  gagePressure.startMeasurement();

  // Set PID Mode to Automatic, may change later
  Pressure_PID.SetMode(AUTOMATIC);
  Pressure_PID.SetOutputLimits(0, 180);

  // What's this extra one for??
  // currBreathCycleState.CurrCycleStep = EXHALE;
  //TODO: set pressure setpoint to BREATHCYCLE__MINIMUM_PEEP__CENTIMETERSH2O
  //TODO: run PID control loop until things stabilize at min PEEP, then let the actual loop start

  CurrCycleStep = INHALE_HOLD;
  CurrTimeInCycleMilliseconds = 0;
  CycleStartTimeFromSysClockMilliseconds = millis();
}

void loop()
{
  // Get a pressure reading in units of cmH2O
  if(!gagePressure.readData(true))
  {
    gagePressure.startMeasurement();
    // delay(50);
    pressure_input = gagePressure.pressure * 2.53746;
    // Serial.print("Pressure: ");
    // Serial.println(pressure_input);
  }

  //TODO: Remove delay later, just for testing -- pressure sensor needs time
  // delay(50);

  // //TODO: Make sure clock overflow is handled gracefully
  // CurrTimeInCycleMilliseconds = millis()-CycleStartTimeFromSysClockMilliseconds;
  // if(CurrTimeInCycleMilliseconds <= InhaleRampDurationMilliseconds)
  // {
  //   CurrCycleStep = INHALE_RAMP;
  //   // Serial.println("INHALE_RAMP");
  // }
  // else if((InhaleRampDurationMilliseconds < CurrTimeInCycleMilliseconds) &&
  //         (CurrTimeInCycleMilliseconds <= InhaleDurationMilliseconds))
  // { 
  //   CurrCycleStep = INHALE_HOLD;
  //   // Serial.println("INHALE_HOLD");
  // }
  // else if((InhaleDurationMilliseconds < CurrTimeInCycleMilliseconds) &&
  //        (CurrTimeInCycleMilliseconds <= BreathCycleDurationMilliseconds))
  // {
  //   CurrCycleStep = EXHALE;
  //   // Serial.println("EXHALE");
  // }
  // else if(CurrTimeInCycleMilliseconds > BreathCycleDurationMilliseconds)
  // {
  //   CurrCycleStep = INHALE_RAMP;
  //   // Serial.println("INHALE_RAMP");
  //   CurrTimeInCycleMilliseconds = 0;
  //   CycleStartTimeFromSysClockMilliseconds = millis();
  // }

  // //Recompute Setpoints
  switch(CurrCycleStep)
  {
    case INHALE_RAMP:
      // calculate new setpoint based on linear ramp from PEEP pressure to PIP pressure over set duration
      // PRESSURE_SETPOINT(t) = t*(PIP/RAMP_DURATION)+PEEP
      CurrPressureSetpointCentimetersH2O = (((float)CurrTimeInCycleMilliseconds/(float)InhaleRampDurationMilliseconds)*PipPressureCentimetersH2O)+PeepPressureCentimetersH2O;
      Serial.println("INHALE_RAMP");
    break;
    case INHALE_HOLD:
      CurrPressureSetpointCentimetersH2O = PipPressureCentimetersH2O;
      Serial.println("INHALE_HOLD");
    break;
    case EXHALE:
      CurrPressureSetpointCentimetersH2O = PeepPressureCentimetersH2O;
      Serial.println("EXHALE");
    break;
  }

  blower.write(blower_output);

  // Comute Pressure PID
  Pressure_PID.Compute();

  Serial.println(pressure_input);
  Serial.println(CurrPressureSetpointCentimetersH2O);
}
