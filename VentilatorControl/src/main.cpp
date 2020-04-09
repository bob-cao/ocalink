//TODO: Refactor to have all includes in one include file (later)
// Libraries
#include <Arduino.h>
#include <Wire.h>
#include <AllSensors_DLHR.h>
#include <Servo.h>
#include <PID_v1.h>

//TODO: Refactor to have all variables extremely modular and not hard coded (later)

// Pressure Variables
AllSensors_DLHR_L60D_8 gagePressure(&Wire);
float pressure_cmH20;

// Blower Variables
byte blower_pin = 5;
byte blower_speed;
Servo blower;

// Pressure Controlled Blower PID
double pressure_input, blower_output, pressure_setpoint;
double Kp=1, Ki=1, Kd=1;
PID Pressure_PID(&pressure_input, &blower_output, &pressure_setpoint, Kp, Ki, Kd, DIRECT);

uint32_t CycleStartTimeFromSysClockMilliseconds;
uint32_t CurrTimeInCycleMilliseconds;
uint32_t InhaleRampDurationMilliseconds;
uint32_t InhaleDurationMilliseconds;
uint32_t ExhaleDurationMilliseconds;
uint32_t BreathCycleDurationMilliseconds;

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

  CurrCycleStep = INHALE_RAMP;
  CurrTimeInCycleMilliseconds = 0;
  CycleStartTimeFromSysClockMilliseconds = millis();
}

void loop()
{
  // Get a pressure reading in units of cmH2O
  if(!gagePressure.readData(true))
  {
    gagePressure.startMeasurement();
    pressure_input = gagePressure.pressure * 2.53746;
    // Serial.print("Pressure: ");
    // Serial.print(pressure_input);
    // Serial.println();
  }

  //TODO: Remove delay later, just for testing -- pressure sensor needs time
  delay(50);

  // Get pressure reading and declare setpoint
  pressure_setpoint = 5.0;

  //TODO: Make sure clock overflow is handled gracefully
  CurrTimeInCycleMilliseconds = millis()-CycleStartTimeFromSysClockMilliseconds;
  if(CurrTimeInCycleMilliseconds <= InhaleRampDurationMilliseconds)
  {
    CurrCycleStep = INHALE_RAMP;
    Serial.println("INHALE_RAMP");
  }
  else if((InhaleRampDurationMilliseconds < CurrTimeInCycleMilliseconds) &&
          (CurrTimeInCycleMilliseconds <= InhaleDurationMilliseconds))
  { 
    CurrCycleStep = INHALE_HOLD;
    Serial.println("INHALE_HOLD");
  }
  else if((InhaleDurationMilliseconds < CurrTimeInCycleMilliseconds) &&
         (CurrTimeInCycleMilliseconds <= BreathCycleDurationMilliseconds))
  {
    CurrCycleStep = EXHALE;
    Serial.println("EXHALE");
  }
  else if(CurrTimeInCycleMilliseconds > BreathCycleDurationMilliseconds)
  {
    CurrCycleStep = INHALE_RAMP;
    Serial.println("INHALE_RAMP");
    CurrTimeInCycleMilliseconds = 0;
    CycleStartTimeFromSysClockMilliseconds = millis();
  }

  // //Recompute Setpoints
  // switch(currBreathCycleState.CurrCycleStep)
  // {
  //   case INHALE_RAMP:
  //     // calculate new setpoint based on linear ramp from PEEP pressure to PIP pressure over set duration
  //     // PRESSURE_SETPOINT(t) = t*(PIP/RAMP_DURATION)+PEEP
  //     currBreathCycleState.CurrPressureSetpointCentimetersH2O = (((float)currBreathCycleState.CurrTimeInCycleMilliseconds/(float)currBreathCycleSettings.InhaleRampDurationMilliseconds)*currBreathCycleSettings.PipPressureCentimetersH2O)+currBreathCycleSettings.PeepPressureCentimetersH2O;
  //   break;
  //   case INHALE_HOLD:
  //     currBreathCycleState.CurrPressureSetpointCentimetersH2O = currBreathCycleSettings.PipPressureCentimetersH2O;
  //   break;
  //   case EXHALE:
  //     currBreathCycleState.CurrPressureSetpointCentimetersH2O = currBreathCycleSettings.PeepPressureCentimetersH2O;
  //   break;
  // }

  // blower_speed = map(blower_output, 0, 100, 18, 180);
  blower.write(blower_output);

  // Comute Pressure PID
  Pressure_PID.Compute();
}
