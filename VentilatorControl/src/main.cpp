//TODO: Refactor to have all includes in one include file (later)
// Libraries
#include <Arduino.h>
#include <Wire.h>
#include <AllSensors_DLHR.h>
#include <Servo.h>
#include <PID_v1.h>

// ESC Pulse Widths (using OTS hobby ESCs)
// TODO: Use later instead of Servo library
// #define BLOWER_DRIVER__MIN_PULSE__MICROSECONDS 1000
// #define BLOWER_DRIVER__MAX_PULSE__MICROSECONDS 2000

//TODO: Refactor to have all variables extremely modular and not hard coded (later)

// Pressure Variables
AllSensors_DLHR_L60D_8 gagePressure(&Wire);
float pressure_cmH20;
double expiration_offset = 3.0;
double expiration_hysteresis = 0.25;

//Solenoid
byte solenoid_pin = 4;

// Blower Variables
byte blower_pin = 5;
byte blower_speed;
//TODO: Temporary measure, Servo library only uses 8b, pulse width can be ~10b
Servo blower;

// Pressure Controlled Blower PID
double pressure_input, blower_output;
double CurrPressureSetpointCentimetersH2O;
double Kp=1.000000, Ki=1.000000, Kd=0.0000000;
PID Pressure_PID(&pressure_input, &blower_output, &CurrPressureSetpointCentimetersH2O, Kp, Ki, Kd, DIRECT);

uint32_t CycleStartTimeFromSysClockMilliseconds;
uint32_t CurrTimeInCycleMilliseconds;
uint32_t InhaleRampDurationMilliseconds = 1000;
uint32_t InhaleDurationMilliseconds = 3000;
uint32_t ExhaleDurationMilliseconds = 10000;
uint32_t BreathCycleDurationMilliseconds = InhaleDurationMilliseconds + ExhaleDurationMilliseconds;
uint32_t ControlLoopInitialStabilizationTImeMilliseconds = 1000;;
uint32_t ControlLoopStartTimeMilliseconds;
uint32_t TimeOfLastSolenoidToggleMilliseconds = 0;
uint32_t SolenoidMinimumDwellTimeMilliseconds = 250;

double PeepPressureCentimetersH2O = 8.000000;
double PipPressureCentimetersH2O = 50.500000;

typedef enum{
    INHALE_RAMP,
    INHALE_HOLD,
    EXHALE,
    IDLE
}BreathCycleStep;

BreathCycleStep CurrCycleStep;

String string_from_pi;
byte pi_string_index_comma;
byte pi_string_index_asterik;
String property_name;
double value;

void setup()
{
  pinMode(solenoid_pin, OUTPUT);
  digitalWrite(solenoid_pin, LOW);

  // Need a simulated throttle LOW for at least 1 second delay for ESC to start properly
  blower.attach(blower_pin);
  blower.write(10);
  delay(2000);

  //TODO: Make for debugging ONLY
  Serial.begin(115200);

  //TODO: Change from I2C to SPI
  Wire.begin();
  gagePressure.setPressureUnit(AllSensors_DLHR::PressureUnit::IN_H2O);
  gagePressure.startMeasurement();

  // Set PID Mode to Automatic, may change later
  Pressure_PID.SetMode(AUTOMATIC);
  Pressure_PID.SetOutputLimits(15, 180);
  Pressure_PID.SetSampleTime(10);

  //TODO: Add ramp up to PIP value and stabilize
  // currBreathCycleState.CurrCycleStep = EXHALE;
  //TODO: set pressure setpoint to BREATHCYCLE__MINIMUM_PEEP__CENTIMETERSH2O
  //TODO: run PID control loop until things stabilize at min PEEP, then let the actual loop start

  // CurrCycleStep = INHALE_HOLD;
  CurrCycleStep = IDLE;
  // CurrCycleStep = INHALE_RAMP;
  CurrTimeInCycleMilliseconds = 0;
  ControlLoopStartTimeMilliseconds = CycleStartTimeFromSysClockMilliseconds = millis();
}

void loop()
{
  // Get a pressure reading in units of cmH2O
  // gagePressure.startMeasurement();
  if(!gagePressure.readData(!true))
  {
    gagePressure.startMeasurement();
    pressure_input = gagePressure.pressure * 2.53746;
    // Serial.print("Pressure: ");
    // Serial.println(pressure_input);
  }

  //TODO: Make sure clock overflow is handled gracefully
  if( CurrCycleStep != IDLE )
  {
    CurrTimeInCycleMilliseconds = millis()-CycleStartTimeFromSysClockMilliseconds;
    if(millis()-ControlLoopStartTimeMilliseconds > ControlLoopInitialStabilizationTImeMilliseconds)
    {
      if(CurrTimeInCycleMilliseconds <= InhaleRampDurationMilliseconds)
      {
        CurrCycleStep = INHALE_RAMP;
        // Serial.println("INHALE_RAMP");
      }
      else if((InhaleRampDurationMilliseconds < CurrTimeInCycleMilliseconds) &&
              (CurrTimeInCycleMilliseconds <= InhaleDurationMilliseconds))
      { 
        CurrCycleStep = INHALE_HOLD;
        // Serial.println("INHALE_HOLD");
      }
      else if((InhaleDurationMilliseconds < CurrTimeInCycleMilliseconds) &&
            (CurrTimeInCycleMilliseconds <= BreathCycleDurationMilliseconds))
      {
        CurrCycleStep = EXHALE;
        // Serial.println("EXHALE");
      }
      else if(CurrTimeInCycleMilliseconds > BreathCycleDurationMilliseconds)
      {
        CurrCycleStep = INHALE_RAMP;
        // Serial.println("INHALE_RAMP");
        CurrTimeInCycleMilliseconds = 0;
        CycleStartTimeFromSysClockMilliseconds = millis();
      }
    }
    else
    {
      CurrCycleStep = EXHALE;
    }
  }

  //Recompute Setpoints
  switch(CurrCycleStep)
  {
    case INHALE_RAMP:
      // calculate new setpoint based on linear ramp from PEEP pressure to PIP pressure over set duration
      // PRESSURE_SETPOINT(t) = t*(PIP/RAMP_DURATION)+PEEP
      Kp=64.000000, Ki=1.800000, Kd=13.000000;
      CurrPressureSetpointCentimetersH2O = (((float)CurrTimeInCycleMilliseconds/(float)InhaleRampDurationMilliseconds)*(PipPressureCentimetersH2O-PeepPressureCentimetersH2O))+PeepPressureCentimetersH2O;
      // Serial.println("INHALE_RAMP");
    break;
    case INHALE_HOLD:
      Kp=14.000000, Ki=12.000000, Kd=0.000000;
      CurrPressureSetpointCentimetersH2O = PipPressureCentimetersH2O;  // high
      // Serial.println("INHALE_HOLD");
    break;
    case EXHALE:
    case IDLE:
      // Kp=700.00000, Ki=20.50000, Kd=25.250000;
      Kp=1000, Ki=500.00000, Kd=8.0000;
      CurrPressureSetpointCentimetersH2O = PeepPressureCentimetersH2O;  // low
      // Serial.println("EXHALE");
    break;
  }

  // Set Kp, Ki, Kd and comute Pressure PID
  Pressure_PID.SetTunings(Kp, Ki, Kd);
  Pressure_PID.Compute();

  // Output calculated pulse width to motor
  blower.write(blower_output);

  // Open expiration valve
  char NewSolenoidState = digitalRead(solenoid_pin); 
  if(CurrCycleStep == EXHALE)
  {
    if((CurrPressureSetpointCentimetersH2O + expiration_offset + expiration_hysteresis) < (pressure_input))
    {
      NewSolenoidState = LOW;
    }
    
    else if((CurrPressureSetpointCentimetersH2O + expiration_offset - expiration_hysteresis)>= (pressure_input))
    {
      NewSolenoidState = HIGH;
    }

    if((digitalRead(solenoid_pin) != NewSolenoidState) && 
       ((TimeOfLastSolenoidToggleMilliseconds + SolenoidMinimumDwellTimeMilliseconds) < millis()))
    {
      digitalWrite(solenoid_pin, NewSolenoidState);
      TimeOfLastSolenoidToggleMilliseconds = millis();
    }

  }

  else // if inhale_ramp or inhale_hold
  {
    digitalWrite(solenoid_pin, HIGH);
  }

  Serial.print(pressure_input);
  Serial.print(" ");
  Serial.print(CurrPressureSetpointCentimetersH2O);
  // Serial.print(" ");
  // Serial.print(Pressure_PID.GetKp());
  // Serial.print(" ");
  // Serial.print(Pressure_PID.GetKi());
  // Serial.print(" ");
  // Serial.print(Pressure_PID.GetKd());
  // Serial.print(" ");
  // Serial.print(blower_output);
  Serial.println();

  // $<property_name> <value><LF>
  // PEEP	cmH20
  // PIP	cmH20
  // FI02	%
  // F	b/m
  // RISE	sec
  // I/E	ratio (denominator) <1,2,3>
  if (Serial.available())
  {
    string_from_pi = Serial.readStringUntil(0x0A);  // LF
    if(string_from_pi[0] == '$')
    {
      property_name = string_from_pi.substring(string_from_pi.indexOf('$') + 1, string_from_pi.indexOf(' '));
      value = string_from_pi.substring(string_from_pi.indexOf(' ') + 1, string_from_pi.indexOf(0x0A)).toFloat();
    }

    if(property_name.equalsIgnoreCase("PEEP"))
    {
      // PEEP Value
      PeepPressureCentimetersH2O = value;
    }

    else if(property_name.equalsIgnoreCase("PIP"))
    {
      // PIP Value
      PipPressureCentimetersH2O = value;
    }

    // else if(property_name == "FI02")
    // {
    //   // Flow of O2 in %
    // }

    // else if(property_name.equalsIgnoreCase("F"))
    // {
    //   // Breathes per minute in b/m
    // }

    else if(property_name.equalsIgnoreCase("Ramptime"))
    {
      // Rise time in seconds
      InhaleRampDurationMilliseconds = value;
    }

    else if(property_name.equalsIgnoreCase("InhaleTime"))
    {
      InhaleDurationMilliseconds = value*1000.00;
      InhaleRampDurationMilliseconds = InhaleDurationMilliseconds*0.25;
    }

    else if(property_name.equalsIgnoreCase("BreathRate"))
    {
      BreathCycleDurationMilliseconds = 60.0/value;
      if( (InhaleDurationMilliseconds/BreathCycleDurationMilliseconds) >= 0.50 )
      {
        InhaleDurationMilliseconds = BreathCycleDurationMilliseconds*0.50;
        InhaleRampDurationMilliseconds = InhaleDurationMilliseconds*0.25;
      }
    }

    else if( property_name.equalsIgnoreCase("Start") )
    {
      if(CurrCycleStep == IDLE)
      {
        CurrCycleStep = EXHALE;
        CurrCycleStep = EXHALE;
        CurrTimeInCycleMilliseconds = 0;
        ControlLoopStartTimeMilliseconds = CycleStartTimeFromSysClockMilliseconds = millis();
        Serial.println("Test Started");
        Serial.print("PEEP: ");              Serial.print(PeepPressureCentimetersH2O);                  Serial.println("cmH20");
        Serial.print("PIP: ");               Serial.print(PipPressureCentimetersH2O);                   Serial.println("cmH20");
        Serial.print("Rate: ");              Serial.print(BreathCycleDurationMilliseconds/(60*1000)) ;  Serial.println("/min");
        Serial.print("Inhale Duration: ");   Serial.print(InhaleDurationMilliseconds/1000) ;            Serial.println("s");
      }
    }

    else if( property_name.equalsIgnoreCase("Stop") )
    {
      Serial.println("Test Stopped");
      CurrCycleStep = IDLE;
    }
    
  }
}
