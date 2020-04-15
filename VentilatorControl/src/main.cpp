// ------------------------------VENTILATOR CONTROL---------------------------------- //



// ----------------------------------LIBRARIES--------------------------------------- //
#include "includes.h"
// ----------------------------------LIBRARIES--------------------------------------- //



// ----------------------------------DEBUGGING--------------------------------------- //
#define SYSTEM__SERIAL_DEBUG__STATEMACHINE 1
// ----------------------------------DEBUGGING--------------------------------------- //



// ----------------------------------CONSTANTS--------------------------------------- //
#define INCHES_2_CM 2.54f

#define DEFAULT_BAUD_RATE 115200

#define BLOWER_DRIVER_MIN_PULSE_MICROSECONDS (double)1000
#define BLOWER_DRIVER_MAX_PULSE_MICROSECONDS (double)2000
#define DEFAULT_ESC_INIT_TIME 3000

#define MIN_PERCENTAGE (double)0
#define MAX_PERCENTAGE (double)100

#define EXPIRATION_OFFSET (double)3.0f
#define EXPIRATION_HYSTERESIS (double)0.25f

#define SOLENOID_PIN 4
#define BLOWER_PIN 5

#define DEFAULT_PEEP 5.000000f
#define DEFAULT_PIP 20.000000f
#define DEFAULT_BM 10  // breaths per minute
#define DEFAULT_RISE 1000  // 1 second
#define DEFAULT_IE 2 // inhale/exhale ratio
#define DEFAULT_KP 1.000000
#define DEFAULT_KI 1.000000
#define DEFAULT_KD 0.000000

#define DEFAULT_INHALE_RAMP (uint32_t)250
#define DEFAULT_INHALE_DURATION (uint32_t)1000
#define DEFAULT_BREATH_CYCLE_DURATION (uint32_t)6000
#define DEFAULT_CONTROL_LOOP_INIT_STABILIZATION (uint32_t)3000

#define DEFAULT_PINCH_VALVE_MIN_DWELL_TIME (uint32_t)250

#define DEFAULT_PID_SAMPLE_TIME 10

typedef enum{
    INHALE_RAMP,
    INHALE_HOLD,
    EXHALE,
    IDLE
}BreathCycleStep;
BreathCycleStep CurrCycleStep;

Servo blower;

AllSensors_DLHR_L60D_8 gagePressure(&Wire);
// ----------------------------------CONSTANTS--------------------------------------- //



// --------------------------------USER SETTINGS------------------------------------- //
double pressure_reading;
double blower_speed;

double PeepPressureCentimetersH2O = DEFAULT_PEEP;
double PipPressureCentimetersH2O = DEFAULT_PIP;

String string_from_pi;
String property_name;
double value;
// --------------------------------USER SETTINGS------------------------------------- //



// --------------------------------STATE TIMINGS------------------------------------- //

// --------------------------------STATE TIMINGS------------------------------------- //



// --------------------------------PID SETTINGS-------------------------------------- //
// TODO: MEDIUM reorganize constants
// Pressure Controlled Blower PID
double pressure_system_input, blower_output_speed_in_percentage, CurrPressureSetpointCentimetersH2O;
double Kp = DEFAULT_KP, Ki = DEFAULT_KI, Kd = DEFAULT_KD;
PID Pressure_PID(&pressure_system_input,
                &blower_output_speed_in_percentage,
                &CurrPressureSetpointCentimetersH2O,
                Kp, Ki, Kd, DIRECT);
// --------------------------------PID SETTINGS-------------------------------------- //



//TODO: LOW Refactor to have all variables extremely modular and not hard coded (later)

uint32_t CurrTimeInCycleMilliseconds; // Time since the start of the current breath cycle. Resets at the beginning of every breath cycle
uint32_t CycleStartTimeFromSysClockMilliseconds;  // Time that the current breath cycle started ( in terms of system clock millis() )
uint32_t ControlLoopStartTimeMilliseconds; // Time, in terms of millis(), the state machine last switched out of IDLE
uint32_t ControlLoopInitialStabilizationTimeMilliseconds = DEFAULT_CONTROL_LOOP_INIT_STABILIZATION; // Length of time after transitioning out of IDLE that the system waits before transitioning to INHALE_RAMP
uint32_t InhaleRampDurationMilliseconds = DEFAULT_INHALE_RAMP; // Length of the INHALE_RAMP period for a breath cycle. AKA Value of CurrTimeInCycleMilliseconds when the state changes to INHALE_HOLD .User configurable
uint32_t InhaleDurationMilliseconds = DEFAULT_INHALE_DURATION; // Combined length of the INHALE_RAMP and INHALE_HOLD periods. AKA Value of CurrTimeInCycleMilliseconds when the state changes to EXHALE. User configurable.
uint32_t BreathCycleDurationMilliseconds = DEFAULT_BREATH_CYCLE_DURATION; // Total length of breath cycle, AKA when cycle step resets to INHALE_RAMP and CurrTimeInCycleMilliseconds resets to 0

uint32_t TimeOfLastSolenoidToggleMilliseconds = 0; // Time, in terms of millis(), that the solenoid last changed states
uint32_t SolenoidMinimumDwellTimeMilliseconds = DEFAULT_PINCH_VALVE_MIN_DWELL_TIME; // Minimum value of TimeOfLastSolenoidToggleMilliseconds before the solenoid may switch states again

void breath_cycle_timer_reset(void)
{
  CurrTimeInCycleMilliseconds = 0;
  ControlLoopStartTimeMilliseconds = CycleStartTimeFromSysClockMilliseconds = millis();
}

void blower_esc_init (void)
{
    // Need to hold throttle LOW for ESC to start properly
  blower.attach(BLOWER_PIN);
  blower.writeMicroseconds(BLOWER_DRIVER_MIN_PULSE_MICROSECONDS);
  delay(DEFAULT_ESC_INIT_TIME);
}

void pinch_valve_init (void)
{
  pinMode(SOLENOID_PIN, OUTPUT);
  digitalWrite(SOLENOID_PIN, LOW);
}

void pressure_sensors_init (void)
{
  Wire.begin();  // Each pressure sensor will be unique I2C addresses based on MPN
  gagePressure.setPressureUnit(AllSensors_DLHR::PressureUnit::IN_H2O);
  gagePressure.startMeasurement();
}

void pid_init (void)
{
  Pressure_PID.SetMode(AUTOMATIC);  // Set PID Mode to Automatic, may change later
  Pressure_PID.SetOutputLimits(MIN_PERCENTAGE, MAX_PERCENTAGE);
  Pressure_PID.SetSampleTime(DEFAULT_PID_SAMPLE_TIME);
}

double get_pressure_reading (void)
{
  // Get a pressure reading in units of cmH2O
  if(!gagePressure.readData(!true))
  {
    gagePressure.startMeasurement();
    pressure_reading = gagePressure.pressure * INCHES_2_CM;
    // Serial.print("Pressure: ");
    // Serial.println(pressure_system_input);
  }

  return pressure_reading;
}

void setup()
{
  // Inits
  pinch_valve_init();
  blower_esc_init();
  pressure_sensors_init();
  pid_init();

  // Start cycle state to idle
  CurrCycleStep = IDLE;

    // Serial Init
  #if SYSTEM__SERIAL_DEBUG__STATEMACHINE
  Serial.begin(DEFAULT_BAUD_RATE);
  #endif

  breath_cycle_timer_reset();
}

void loop()
{
  pressure_system_input = get_pressure_reading();

  // TODO: break out into separate file and rewrite for better readability
  //TODO: LOW Make sure clock overflow is handled gracefully
  if( CurrCycleStep != IDLE )
  {
    CurrTimeInCycleMilliseconds = millis()-CycleStartTimeFromSysClockMilliseconds;
    if(millis()-ControlLoopStartTimeMilliseconds > ControlLoopInitialStabilizationTimeMilliseconds)
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
        breath_cycle_timer_reset();
      }
    }
    else // if idle == true
    {
      CurrCycleStep = EXHALE;
    }
  }

  // TODO: HIGH breakout into separate file and rewrite for readability.
  // TODO: MEDIUM put gains in a header, not in the source
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
      // TODO: MEDIUM tune/fix. These gains are wack, yo.
      // Kp=700.00000, Ki=20.50000, Kd=25.250000;
      Kp=1000, Ki=500.00000, Kd=8.0000;
      CurrPressureSetpointCentimetersH2O = PeepPressureCentimetersH2O;  // low
      // Serial.println("EXHALE");
    break;
  }

  // Set Kp, Ki, Kd and comute Pressure PID
  Pressure_PID.SetTunings(Kp, Ki, Kd);
  Pressure_PID.Compute();

  // Output PID calculated 0-100% to motor
  blower_speed = map(blower_output_speed_in_percentage,
                    MIN_PERCENTAGE,
                    MAX_PERCENTAGE,
                    BLOWER_DRIVER_MIN_PULSE_MICROSECONDS,
                    BLOWER_DRIVER_MAX_PULSE_MICROSECONDS);
  blower.writeMicroseconds(blower_speed);

  // TODO: HIGH Rewrite solenoid state handling to be proportional, rather than binary
  // TODO: HIGH break out solenoid handling into seperate file
  // Open expiration valve
  char NewSolenoidState = digitalRead(SOLENOID_PIN); 
  if(CurrCycleStep == EXHALE)
  {
    if((CurrPressureSetpointCentimetersH2O + EXPIRATION_OFFSET + EXPIRATION_HYSTERESIS) < (pressure_system_input))
    {
      NewSolenoidState = LOW;
    }
    
    else if((CurrPressureSetpointCentimetersH2O + EXPIRATION_OFFSET - EXPIRATION_HYSTERESIS)>= (pressure_system_input))
    {
      NewSolenoidState = HIGH;
    }

    if((digitalRead(SOLENOID_PIN) != NewSolenoidState) && 
       ((TimeOfLastSolenoidToggleMilliseconds + SolenoidMinimumDwellTimeMilliseconds) < millis()))
    {
      digitalWrite(SOLENOID_PIN, NewSolenoidState);
      TimeOfLastSolenoidToggleMilliseconds = millis();
    }
  }

  else // if inhale_ramp or inhale_hold
  {
    digitalWrite(SOLENOID_PIN, HIGH);
  }

  if( CurrCycleStep != IDLE )
  {
    if( millis() % 25 == 0 )
    {
      Serial.print(pressure_system_input);
      Serial.print(" ");
      Serial.print(CurrPressureSetpointCentimetersH2O);
      Serial.println();
    }
  }

  // TODO: HIGH Break out serial protocol into seperate file
  // TODO: HIGH ensure serial operations do not dirupt control loop
  // $<property_name> <value><LF>
  // PEEP	cmH20
  // PIP	cmH20
  // FI02	%
  // F	b/m
  // RISE	sec
  // I/E	ratio (denominator) <1,2,3>
  if (Serial.available())
  {
    string_from_pi = Serial.readStringUntil('*'); 
    if(string_from_pi[0] == '$')
    {
      property_name = string_from_pi.substring(string_from_pi.indexOf('$') + 1, string_from_pi.indexOf(' '));
      value = string_from_pi.substring(string_from_pi.indexOf(' ') + 1, string_from_pi.indexOf('*')).toFloat();
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
      InhaleDurationMilliseconds = value;

      // arbitrarily default to having the ramp time be 25% of the inhale duration
      InhaleRampDurationMilliseconds = InhaleDurationMilliseconds*0.25;
    }

    else if(property_name.equalsIgnoreCase("BreathDuration"))
    {
      BreathCycleDurationMilliseconds = value;

      // if the I:E ratio exceeds 1:1 (e.g ~2:1), cap it at 1:1. The standards we are workign from  state an expected I:E of 1:1-1:3
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
        breath_cycle_timer_reset();
        Serial.println("Test Started");
        Serial.print("PEEP: ");              Serial.print(PeepPressureCentimetersH2O);                  Serial.println("cmH20");
        Serial.print("PIP: ");               Serial.print(PipPressureCentimetersH2O);                   Serial.println("cmH20");
        Serial.print("Breathcycle Duration: ");   Serial.print(BreathCycleDurationMilliseconds) ;  Serial.println("ms");
        Serial.print("Inhale Duration: ");   Serial.print(InhaleDurationMilliseconds) ;            Serial.println("ms");
      }
    }

    else if( property_name.equalsIgnoreCase("Stop") )
    {
      Serial.println("Test Stopped");
      CurrCycleStep = IDLE;
    }
    
  }

    // TODO: HIGH Implement alarms, with serial protocol to inform the GUI/HMI
}
