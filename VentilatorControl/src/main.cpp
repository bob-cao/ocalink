// ------------------------------VENTILATOR CONTROL---------------------------------- //

//TODO: LOW Refactor to have all variables extremely modular and not hard coded (later)

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

#define PINCH_VALVE_DRIVER_MIN_PULSE_MICROSECONDS (double)1450
#define PINCH_VALVE_DRIVER_MAX_PULSE_MICROSECONDS (double)1675

#define MIN_PERCENTAGE (double)0
#define MAX_PERCENTAGE (double)100

#define EXPIRATION_OFFSET (double)3.0f
#define EXPIRATION_HYSTERESIS (double)0.25f

#define BUZZER_PIN 2
#define PINCH_VALVE_PIN 3
// #define SOLENOID_PIN 4
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
#define DEFAULT_EXHALE_DURATION (uint32_t)1000
#define DEFAULT_CONTROL_LOOP_INIT_STABILIZATION (uint32_t)3000

#define BREATHS_PER_MINUTE_TO_MS 60000.000000

#define DEFAULT_PINCH_VALVE_MIN_DWELL_TIME (uint32_t)250

#define DEFAULT_PID_SAMPLE_TIME 10

#define PEEP_LOW_ALARM 1
#define PIP_ALARM 2
#define PEEP_ALARM 2

typedef enum{
    INHALE_RAMP,
    INHALE_HOLD,
    EXHALE_RAMP,
    EXHALE_HOLD,
    IDLE
}BreathCycleStep;
BreathCycleStep CurrCycleStep;

Servo blower;
Servo pinch_valve;

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

double valve_position, valve_state;

double peep_low_alarm, peep_alarm, pip_alarm;
bool buzzer_state = 1;
double RespritoryRate;
double InhalationExhalationRatio;
// --------------------------------USER SETTINGS------------------------------------- //



// --------------------------------STATE TIMINGS------------------------------------- //
uint32_t CurrTimeInCycleMilliseconds; // Time since the start of the current breath cycle. Resets at the beginning of every breath cycle
uint32_t CycleStartTimeFromSysClockMilliseconds;  // Time that the current breath cycle started ( in terms of system clock millis() )
uint32_t ControlLoopStartTimeMilliseconds; // Time, in terms of millis(), the state machine last switched out of IDLE
uint32_t ControlLoopInitialStabilizationTimeMilliseconds = DEFAULT_CONTROL_LOOP_INIT_STABILIZATION; // Length of time after transitioning out of IDLE that the system waits before transitioning to INHALE_RAMP
uint32_t InhaleRampDurationMilliseconds = DEFAULT_INHALE_RAMP; // Length of the INHALE_RAMP period for a breath cycle. AKA Value of CurrTimeInCycleMilliseconds when the state changes to INHALE_HOLD .User configurable
uint32_t InhaleDurationMilliseconds = DEFAULT_INHALE_DURATION; // Combined length of the INHALE_RAMP and INHALE_HOLD periods. AKA Value of CurrTimeInCycleMilliseconds when the state changes to EXHALE_HOLD. User configurable.
uint32_t ExhaleDurationMilliseconds = DEFAULT_EXHALE_DURATION; // Combined length of the EXHALE_RAMP and EXHALE_HOLD periods. AKA Value of CurrTimeInCycleMilliseconds when the state changes to INHALE_HOLD. User configurable.
uint32_t BreathCycleDurationMilliseconds = InhaleDurationMilliseconds + ExhaleDurationMilliseconds; // Total length of breath cycle, AKA when cycle step resets to INHALE_RAMP and CurrTimeInCycleMilliseconds resets to 0

uint32_t TimeOfLastSolenoidToggleMilliseconds = 0; // Time, in terms of millis(), that the solenoid last changed states
uint32_t SolenoidMinimumDwellTimeMilliseconds = DEFAULT_PINCH_VALVE_MIN_DWELL_TIME; // Minimum value of TimeOfLastSolenoidToggleMilliseconds before the solenoid may switch states again
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



void breath_cycle_timer_reset(bool hardreset = false)
{
  CurrTimeInCycleMilliseconds = 0;
  CycleStartTimeFromSysClockMilliseconds = millis();
  if(hardreset)
  {
    ControlLoopStartTimeMilliseconds = CycleStartTimeFromSysClockMilliseconds;
  }
}

void blower_esc_init (void)
{
  pinMode(BLOWER_PIN, OUTPUT);
  digitalWrite(BLOWER_PIN, LOW);
  blower.attach(BLOWER_PIN);
  // Hold throttle LOW for ESC to initialize properly
  blower.writeMicroseconds(BLOWER_DRIVER_MIN_PULSE_MICROSECONDS);
  delay(DEFAULT_ESC_INIT_TIME);
}

void pinch_valve_init (void)
{
  pinMode(PINCH_VALVE_PIN, OUTPUT);
  pinch_valve.attach(PINCH_VALVE_PIN);
  pinch_valve.writeMicroseconds(PINCH_VALVE_DRIVER_MIN_PULSE_MICROSECONDS);
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

void buzzer_init(void)
{
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);
}

double get_pressure_reading (void)
{
  if(!gagePressure.readData(false))
  {
    gagePressure.startMeasurement();
    // Get a pressure reading and convert to units of cmH2O
    pressure_reading = gagePressure.pressure * INCHES_2_CM;
  }
  return pressure_reading;
}

void get_values_from_raspberry_pi (void)
{
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

    if(property_name.equalsIgnoreCase("PIP"))
    {
      // PIP Value
      PipPressureCentimetersH2O = value;
    }

    else if(property_name.equalsIgnoreCase("PEEP"))
    {
      // PEEP Value
      PeepPressureCentimetersH2O = value;
    }

    // else if(property_name == "FI02")
    // {
    //   // Flow of O2 in %
    // }

    else if(property_name.equalsIgnoreCase("TRISE"))
    {
      // Rise time in seconds
      InhaleRampDurationMilliseconds = value * (1.000000 / BREATHS_PER_MINUTE_TO_MS);
    }

    else if(property_name.equalsIgnoreCase("RR"))
    {
      // Respritory Rate in (breathes per minute in b/m)
      RespritoryRate = value * (1.000000 / BREATHS_PER_MINUTE_TO_MS);
    }

    else if(property_name.equalsIgnoreCase("IE"))
    {
      // Inhalation/Exhalation Ratio
      InhalationExhalationRatio = value;
      InhaleDurationMilliseconds = (1 / RespritoryRate) * (1 / InhalationExhalationRatio) * (1.000000 / BREATHS_PER_MINUTE_TO_MS);
      ExhaleDurationMilliseconds = (1 / RespritoryRate) * (1.000000 - (1 / InhalationExhalationRatio)) * (1.000000 / BREATHS_PER_MINUTE_TO_MS);
    }

    else if( property_name.equalsIgnoreCase("START") )
    {
      if(CurrCycleStep == IDLE)
      {
        CurrCycleStep = EXHALE_HOLD;
        breath_cycle_timer_reset(true);
        Serial.println("TEST STARTED");
        Serial.print("PEEP: ");         Serial.print(PeepPressureCentimetersH2O);       Serial.println("cmH20");
        Serial.print("PIP: ");          Serial.print(PipPressureCentimetersH2O);        Serial.println("cmH20");
        // Serial.print("FIO2: ");         Serial.print(PipPressureCentimetersH2O);        Serial.println("cmH20");
        // Serial.print("TRISE: ");        Serial.print(PipPressureCentimetersH2O);        Serial.println("cmH20");
        // Serial.print("RR: ");           Serial.print(PipPressureCentimetersH2O);        Serial.println("cmH20");
        // Serial.print("IE: ");           Serial.print(PipPressureCentimetersH2O);        Serial.println("cmH20");

        // Serial.print("Breathcycle Duration: ");   Serial.print(BreathCycleDurationMilliseconds) ;  Serial.println("ms");
        // Serial.print("Inhale Duration: ");   Serial.print(InhaleDurationMilliseconds) ;            Serial.println("ms");
      }
    }

    else if( property_name.equalsIgnoreCase("STOP") )
    {
      Serial.println("Test Stopped");
      CurrCycleStep = IDLE;
    }

    // else if(property_name.equalsIgnoreCase("InhaleTime"))
    // {
    //   InhaleDurationMilliseconds = value;

    //   // arbitrarily default to having the ramp time be 25% of the inhale duration
    //   InhaleRampDurationMilliseconds = InhaleDurationMilliseconds*0.25;
    // }

    // else if(property_name.equalsIgnoreCase("BreathDuration"))
    // {
    //   BreathCycleDurationMilliseconds = value;

    //   // if the I:E ratio exceeds 1:1 (e.g ~2:1), cap it at 1:1. The standards we are workign from  state an expected I:E of 1:1-1:3
    //   if( (InhaleDurationMilliseconds/BreathCycleDurationMilliseconds) >= 0.50 )
    //   {
    //     InhaleDurationMilliseconds = BreathCycleDurationMilliseconds*0.50;
    //     InhaleRampDurationMilliseconds = InhaleDurationMilliseconds*0.25;
    //   }
    // }
  }
}

void print_pid_setpoint_and_current_value(void)
{
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
}

void write_calculated_pid_blower_speed(void)
{
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
}

void pinch_valve_control(void)
{
  // TODO: HIGH Rewrite solenoid state handling to be proportional, rather than binary
  static bool solenoidIsOpen = true; // initialize to open

  // TODO: HIGH break out solenoid handling into seperate file
  // Open expiration valve
  char NewSolenoidState = solenoidIsOpen;
  if(CurrCycleStep == EXHALE_HOLD)
  {
    if((CurrPressureSetpointCentimetersH2O + EXPIRATION_OFFSET + EXPIRATION_HYSTERESIS) < (pressure_system_input))
    {
      NewSolenoidState = true;
    }
    
    else if((CurrPressureSetpointCentimetersH2O + EXPIRATION_OFFSET - EXPIRATION_HYSTERESIS)>= (pressure_system_input))
    {
      NewSolenoidState = false;
    }

    if((solenoidIsOpen != NewSolenoidState) && 
       ((TimeOfLastSolenoidToggleMilliseconds + SolenoidMinimumDwellTimeMilliseconds) < millis()))
    {
      pinch_valve.writeMicroseconds(NewSolenoidState?PINCH_VALVE_DRIVER_MIN_PULSE_MICROSECONDS:PINCH_VALVE_DRIVER_MAX_PULSE_MICROSECONDS);
      TimeOfLastSolenoidToggleMilliseconds = millis();
      solenoidIsOpen = NewSolenoidState;
    }
  }

  else // if inhale_ramp or inhale_hold
  {
    pinch_valve.writeMicroseconds(PINCH_VALVE_DRIVER_MAX_PULSE_MICROSECONDS);
  }
}

void cycle_state_handler (void)
{
  // TODO: LOW Make sure clock overflow is handled gracefully
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
        CurrCycleStep = EXHALE_HOLD;
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

void cycle_state_setpoint_handler(void)
{
  // TODO: HIGH breakout into separate file and rewrite for readability
  // TODO: MEDIUM put gains in a header, not in the source

  //Recompute Setpoints
  switch(CurrCycleStep)
  {
    case INHALE_RAMP:
      // calculate new setpoint based on linear ramp from PEEP pressure to PIP pressure over set duration
      // PRESSURE_SETPOINT(t) = t*(PIP/RAMP_DURATION)+PEEP
      // Kp=64.000000, Ki=1.800000, Kd=13.000000;
      Kp=1.000000, Ki=0.028125, Kd=0.203125;
      CurrPressureSetpointCentimetersH2O = (((float)CurrTimeInCycleMilliseconds/(float)InhaleRampDurationMilliseconds)*(PipPressureCentimetersH2O-PeepPressureCentimetersH2O))+PeepPressureCentimetersH2O;
    break;
    case INHALE_HOLD:
      Kp=1.000000, Ki=0.857143, Kd=0.000000;
      CurrPressureSetpointCentimetersH2O = PipPressureCentimetersH2O;
    break;
    case EXHALE_HOLD:
    case IDLE:
    default:
      Kp=1.000000, Ki=0.500000, Kd=0.008000;
      CurrPressureSetpointCentimetersH2O = PeepPressureCentimetersH2O;
    break;
  }
}

void buzzer_toggle(void)
{
  static unsigned long lastBuzzerToggle = 0;
  if( millis()-lastBuzzerToggle > 500 )
  {
    digitalWrite(BUZZER_PIN, buzzer_state);
    buzzer_state = !buzzer_state;
    lastBuzzerToggle = millis();
  }
}

void alarms_settings(void)
{
  peep_low_alarm = PEEP_LOW_ALARM;
  peep_alarm = PEEP_ALARM;
  pip_alarm = PIP_ALARM;

  // // FOR TESTING
  // // CurrCycleStep = EXHALE_HOLD;
  // CurrCycleStep = INHALE_HOLD;

  // pressure_system_input = 5.0;
  // // FOR TESTING

  // 1 & 2a: High and Low PIP
  if((CurrCycleStep == EXHALE_HOLD && CurrCycleStep != EXHALE_RAMP && CurrCycleStep != INHALE_RAMP)
      && (pressure_system_input <= PipPressureCentimetersH2O - pip_alarm
      || pressure_system_input >= PipPressureCentimetersH2O + pip_alarm))
  {
    // make sound and send Raspberry Pi alarm status flag
    buzzer_toggle();
    Serial.write("PIP ERROR");
  }

  // 2b & 2c: High and Low PEEP
  if((CurrCycleStep == INHALE_HOLD && CurrCycleStep != EXHALE_RAMP && CurrCycleStep != INHALE_RAMP)
      && (pressure_system_input <= PeepPressureCentimetersH2O - peep_alarm
      || pressure_system_input >= PeepPressureCentimetersH2O + peep_alarm))
  {
    // make sound and send Raspberry Pi alarm status flag
    buzzer_toggle();
    Serial.write("PEEP ERROR");
  }

  // 2g: Disconnect Alarm
  if((CurrCycleStep == EXHALE_HOLD && CurrCycleStep != INHALE_RAMP)
      && (pressure_system_input >= -peep_low_alarm
      && pressure_system_input <= peep_low_alarm))
  {
    // make sound and send Raspberry Pi alarm status flag
    digitalWrite(BUZZER_PIN, HIGH);
    Serial.write("DISCONNECT ERROR");
  }
}

void alarms_faults(void)
{
  //
}

void setup()
{
  buzzer_init();

  /*// Initializations
  pinch_valve_init();
  blower_esc_init();
  pressure_sensors_init();
  pid_init();

  // Start cycle state in IDLE state
  CurrCycleStep = IDLE;
  valve_position = PINCH_VALVE_DRIVER_MIN_PULSE_MICROSECONDS;
  valve_state = true;

  // Serial initialization
  #if SYSTEM__SERIAL_DEBUG__STATEMACHINE
  Serial.begin(DEFAULT_BAUD_RATE);
  #endif

  breath_cycle_timer_reset(true);*/
}

void loop()
{
  alarms_settings();

  /*pressure_system_input = get_pressure_reading();

  cycle_state_handler();

  cycle_state_setpoint_handler();

  pinch_valve_control();

  write_calculated_pid_blower_speed();

  print_pid_setpoint_and_current_value();

  get_values_from_raspberry_pi();

    // TODO: HIGH Implement alarms, with serial protocol to inform the GUI/HMI*/
}
