#include "includes.h"

// ----------------------------------CONSTANTS--------------------------------------- //
BreathCycleStep CurrCycleStep;

Servo blower;
Servo pinch_valve;

AllSensors_DLHR_L60D_8 gagePressure(&Wire);
// ----------------------------------CONSTANTS--------------------------------------- //



// --------------------------------USER SETTINGS------------------------------------- //
double pressure_reading;
double blower_speed;

double PeepPressureCentimetersH2O = DEFAULT_PEEP;
double PipPressureCentimetersH2O =  DEFAULT_PIP;

String string_from_pi;
String property_name;
String argument;
double argument_value;

double peep_low_alarm =             PEEP_LOW_ALARM;
double peep_alarm =                 PEEP_ALARM;
double pip_alarm =                  PIP_ALARM;

// double ApneaTimer =                 DEFAULT_APNEA_TIME;
double RespritoryRate =             DEFAULT_RR;
double InhalationExhalationRatio =  DEFAULT_IE_RATIO;
// double FlowOfOxygen;
double IEScalingFactor =            DEFAULT_IE_SCALING_FACTOR;
double DisconnectAlarmTimer =       DEFAULT_DISCONNECT_TIME;
double PipAlarmTimer =              DEFAULT_PIP_TIME;
double PeepAlarmTimer =             DEFAULT_PEEP_TIME;

bool isBatteryActivated = false;
bool buzzer_state = 1;

Adafruit_NeoPixel AlarmLED = Adafruit_NeoPixel(NUM_LEDS, LED_DATA_PIN, NEO_GRB + NEO_KHZ800);

byte low            =   AlarmLED.Color(0, 0, 0);
byte red            =   AlarmLED.Color(255, 0, 0);
byte low_red        =   AlarmLED.Color(100,0,0);
byte amber          =   AlarmLED.Color(255, 70, 0);
byte low_amber      =   AlarmLED.Color(100,30,0);
byte green          =   AlarmLED.Color(0, 40, 0);
byte low_green      =   AlarmLED.Color(0,75,0);
byte perywinkle     =   AlarmLED.Color(5 , 7, 10);  // perywinkle (pastell blue)
byte low_perywinkle =   AlarmLED.Color(2,3,5);      // perywinkle (pastell blue)

byte alarm_state = 1;
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
// --------------------------------STATE TIMINGS------------------------------------- //



// --------------------------------PID SETTINGS-------------------------------------- //
double pressure_system_input, blower_output_speed_in_percentage, pinch_valve_output_openness_in_percentage, CurrPressureSetpointCentimetersH2O;
double Blower_Kp = DEFAULT_BLOWER_KP, Blower_Ki = DEFAULT_BLOWER_KI, Blower_Kd = DEFAULT_BLOWER_KD;
double PinchValve_Kp = DEFAULT_PINCH_VALVE_KP, PinchValve_Ki = DEFAULT_PINCH_VALVE_KI, PinchValve_Kd = DEFAULT_PINCH_VALVE_KD;

PID Blower_PID(&pressure_system_input,
                &blower_output_speed_in_percentage,
                &CurrPressureSetpointCentimetersH2O,
                Blower_Kp, Blower_Ki, Blower_Kd, DIRECT);

PID PinchValve_PID(&pressure_system_input,
                &pinch_valve_output_openness_in_percentage,
                &CurrPressureSetpointCentimetersH2O,
                PinchValve_Kp, PinchValve_Kd, PinchValve_Kd, REVERSE);
// --------------------------------PID SETTINGS-------------------------------------- //
