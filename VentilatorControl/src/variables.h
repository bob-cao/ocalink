#ifndef __VARIABLES_H__
#define __VARIABLES_H__

// ----------------------------------LIBRARIES--------------------------------------- //
#include "includes.h"
// ----------------------------------LIBRARIES--------------------------------------- //



// ----------------------------------DEBUGGING--------------------------------------- //
#define SYSTEM__SERIAL_DEBUG__STATEMACHINE 1
// ----------------------------------DEBUGGING--------------------------------------- //



// ----------------------------------CONSTANTS--------------------------------------- //
#define INCHES_2_CM (double)2.54

#define DEFAULT_BAUD_RATE 256000

#define BLOWER_DRIVER_MIN_PULSE_MICROSECONDS (double)1000
#define BLOWER_DRIVER_MAX_PULSE_MICROSECONDS (double)2000
#define BLOWER_DRIVER_PULSE_STARTUP_WIGGLE   (double)100
#define DEFAULT_ESC_INIT_TIME 3000

#define PINCH_VALVE_DRIVER_FULL_OPEN_PULSE_MICROSECONDS (double)1450
#define PINCH_VALVE_DRIVER_FULL_CLOSE_PULSE_MICROSECONDS (double)1725

#define MIN_PERCENTAGE (double)0
#define MAX_PERCENTAGE (double)100

#define BUZZER_PIN 2
#define PINCH_VALVE_PIN 4
#define BLOWER_FEEDBACK_PIN A1
#define BLOWER_SPEED_PIN 3
#define BATTERY_ALARM_PIN 6
#define BATTERY_ENABLE_PIN 7
#define LED_DATA_PIN A2

#define NUM_LEDS 24 //2 Inch LED Ring

#define DEFAULT_PEEP (double)5.000000
#define DEFAULT_PIP (double)20.000000
#define DEFAULT_BM 10  // breaths per minute
#define DEFAULT_RISE 1000  // 1 second
#define DEFAULT_IE 2 // inhale/exhale ratio
#define DEFAULT_KP (double)1.000000
#define DEFAULT_KI (double)1.000000
#define DEFAULT_KD (double)0.000000

#define DEFAULT_INHALE_RAMP (uint32_t)250

#define BREATHS_PER_MINUTE_TO_SEC (double)60.000000
#define SEC_TO_MS (double)1000.000000

#define DEFAULT_PINCH_VALVE_MIN_DWELL_TIME (uint32_t)250

#define IE_DEFAULT_SCALING_FACTOR (double)10.000000

#define PEEP_LOW_ALARM 1
#define PIP_ALARM 2
#define PEEP_ALARM 2

#define DEFAULT_INHALE_DURATION (uint32_t)1500
#define DEFAULT_BREATH_CYCLE_DURATION (uint32_t)6000
#define DEFAULT_EXHALE_DURATION (DEFAULT_BREATH_CYCLE_DURATION - DEFAULT_INHALE_DURATION)
#define DEFAULT_CONTROL_LOOP_INIT_STABILIZATION (uint32_t)3000

#define DEFAULT_PID_SAMPLE_TIME 1.5
#define DEFAULT_APNEA_TIME 3000

#define DEFAULT_DISCONNECT_TIME 500
#define DEFAULT_PIP_TIME 100
#define DEFAULT_PEEP_TIME 500

#define DEFAULT_RR 10
#define DEFAULT_IE_RATIO 1

#define LED_ON_TIME 50

#define PIP_MIN_RECEIVE 15
#define PIP_MAX_RECEIVE 55
#define PEEP_MIN_RECEIVE 5
#define PEEP_MAX_RECEIVE 25
#define FIO2_MIN_RECEIVE 0.2
#define FIO2_MAX_RECEIVE 1
#define TRISE_MIN_RECEIVE 0.5
#define TRISE_MAX_RECEIVE 4
#define RR_MIN_RECEIVE 5
#define RR_MAX_RECEIVE 50
#define IE_MIN_RECEIVE 1
#define IE_MAX_RECEIVE 4

#define TRISE_MULTIPLIER (double)10
#define FIO2_MULTIPLIER (double)100
#define IE_MULTIPLIER (double)10

typedef enum{
    INHALE_RAMP,
    INHALE_HOLD,
    EXHALE_RAMP,
    EXHALE_HOLD,
    IDLE
}BreathCycleStep;


extern BreathCycleStep CurrCycleStep;

extern Servo blower;
extern Servo pinch_valve;

extern AllSensors_DLHR_L60D_8 gagePressure;
// ----------------------------------CONSTANTS--------------------------------------- //



// --------------------------------USER SETTINGS------------------------------------- //
extern double pressure_reading;
extern double blower_speed;

extern double PeepPressureCentimetersH2O;
extern double PipPressureCentimetersH2O;

extern String string_from_pi;
extern String property_name;
extern String argument;
extern double argument_value;

extern double valve_position, valve_state;

extern double peep_low_alarm;
extern double peep_alarm;
extern double pip_alarm;
extern double ApneaTimer;
extern bool buzzer_state;
extern double RespritoryRate;
extern double InhalationExhalationRatio;
extern double FlowOfOxygen;
extern double IEScalingFactor;
extern bool isBatteryActivated;
extern double DisconnectAlarmTimer;
extern double PipAlarmTimer;
extern double PeepAlarmTimer;

extern Adafruit_NeoPixel AlarmLED;

extern uint32_t low;
extern uint32_t red;
extern uint32_t low_red;
extern uint32_t amber;
extern uint32_t low_amber;
extern uint32_t green;
extern uint32_t low_green;
extern uint32_t perywinkle;  // perywinkle (pastell blue)
extern uint32_t low_perywinkle;  // perywinkle (pastell blue)
extern byte led_colour;
// --------------------------------USER SETTINGS------------------------------------- //



// --------------------------------STATE TIMINGS------------------------------------- //
extern uint32_t CurrTimeInCycleMilliseconds; // Time since the start of the current breath cycle. Resets at the beginning of every breath cycle
extern uint32_t CycleStartTimeFromSysClockMilliseconds;  // Time that the current breath cycle started ( in terms of system clock millis() )
extern uint32_t ControlLoopStartTimeMilliseconds; // Time, in terms of millis(), the state machine last switched out of IDLE
extern uint32_t ControlLoopInitialStabilizationTimeMilliseconds; // Length of time after transitioning out of IDLE that the system waits before transitioning to INHALE_RAMP
extern uint32_t InhaleRampDurationMilliseconds; // Length of the INHALE_RAMP period for a breath cycle. AKA Value of CurrTimeInCycleMilliseconds when the state changes to INHALE_HOLD .User configurable
extern uint32_t InhaleDurationMilliseconds; // Combined length of the INHALE_RAMP and INHALE_HOLD periods. AKA Value of CurrTimeInCycleMilliseconds when the state changes to EXHALE_HOLD. User configurable.
extern uint32_t ExhaleDurationMilliseconds; // Combined length of the EXHALE_RAMP and EXHALE_HOLD periods. AKA Value of CurrTimeInCycleMilliseconds when the state changes to INHALE_HOLD. User configurable.
extern uint32_t BreathCycleDurationMilliseconds; // Total length of breath cycle, AKA when cycle step resets to INHALE_RAMP and CurrTimeInCycleMilliseconds resets to 0

extern uint32_t TimeOfLastSolenoidToggleMilliseconds; // Time, in terms of millis(), that the solenoid last changed states
// --------------------------------STATE TIMINGS------------------------------------- //



// --------------------------------PID SETTINGS-------------------------------------- //
// TODO: MEDIUM reorganize constants
// Pressure Controlled Blower PID
extern double pressure_system_input, blower_output_speed_in_percentage, pinch_valve_output_openness_in_percentage, CurrPressureSetpointCentimetersH2O;
//double Blower_Kp;
extern double Blower_Kp, Blower_Ki, Blower_Kd;
extern double PinchValve_Kp, PinchValve_Ki, PinchValve_Kd;
extern PID Blower_PID;

extern PID PinchValve_PID;
// --------------------------------PID SETTINGS-------------------------------------- //

#endif // __VARIABLES_H__
