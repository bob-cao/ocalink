#ifndef __VARIABLES_H__
#define __VARIABLES_H__

// ----------------------------------LIBRARIES--------------------------------------- //
#include "includes.h"
#include "Types.h"
// ----------------------------------LIBRARIES--------------------------------------- //



// ----------------------------------DEBUGGING--------------------------------------- //
#define SYSTEM__SERIAL_DEBUG__STATEMACHINE 1
// ----------------------------------DEBUGGING--------------------------------------- //



// ----------------------------------CONSTANTS--------------------------------------- //
#define DEFAULT_BAUD_RATE       256000

#define INCHES_2_CM     (double)2.54

#define MIN_PERCENTAGE  (double)0
#define MAX_PERCENTAGE  (double)100

#define BLOWER_DRIVER_MIN_PULSE_MICROSECONDS                (double)1000
#define BLOWER_DRIVER_MAX_PULSE_MICROSECONDS                (double)2000
#define BLOWER_DRIVER_PULSE_STARTUP_WIGGLE                  (double)100
#define DEFAULT_ESC_INIT_TIME                               (double)3000

#define PINCH_VALVE_DRIVER_FULL_OPEN_PULSE_MICROSECONDS     (double)1450
#define PINCH_VALVE_DRIVER_FULL_CLOSE_PULSE_MICROSECONDS    (double)1725
#define DEFAULT_PINCH_VALVE_MIN_DWELL_TIME                  (double)250

#define NUM_LEDS            24                          // 2 Inch LED Ring
#define LED_ON_TIME         50                          // time in ms

#define DEFAULT_PID_SAMPLE_TIME     (double)1.5         // time in ms

#define DEFAULT_BLOWER_KP           (double)10.010
#define DEFAULT_BLOWER_KI           (double)0.000
#define DEFAULT_BLOWER_KD           (double)0.008000

#define DEFAULT_PINCH_VALVE_KP      (double)5.000
#define DEFAULT_PINCH_VALVE_KI      (double)0.000
#define DEFAULT_PINCH_VALVE_KD      (double)1.000

#define DEFAULT_PEEP                (double)5.000       // in cmH2O
#define DEFAULT_PIP                 (double)20.000      // in cmH2O
#define DEFAULT_RISE                (double)1000        // 1 second in ms
#define DEFAULT_INHALE_RAMP         (double)250         // 0.25 seconds in ms
#define DEFAULT_IE                          1           // inhale/exhale ratio normalized to 1 (1:2)
#define DEFAULT_RR                          10          // in breaths per minute

#define BREATHS_PER_MINUTE_TO_SEC   (double)60.000
#define SEC_TO_MS                   (double)1000.000
#define RATIO_TO_PERCENTAGE         (double)100.00

#define PEEP_LOW_ALARM      1
#define PEEP_ALARM          2
#define PIP_ALARM           2

#define DEFAULT_INHALE_DURATION                     (double)1500
#define DEFAULT_BREATH_CYCLE_DURATION               (double)6000
#define DEFAULT_EXHALE_DURATION                     (DEFAULT_BREATH_CYCLE_DURATION - DEFAULT_INHALE_DURATION)
#define DEFAULT_CONTROL_LOOP_INIT_STABILIZATION     (double)3000

#define DEFAULT_PEEP_TIME           (double)500
#define DEFAULT_PIP_TIME            (double)100
#define DEFAULT_DISCONNECT_TIME     (double)500
// #define DEFAULT_APNEA_TIME          (double)3000

#define PEEP_MIN_RECEIVE    5
#define PEEP_MAX_RECEIVE    25
#define PIP_MIN_RECEIVE     15
#define PIP_MAX_RECEIVE     55
#define FIO2_MIN_RECEIVE    0.2
#define FIO2_MAX_RECEIVE    1
#define TRISE_MIN_RECEIVE   0.5
#define TRISE_MAX_RECEIVE   4
#define RR_MIN_RECEIVE      5
#define RR_MAX_RECEIVE      50
#define IE_MIN_RECEIVE      1
#define IE_MAX_RECEIVE      4

#define DEFAULT_IE_SCALING_FACTOR       10
#define DEFAULT_TRISE_SCALING_FACTOR    10
#define DEFAULT_FIO2_SCALING_FACTOR     100

// ----------------------------------CONSTANTS--------------------------------------- //



// -----------------------------------OBJECTS---------------------------------------- //
extern BreathCycleStep CurrCycleStep;
extern AllSensors_DLHR_L60D_8 gagePressure;

extern Servo blower;
extern Servo pinch_valve;
// -----------------------------------OBJECTS---------------------------------------- //



// --------------------------------USER SETTINGS------------------------------------- //
extern double pressure_reading;
extern double blower_speed;

extern String string_from_pi;
extern String property_name;
extern String argument;
extern double argument_value;

extern double peep_low_alarm;
extern double peep_alarm;
extern double pip_alarm;

extern double PeepPressureCentimetersH2O;
extern double PipPressureCentimetersH2O;
extern double RespritoryRate;
extern double InhalationExhalationRatio;
// extern double FlowOfOxygen;
extern double IEScalingFactor;

extern double PeepAlarmTimer;
extern double PipAlarmTimer;
extern double DisconnectAlarmTimer;
// extern double ApneaTimer;

extern bool isBatteryActivated;
extern bool buzzer_state;

extern Adafruit_NeoPixel AlarmLED;

extern byte low;
extern byte red;
extern byte low_red;
extern byte amber;
extern byte low_amber;
extern byte green;
extern byte low_green;
extern byte perywinkle;     // perywinkle (pastell blue)
extern byte low_perywinkle; // perywinkle (pastell blue)

extern byte alarm_state;
// --------------------------------USER SETTINGS------------------------------------- //



// --------------------------------STATE TIMINGS------------------------------------- //
extern double CurrTimeInCycleMilliseconds;                        // Time since the start of the current breath cycle. Resets at the beginning of every breath cycle
extern double CycleStartTimeFromSysClockMilliseconds;             // Time that the current breath cycle started ( in terms of system clock millis() )
extern double ControlLoopStartTimeMilliseconds;                   // Time, in terms of millis(), the state machine last switched out of IDLE
extern double ControlLoopInitialStabilizationTimeMilliseconds;    // Length of time after transitioning out of IDLE that the system waits before transitioning to INHALE_RAMP
extern double InhaleRampDurationMilliseconds;                     // Length of the INHALE_RAMP period for a breath cycle. AKA Value of CurrTimeInCycleMilliseconds when the state changes to INHALE_HOLD .User configurable
extern double InhaleDurationMilliseconds;                         // Combined length of the INHALE_RAMP and INHALE_HOLD periods. AKA Value of CurrTimeInCycleMilliseconds when the state changes to EXHALE_HOLD. User configurable.
extern double ExhaleDurationMilliseconds;                         // Combined length of the EXHALE_RAMP and EXHALE_HOLD periods. AKA Value of CurrTimeInCycleMilliseconds when the state changes to INHALE_HOLD. User configurable.
extern double BreathCycleDurationMilliseconds;                    // Total length of breath cycle, AKA when cycle step resets to INHALE_RAMP and CurrTimeInCycleMilliseconds resets to 0

extern double TimeOfLastSolenoidToggleMilliseconds;               // Time, in terms of millis(), that the solenoid last changed states
// --------------------------------STATE TIMINGS------------------------------------- //



// --------------------------------PID SETTINGS-------------------------------------- //
extern double pressure_system_input, blower_output_speed_in_percentage, pinch_valve_output_openness_in_percentage, CurrPressureSetpointCentimetersH2O;
extern double Blower_Kp, Blower_Ki, Blower_Kd;
extern double PinchValve_Kp, PinchValve_Ki, PinchValve_Kd;

extern PID Blower_PID;
extern PID PinchValve_PID;
// --------------------------------PID SETTINGS-------------------------------------- //

#endif // __VARIABLES_H__
