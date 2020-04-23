#ifndef __VARIABLES_H__
#define __VARIABLES_H__

// ----------------------------------LIBRARIES--------------------------------------- //
#include "includes.h"
// ----------------------------------LIBRARIES--------------------------------------- //



// ----------------------------------DEBUGGING--------------------------------------- //
#define SYSTEM__SERIAL_DEBUG__STATEMACHINE 1
// ----------------------------------DEBUGGING--------------------------------------- //



// -----------------------------------OBJECTS---------------------------------------- //
extern BreathCycleStep CurrCycleStep;
// extern AllSensors_DLHR_L60D_8 gagePressure;

extern Servo blower;
extern Servo pinch_valve;

TCA9548A I2CMux;
AllSensors_DLHR_L60D_8 patientCircuitPressure(&Wire);
AllSensors_DLHR_L60D_8 venturiDifferentialPressure(&Wire);
// -----------------------------------OBJECTS---------------------------------------- //



// --------------------------------USER SETTINGS------------------------------------- //
extern double pressure_reading;
extern double venturiDifferentialPressureReading;
extern double venturiFlowRateLpm;
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

extern double PressureSensorLastStatusRead;

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
