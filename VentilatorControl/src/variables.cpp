#include "includes.h"



// TODO: LOW Priority) Update variable names to be consistent with C++ standard

// OLD NAME                                        -> NEW NAME

// patientCircuitPressure				            -> PatientCircuitPressure
// venturiDifferentialPressure			            -> VenturiDifferentialPressure

// pressure_reading			        	        -> pressureReading
// blower_speed				        	        -> blowerSpeed
// string_from_pi				       	            -> stringFromPi
// property_name				    	            -> propertyName
// argument_value			    		            -> argumentValue
// peep_low_alarm			    		            -> peepLowAlarm
// peep_alarm				        	            -> peepAlarm
// pip_alarm				        	            -> pipAlarm
// PeepPressureCentimetersH2O			            -> peepPressureCentimetersH2O
// PipPressureCentimetersH2O			            -> pipPressureCentimetersH2O
// InhalationExhalationRatio			            -> inhalationExhalationRatio
// FlowOfOxygen					                -> flowOfOxygen
// PeepAlarmTimer				    	            -> peepAlarmTimer
// PipAlarmTimer				    	            -> pipAlarmTimer
// DisconnectAlarmTimer			    	        -> disconnectAlarmTimer
// ApneaTimer				        	            -> apneaTimer
// buzzer_state					                -> buzzerState
// low_red					        	            -> lowRed
// low_amber				        	            -> lowAmber
// low_green					                    -> lowGreen
// low_perywinkle				    	            -> lowPeriwinkle
// alarm_state					                    -> alarmState
// CurrTimeInCycleMilliseconds			            -> currTimeInCycleMilliseconds
// CycleStartTimeFromSysClockMilliseconds		    -> cycleStartTimeFromSysClockMilliseconds
// ControlLoopStartTimeMilliseconds		        -> controlLoopStartTimeMilliseconds
// ControlLoopInitialStabilizationTimeMilliseconds -> controlLoopInitialStabilizationTimeMilliseconds
// InhaleRampDurationMilliseconds			        -> inhaleRampDurationMilliseconds
// inhaleDurationMilliseconds                      -> inhaleDurationMilliseconds
// ExhaleDurationMilliseconds			            -> exhaleDurationMilliseconds
// BreathCycleDurationMilliseconds			        -> breathCycleDurationMilliseconds
// PressureSensorLastStatusRead			        -> pressureSensorLastStatusRead
// TimeOfLastSolenoidToggleMilliseconds		    -> timeOfLastSolenoidToggleMilliseconds
// pressure_system_input				            -> pressureSystemInput
// blower_output_speed_in_percentage		        -> blowerOutputSpeedInPercentage
// pinch_valve_output_openness_in_percentage	    -> pinchValveOutputOpennessInPercentage
// CurrPressureSetpointCentimetersH2O		        -> currPressureSetpointCentimetersH2O
// Blower_Kp					                    -> blowerKp
// Blower_Ki					                    -> blowerKi
// Blower_Kd					                    -> blowerKd
// Blower_PID					                    -> blowerPID
// PinchValve_PID                                  -> pinchValvePID



// -----------------------------------OBJECTS---------------------------------------- //
BreathCycleStep CurrCycleStep;

Servo PinchValve;

TCA9548A I2CMux;
AllSensors_DLHR_L60D_8 patientCircuitPressure(&Wire);
AllSensors_DLHR_L60D_8 venturiDifferentialPressure(&Wire);
// -----------------------------------OBJECTS---------------------------------------- //



// --------------------------------USER SETTINGS------------------------------------- //
double pressure_reading;
double venturiDifferentialPressureReading;
double venturiFlowRateLpm;
double blower_speed;

String string_from_pi;
String property_name;
String argument;
double argument_value;
String propertyNameAlarms;
String argumentAlarms;
String serialSendString;

double peep_low_alarm             = PEEP_LOW_ALARM;
double peep_alarm_undershoot      = PEEP_ALARM;
double pip_alarm_overshoot        = PIP_ALARM;

double PeepPressureCentimetersH2O = DEFAULT_PEEP;
double PipPressureCentimetersH2O  = DEFAULT_PIP;
// double FlowOfOxygen;

double PeepAlarmTimer             = DEFAULT_PEEP_TIME;
double PipAlarmTimer              = DEFAULT_PIP_TIME;
double DisconnectAlarmTimer       = DEFAULT_DISCONNECT_TIME;
// double ApneaTimer                 = DEFAULT_APNEA_TIME;

bool isBatteryActivated = false;
bool isBatterySnoozeTriggered = false;
bool isDisconnectSnoozeTriggered = false;
bool isHighLowPressureDoneOneCycle = false;
bool buzzer_state = 1;

// Adafruit_NeoPixel_ZeroDMA AlarmLED = Adafruit_NeoPixel_ZeroDMA(NUM_LEDS, LED_DATA_PIN, NEO_GRB + NEO_KHZ800);

// byte low            =   AlarmLED.Color(0, 0, 0);
// byte red            =   AlarmLED.Color(255, 0, 0);
// byte low_red        =   AlarmLED.Color(100,0,0);
// byte amber          =   AlarmLED.Color(255, 70, 0);
// byte low_amber      =   AlarmLED.Color(100,30,0);
// byte green          =   AlarmLED.Color(0, 40, 0);
// byte low_green      =   AlarmLED.Color(0,75,0);
// byte perywinkle     =   AlarmLED.Color(5 , 7, 10);  // perywinkle (pastell blue)
// byte low_perywinkle =   AlarmLED.Color(2,3,5);      // perywinkle (pastell blue)

byte alarm_state = 1;

double currentPipPressureCentimetersH2O;
double previousPipPressureCentimetersH2O;
double maxPipPressure;

double currentPeepPressureCentimetersH2O;
double previousPeepPressureCentimetersH2O;
double minPeepPressure;

double instantPressure;
double inspiratoryVolume;
double instantFlow;

double timeInspiratoryRequested;
// --------------------------------USER SETTINGS------------------------------------- //



// --------------------------------STATE TIMINGS------------------------------------- //
double CurrTimeInCycleMilliseconds;                                                                               // Time since the start of the current breath cycle. Resets at the beginning of every breath cycle
double CycleStartTimeFromSysClockMilliseconds;                                                                    // Time that the current breath cycle started ( in terms of system clock millis() )
double ControlLoopStartTimeMilliseconds;                                                                          // Time, in terms of millis(), the state machine last switched out of IDLE
double ControlLoopInitialStabilizationTimeMilliseconds = DEFAULT_CONTROL_LOOP_INIT_STABILIZATION;                 // Length of time after transitioning out of IDLE that the system waits before transitioning to INHALE_RAMP
double InhaleDurationMilliseconds                      = DEFAULT_INHALE_DURATION;                                 // Combined length of the INHALE_RAMP and INHALE_HOLD periods. AKA Value of CurrTimeInCycleMilliseconds when the state changes to EXHALE_HOLD. User configurable.
double ExhaleDurationMilliseconds                      = DEFAULT_EXHALE_DURATION;                                 // Combined length of the EXHALE_RAMP and EXHALE_HOLD periods. AKA Value of CurrTimeInCycleMilliseconds when the state changes to INHALE_HOLD. User configurable.
double BreathCycleDurationMilliseconds                 = InhaleDurationMilliseconds + ExhaleDurationMilliseconds; // Total length of breath cycle, AKA when cycle step resets to INHALE_RAMP and CurrTimeInCycleMilliseconds resets to 0

double PressureSensorLastStatusRead;

double TimeOfLastSolenoidToggleMilliseconds = 0;                                                                  // Time, in terms of millis(), that the solenoid last changed states
// --------------------------------STATE TIMINGS------------------------------------- //



// --------------------------------PID SETTINGS-------------------------------------- //
double pressure_system_input, blower_pressure_offset, pinch_valve_output_openness_in_percentage, CurrPressureSetpointCentimetersH2O;
double Blower_Kp = DEFAULT_BLOWER_KP, Blower_Ki = DEFAULT_BLOWER_KI, Blower_Kd = DEFAULT_BLOWER_KD;
double PinchValve_Kp = DEFAULT_PINCH_VALVE_KP, PinchValve_Ki = DEFAULT_PINCH_VALVE_KI, PinchValve_Kd = DEFAULT_PINCH_VALVE_KD;

PID Blower_PID(&pressure_system_input,
                &blower_pressure_offset,
                &CurrPressureSetpointCentimetersH2O,
                Blower_Kp, Blower_Ki, Blower_Kd, DIRECT);

PID PinchValve_PID(&pressure_system_input,
                &pinch_valve_output_openness_in_percentage,
                &CurrPressureSetpointCentimetersH2O,
                PinchValve_Kp, PinchValve_Kd, PinchValve_Kd, REVERSE);
// --------------------------------PID SETTINGS-------------------------------------- //
