#ifndef __CONSTANTS_H__
#define __CONSTANTS_H__

#define DEFAULT_BAUD_RATE       256000
#define ALARM_TIME              (double)500
#define SERIAL_SEND_TIME        (double)500
#define SNOOZE_TIME             (double)120000  // Snooze time in ms (2 minutes)

#define INCHES_2_CM     (double)2.54

#define MIN_PERCENTAGE  (double)0
#define MAX_PERCENTAGE  (double)100

#define PATIENT_CIRCUIT_PRESSURE_MUX_CHANNEL        0
#define VENTRUI_DIFFERENTIAL_PRESSURE_MUX_CHANNEL   1

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
#define INSP_MIN_RECEIVE    0.5
#define INSP_MAX_RECEIVE    5
#define RR_MIN_RECEIVE      5
#define RR_MAX_RECEIVE      50
#define IE_MIN_RECEIVE      1
#define IE_MAX_RECEIVE      4

#define DEFAULT_INSP_SCALING_FACTOR    10
// #define DEFAULT_FIO2_SCALING_FACTOR     100

#endif // __CONSTANTS_H__
