#ifndef config_h                         // To prevent the main program of calling this header (*.h) several times one cas to put in the following two lines before the code and the #endif statement at the end. 
#define config_h                         // Note the different syntax for the #ifndef (if not defined)



#define KALMAN_FILTER_VENTURI_TRANSIENT_FACTOR 15
#define KALMAN_FILTER_VENTURI_SENSOR_ERROR 2
#define KALMAN_FILTER_VENTURI_SENSOR_RANGE 600
#define KALMAN_FILTER_GAUGE_TRANSIENT_FACTOR 200
#define KALMAN_FILTER_GAUGE_SENSOR_ERROR 1
#define KALMAN_FILTER_GAUGE_SENSOR_RANGE 10000

#define MULTIPLEXER_I2C_ADDRESS 0x70

#define PRESSURE_MAX_PIP 3000           // Pascal gauge
#define PRESSURE_MIN_PEEP 800
#define TIME_PEEP_TO_PIP 0
#define TIME_INHALATION 3
#define TIME_EXHALATION 3
#define DISCONNECT_FLOW_RATE 0.5

#define BLOWER_ESC_PIN 2
#define ESC_PWM_MIN 1000
#define ESC_PWM_MAX 2000

#define EXHALATION_VALVE_SERVO_PIN 3
#define EXHALATION_VALVE_PWM_CLOSED 970
#define EXHALATION_VALVE_PWM_OPEN 1200


const float KP_AIR_PIP = 0.0000011;          // PID values used for pressure control when pressure goes from PEEP to PIP
const float KI_AIR_PIP = 0.00000005;
const float KD_AIR_PIP = 0.0000000;
//const float KP_AIR_PIP = 0.0000011;          // PID values used for pressure control when pressure goes from PEEP to PIP
//const float KI_AIR_PIP = 0.00000005;
//const float KD_AIR_PIP = 0.0000000;

const float KP_AIR_PEEP = 0.000003;         // PID values used for pressure control when pressure goes from PIP to PEEP
const float KI_AIR_PEEP = 0.0000002;
const float KD_AIR_PEEP = 0.000001;
//const float KP_AIR_PEEP = 0.000003;         // PID values used for pressure control when pressure goes from PIP to PEEP
//const float KI_AIR_PEEP = 0.0000002;
//const float KD_AIR_PEEP = 0.000001;

#endif
