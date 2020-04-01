// 0. Info:
  // See info tab
// 1. Libraries:
  #include "config.h"
  #include <M_dlhrDiffPress_avrBoard.h>           // DLHR differential pressure sensor library
  #include <M_KalmanFilter.h>
  #include <M_i2c_multiplexer.h>
  #include <M_ventilator_control.h>
// 2. Pins:
  //2.1. Buzzer:
      int pinBuzzer = 22;
// 3. EEPROM                                      // EEPROM entries are done byte-by-byte and number-by-number
// 4. Variables:
  // 4.1. Time
  // 4.2. Pressure
    float pressureDifferentialOxyVenturi;
    float pressureDifferentialAirVenturi;
    float pressureGaugeOxyLine;
    float pressureGaugeAirLine;  
// 5. Initialization/Instantiate Objects
  M_dlhrDiffPress_avrBoard    DlhrOxyVenturi;                // No input variables required. Sensor I2C address constant
  M_dlhrDiffPress_avrBoard    DlhrAirVenturi;                // No input variables required. Sensor I2C address constant
  M_dlhrDiffPress_avrBoard    DlhrOxyLine;                // No input variables required. Sensor I2C address constant
  M_dlhrDiffPress_avrBoard    DlhrAirLine;                // No input variables required. Sensor I2C address constant

  M_KalmanFilter              KalmanOxyVenturi(KALMAN_FILTER_VENTURI_TRANSIENT_FACTOR,KALMAN_FILTER_VENTURI_SENSOR_ERROR,KALMAN_FILTER_VENTURI_SENSOR_RANGE);     // Create KalmanFilter object. Object requires (Transient scaling factor, Manufacturer defined sensor error, total sensor range(-40 to 75degC))    -    (Transient scaling factor increases sensitivity when temperatures see a chang efrom steady state, Sensor error is best defined as 3 times the standard deviation of steady state error, total sensor range is self explanitory)
  M_KalmanFilter              KalmanAirVenturi(KALMAN_FILTER_VENTURI_TRANSIENT_FACTOR,KALMAN_FILTER_VENTURI_SENSOR_ERROR,KALMAN_FILTER_VENTURI_SENSOR_RANGE);
  M_KalmanFilter              KalmanOxyLine(KALMAN_FILTER_GAUGE_TRANSIENT_FACTOR,KALMAN_FILTER_GAUGE_SENSOR_ERROR,KALMAN_FILTER_GAUGE_SENSOR_RANGE);
  M_KalmanFilter              KalmanAirLine(KALMAN_FILTER_GAUGE_TRANSIENT_FACTOR,KALMAN_FILTER_GAUGE_SENSOR_ERROR,KALMAN_FILTER_GAUGE_SENSOR_RANGE);
  
  M_i2c_multiplexer           MultplexerPressureSensors(MULTIPLEXER_I2C_ADDRESS);             // Multiplexer address is 0x70 by default
  
  VentilatorControl        VentilatorControl(KP_AIR_PIP, KI_AIR_PIP, KD_AIR_PIP, KP_AIR_PEEP, KI_AIR_PEEP, KD_AIR_PEEP,BLOWER_ESC_PIN,ESC_PWM_MIN,ESC_PWM_MAX,EXHALATION_VALVE_SERVO_PIN,EXHALATION_VALVE_PWM_CLOSED,EXHALATION_VALVE_PWM_OPEN);  // (Kp, Ki, Kd, Kp, Ki, Kd)
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void setup(){
  Serial.begin(9600);

  // 1. Pressure sensors:
    MultplexerPressureSensors.tca_select(0);
      DlhrOxyVenturi.dlhrBegin();             // Calibrate Oxy sensor
    MultplexerPressureSensors.tca_select(1);
      DlhrAirVenturi.dlhrBegin();             // Calibrate Air sensor
    MultplexerPressureSensors.tca_select(2);
      DlhrOxyLine.dlhrBegin();            // Calibrate OxyLine sensor
    MultplexerPressureSensors.tca_select(3);
      DlhrAirLine.dlhrBegin();            // Calibrate AirLine sensor
  // 2. Blower control:
    //oBlowerEsc.esc_begin(1000,2000,2);
    VentilatorControl.controlSystemBegin();
  // 3. Buzzer:
    pinMode(pinBuzzer, OUTPUT);
    noTone(pinBuzzer);

    



  
}

void loop(){
  
  //Serial.print("t = ");         Serial.print(oClock1.clockTime());
  //Serial.print(",Freq = ");     Serial.println(oClock1.clockFreq());

  // 1. Get sensor data:
    MultplexerPressureSensors.tca_select(0);
      //pressureDifferentialOxyVenturi = DlhrOxyVenturi.getDiffPressure();             // Get pressure differential
      pressureDifferentialOxyVenturi = KalmanOxyVenturi.getKalmanState(DlhrOxyVenturi.getDiffPressure());
    MultplexerPressureSensors.tca_select(1);
      //pressureDifferentialAirVenturi = DlhrAirVenturi.getDiffPressure();             // Get pressure differential
      pressureDifferentialAirVenturi = KalmanAirVenturi.getKalmanState(DlhrAirVenturi.getDiffPressure());
    MultplexerPressureSensors.tca_select(2);
      //pressureGaugeOxyLine = DlhrOxyLine.getDiffPressure();             // Get pressure differential
      pressureGaugeOxyLine = KalmanOxyLine.getKalmanState(DlhrOxyLine.getDiffPressure());
    MultplexerPressureSensors.tca_select(3);
      //pressureGaugeAirLine = DlhrAirLine.getDiffPressure();             // Get pressure differential
      pressureGaugeAirLine = KalmanAirLine.getKalmanState(DlhrAirLine.getDiffPressure());

      
  //Serial.print(millis());
  //Serial.print(",");
  //Serial.print(DlhrAirLine.getDiffPressure());
  //Serial.print(",");
  //Serial.println(pressureGaugeAirLine);
    

    float A1 = 0.000183;
    float A2 = A1/4;
    float Cd = 0.94;
    float roh_air = 1.225;
    
    float flowRateOxy = A1*Cd*(pow(((2/roh_air)*(pressureDifferentialOxyVenturi/15.0)),2));
    float flowRateAir = A1*Cd*(pow(((2/roh_air)*(pressureDifferentialAirVenturi/15.0)),2));

    //Serial.print(millis());
    //Serial.print(",");
    
    //Serial.print("OV: ");
    //Serial.print(pressureDifferentialOxyVenturi);
    //Serial.print("  AV: ");
    //Serial.print(pressureDifferentialAirVenturi);
    //Serial.print("  OL: ");
    //Serial.print(pressureGaugeOxyLine);
    //Serial.print("  AL: ");
    //Serial.println(pressureGaugeAirLine);
    //Serial.print(",");
    //Serial.print("  OF: ");
    //Serial.print(flowRateOxy,3);
    //Serial.print("  AF: ");
    //Serial.println(flowRateAir,3);
    
    
  // 2. Blower control:
    //oBlowerEsc.set_motor_via_ide();

  // 3. Buzzer:
  /*
    float fTime = oClock1.clockTime();
    Serial.println(fTime);
    if (fTime < 5){
      tone(pinBuzzer,1000);
    }else if (fTime < 10){
      noTone(pinBuzzer);
    }else{
      oClock1.clockReset();
    }
  */

  // 4. Ventilator control
    VentilatorControl.setVentilatorMode(PRESSURE_MAX_PIP,PRESSURE_MIN_PEEP,pressureGaugeAirLine,TIME_PEEP_TO_PIP,TIME_INHALATION,TIME_EXHALATION);       // (PIP,PEEP,sensor reading,PEEP to PIP,inhalation duration,exhalation duration)








}
