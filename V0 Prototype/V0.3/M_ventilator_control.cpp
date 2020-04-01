// ----------------------------------------------------------------------------------------------------------------
// C++ FILE (.cpp)
// ----------------------------------------------------------------------------------------------------------------
#include "M_ventilator_control.h"

VentilatorControl::VentilatorControl(const float KP_AIR_PIP, const float KI_AIR_PIP, const float KD_AIR_PIP, const float KP_AIR_PEEP, const float KI_AIR_PEEP, const float KD_AIR_PEEP, const int escPin, const int escPwmMin, const int escPwmMax, const int exhalationValveServoPin, const int exhalationValvePWMClosed, const int exhalationValvePWMOpen, const int pinBuzzer){
  M_clock ClockBreathCycle;
  M_clock ClockAlarmLowPressure;
  M_clock ClockAlarmHighPressure;
  M_clock ClockAlarmDisconnect;
  ClockBreathCycle.clockBegin();
  ClockAlarmLowPressure.clockBegin();
  ClockAlarmHighPressure.clockBegin();
  ClockAlarmDisconnect.clockBegin();
  
  _KP_AIR_PIP = KP_AIR_PIP;         // max pressure
  _KI_AIR_PIP = KI_AIR_PIP;
  _KD_AIR_PIP = KD_AIR_PIP;
  _KP_AIR_PEEP = KP_AIR_PEEP;       // min pressure
  _KI_AIR_PEEP = KI_AIR_PEEP;
  _KD_AIR_PEEP = KD_AIR_PEEP;

  _pinBuzzer = pinBuzzer;
  pinMode(_pinBuzzer, OUTPUT);
  noTone(_pinBuzzer);
  
  _escPin = escPin;
  _escPwmMin = escPwmMin;
  _escPwmMax = escPwmMax;
  Servo _escObject;

  
  _exhalationValveServoPin = exhalationValveServoPin;
  _exhalationValvePWMClosed = exhalationValvePWMClosed;
  _exhalationValvePWMOpen = exhalationValvePWMOpen;
  Servo _servoObject;
}
void VentilatorControl::controlSystemBegin(){
  _escObject.attach(_escPin);
  VentilatorControl::blowerPwmSignalOut(_escPwmMin);

  _servoObject.attach(_exhalationValveServoPin);
  VentilatorControl::servoPwmSignalOut(_escPwmMin);

  
  delay(2000);
}
// ---------------------------------------------------------------------
void VentilatorControl::blowerPwmSignalOut(int pulseLength){
  if (pulseLength < _escPwmMin) pulseLength = _escPwmMin;
  if (pulseLength > _escPwmMax) pulseLength = _escPwmMax;
  _escObject.writeMicroseconds(pulseLength);
}
void VentilatorControl::servoPwmSignalOut(int pulseLength){
  if (pulseLength < _exhalationValvePWMClosed) pulseLength = _exhalationValvePWMClosed;
  if (pulseLength > _exhalationValvePWMOpen) pulseLength = _exhalationValvePWMOpen;
  _servoObject.writeMicroseconds(pulseLength);
}
// ---------------------------------------------------------------------
void VentilatorControl::controlSignalBlower(float targetPressure, float currentSensorPressure, float loopFrequency, const int pressureMaxPip, const int pressureMinPeep){

  float pidPressrureError = targetPressure-currentSensorPressure;
  // Proportional -----------------------------------------------------------------------------------------------
  _P = _KP_AIR_PIP*pidPressrureError;
  // Integral -----------------------------------------------------------------------------------------------
  if (pidPressrureError > -1000 && pidPressrureError < 1000){
    _I = _I + _KI_AIR_PIP*pidPressrureError;    
  }
  // PUT LIMITS ON I!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
  // Derivative -----------------------------------------------------------------------------------------------
  _D =  _KD_AIR_PIP*(currentSensorPressure - _lastLoopSensorPressure)/(1/loopFrequency);
  _lastLoopSensorPressure = currentSensorPressure;
  // PID signal -----------------------------------------------------------------------------------------------
  float blowerOutputSignal = _P + _I - _D;
  
  // Signal scaling  -----------------------------------------------------------------------------------------------
  int pulseLengthEsc = (1000/0.01)*blowerOutputSignal + 1000;
  if (pulseLengthEsc > _escPwmMax) pulseLengthEsc = _escPwmMax;
  if (pulseLengthEsc < _escPwmMin) pulseLengthEsc = _escPwmMin;

  //if (millis()%10<=1){
    //Serial.print(100000*_P,6);
    //Serial.print(",");
    //Serial.print(100000*_I,6);
    //Serial.print(",");
    //Serial.print(-100000*_D,6);
    //Serial.print(",");

    
    //Serial.print(millis());
    //Serial.print(",");
    Serial.println(currentSensorPressure);
    //Serial.print(",");
    //Serial.print(targetPressure);
    //Serial.print(",");
    //Serial.print(pidPressrureError);
    //Serial.print(",");
    //Serial.println(pulseLengthEsc);
    
  //}
  
  VentilatorControl::blowerPwmSignalOut(pulseLengthEsc);

}
void VentilatorControl::controlSignalBlowerPip(float targetPressure, float currentSensorPressure, float loopFrequency, const int pressureMaxPip, const int pressureMinPeep){   // max pressure
  
  float pidPressrureError = targetPressure-currentSensorPressure;
  // Proportional -----------------------------------------------------------------------------------------------
  _Ppip = _KP_AIR_PIP*pidPressrureError;
  // Integral -----------------------------------------------------------------------------------------------
  if (pidPressrureError > -1000 && pidPressrureError < 1000){
    _Ipip = _Ipip + _KI_AIR_PIP*pidPressrureError;    
  }
  if (_Ipip > 25) _Ipip = 25;               // These are rough values to prevent I runaway when device is not connected or plugged, leaking or something else. 
  if (_Ipip < 0) _Ipip = 0;                 // These values are roughly in the ballpark but need finalization. 
  // Derivative -----------------------------------------------------------------------------------------------
  _Dpip =  _KD_AIR_PIP*(currentSensorPressure - _lastLoopSensorPressure)/(1/loopFrequency);
  _lastLoopSensorPressure = currentSensorPressure;
  // PID signal -----------------------------------------------------------------------------------------------
  float blowerOutputSignal = _Ppip + _Ipip - _Dpip;
  
  // Signal scaling  -----------------------------------------------------------------------------------------------
  int pulseLengthEsc = (1000/0.01)*blowerOutputSignal + 1000;
  if (pulseLengthEsc > _escPwmMax) pulseLengthEsc = _escPwmMax;
  if (pulseLengthEsc < _escPwmMin) pulseLengthEsc = _escPwmMin;

  //if (millis()%10<=1){
    //Serial.print(100000*_P,6);
    //Serial.print(",");
    //Serial.print(100000*_I,6);
    //Serial.print(",");
    //Serial.print(-100000*_D,6);
    //Serial.print(",");

    //Serial.print(millis());
    //Serial.print(",");
    //Serial.println(currentSensorPressure);
    //Serial.print(",");
    //Serial.print(targetPressure);
    //Serial.print(",");
    //Serial.print(pidPressrureError);
    //Serial.print(",");
    //Serial.println(pulseLengthEsc);
  //}
  VentilatorControl::blowerPwmSignalOut(pulseLengthEsc);
}
void VentilatorControl::controlSignalBlowerPeep(float targetPressure, float currentSensorPressure, float loopFrequency, const int pressureMaxPip, const int pressureMinPeep){  // min pressure
  
  float pidPressrureError = targetPressure-currentSensorPressure;
  // Proportional -----------------------------------------------------------------------------------------------
  _Ppeep = _KP_AIR_PEEP*pidPressrureError;
  // Integral -----------------------------------------------------------------------------------------------
  if (pidPressrureError > -1000 && pidPressrureError < 1000){
    _Ipeep = _Ipeep + _KI_AIR_PEEP*pidPressrureError;    
  }
  if (_Ipeep > 25) _Ipeep = 25;               // These are rough values to prevent I runaway when device is not connected or plugged, leaking or something else. 
  if (_Ipeep < 0) _Ipeep = 0;                 // These values are roughly in the ballpark but need finalization. 
  // Derivative -----------------------------------------------------------------------------------------------
  _Dpeep =  _KD_AIR_PEEP*(currentSensorPressure - _lastLoopSensorPressure)/(1/loopFrequency);
  _lastLoopSensorPressure = currentSensorPressure;
  // PID signal -----------------------------------------------------------------------------------------------
  float blowerOutputSignal = _Ppeep + _Ipeep - _Dpeep;
  
  // Signal scaling  -----------------------------------------------------------------------------------------------
  int pulseLengthEsc = (1000/0.01)*blowerOutputSignal + 1000;
  if (pulseLengthEsc > _escPwmMax) pulseLengthEsc = _escPwmMax;
  if (pulseLengthEsc < _escPwmMin) pulseLengthEsc = _escPwmMin;

  //if (millis()%10<=1){
    //Serial.print(100000*_P,6);
    //Serial.print(",");
    //Serial.print(100000*_I,6);
    //Serial.print(",");
    //Serial.print(-100000*_D,6);
    //Serial.print(",");

    //Serial.print(millis());
    //Serial.print(",");
    //Serial.println(currentSensorPressure);
    //Serial.print(",");
    //Serial.print(targetPressure);
    //Serial.print(",");
    //Serial.print(pidPressrureError);
    //Serial.print(",");
    //Serial.println(pulseLengthEsc);
  //}
  
  VentilatorControl::blowerPwmSignalOut(pulseLengthEsc);
}
// ---------------------------------------------------------------------// ---------------------------------------------------------------------
void VentilatorControl::overPressureRelease(float targetPressure, float currentSensorPressure){                                           // This function tracks the the breath cycle, and when there is a significant pressure drop (due to interference) the function will reset the breath cycle. 
                                                                                                                                          // 
  if (currentSensorPressure >= targetPressure){
    int overPressureSensitivity = 250;                               // This is the value above the maximum pressure at which the exhalation valve is completely open
    int servoPressureReleasePulseWidth = (80/overPressureSensitivity)*currentSensorPressure + _exhalationValvePWMClosed - (80/overPressureSensitivity)*targetPressure;
    servoPwmSignalOut(servoPressureReleasePulseWidth);
  }else{
    int servoPressureReleasePulseWidth = _exhalationValvePWMClosed;
    servoPwmSignalOut(servoPressureReleasePulseWidth);
  }
  
}
int VentilatorControl::gaspDetection(){                                           // This function tracks the the breath cycle, and when there is a significant pressure drop (due to interference) the function will reset the breath cycle. 
  //trigger with flow influx
}
// ---------------------------------------------------------------------
void VentilatorControl::alarmLowPressure(const float pressureGaugeAirLine,const int pressureMaxPip, const int pressureMinPeep){                                           // This function tracks the the breath cycle, and when there is a significant pressure drop (due to interference) the function will reset the breath cycle. 
  // Low pressure alarm
  float peepTolerance = 0.9;                      // i.e. 10% below set pressure
  if (pressureGaugeAirLine < peepTolerance*pressureMinPeep){
    if (ClockAlarmLowPressure.clockTime() > 2.0){
      tone(_pinBuzzer,1000);
    }
  }else{
    ClockAlarmLowPressure.clockReset();
    noTone(_pinBuzzer);
  }  
}
void VentilatorControl::alarmHighPressure(const float pressureGaugeAirLine,const int pressureMaxPip, const int pressureMinPeep){                                           // This function tracks the the breath cycle, and when there is a significant pressure drop (due to interference) the function will reset the breath cycle. 
  // High pressure alarm
  float pipTolerance = 1.1;                      // i.e. 10% above set pressure
  if (pressureGaugeAirLine > pipTolerance*pressureMaxPip){
    if (ClockAlarmDisconnect.clockTime() > 2.0){
      tone(_pinBuzzer,1500);
    }
  }else{
    ClockAlarmDisconnect.clockReset();
    noTone(_pinBuzzer);
  }
}
void VentilatorControl::alarmDisconnect(const float pressureDifferentialAirVenturi,const int pressureMaxPip, const int pressureMinPeep){                                           // This function tracks the the breath cycle, and when there is a significant pressure drop (due to interference) the function will reset the breath cycle. 
  // Disconnect alarm
  
  float A1 = 0.00053666;
  float A2 = A1/4;
  float Cd = 0.94;
  float roh_air = 1.225;
  float flowRateAir = A1*Cd*(pow(((2/roh_air)*(pressureDifferentialAirVenturi/15.0)),0.5));
  //Serial.print(ClockAlarmDisconnect.clockTime(),5);
  //Serial.print(",");
  //Serial.print(flowRateAir - 0.002,5);
  //Serial.print(",");
  //Serial.println(flowRateAir,6);
  
  float flowRateDisconnectLimit = 0.0032;      // The flow rate only achievable if there is no lung/patient connected. 
  //Serial.print(flowRateAir,6);
  //Serial.print(",");
  //Serial.print(flowRateAir-flowRateDisconnectLimit,6);
  //Serial.print(",");
  //Serial.println(ClockAlarmDisconnect.clockTime());
  
  if (flowRateAir - flowRateDisconnectLimit > 0){
    if (ClockAlarmDisconnect.clockTime() > 2.0){
      tone(_pinBuzzer,500);
    }
  }else{
    ClockAlarmDisconnect.clockReset();
    noTone(_pinBuzzer);
  }
}
// ---------------------------------------------------------------------
void VentilatorControl::breathCycle(const int pressureMaxPip, const int pressureMinPeep, const float pressureGaugeAirLine, const float pressureDifferentialAirVenturi,const int timePeepToPip, int timeInhalation, int timeExhalation){
  float timerBreathCycle = ClockBreathCycle.clockTime();
  float loopFrequency = ClockBreathCycle.clockFreq();
  
  //Serial.print(pressureGaugeAirLine);
  
  //timeInhalation += timePeepToPip;
  timeExhalation += timeInhalation;

  //if (timerBreathCycle > 0 && timerBreathCycle <= timePeepToPip){                                               // rise - hospital mentioned this would be a nice to have. currently not used thus setting timePeepToPip = 0
  //  float targetPressure = (pressureMaxPip-pressureMinPeep)/timePeepToPip*timerBreathCycle + pressureMinPeep;
  //  VentilatorControl::controlSignalBlower(targetPressure,pressureGaugeAirLine,loopFrequency,pressureMaxPip,pressureMinPeep);
  //  servoPwmSignalOut(_exhalationValvePWMClosed);
  //}
  if (timerBreathCycle > timePeepToPip && timerBreathCycle <= timeInhalation){                              // inhalation
    float targetPressure = pressureMaxPip;
    VentilatorControl::controlSignalBlowerPip(targetPressure,pressureGaugeAirLine,loopFrequency,pressureMaxPip,pressureMinPeep);
    servoPwmSignalOut(_exhalationValvePWMClosed);
  }
  else if (timerBreathCycle > timeInhalation && timerBreathCycle <= timeExhalation){                            // exhalation
    float targetPressure = pressureMinPeep;
    VentilatorControl::controlSignalBlowerPeep(targetPressure,pressureGaugeAirLine,loopFrequency,pressureMaxPip,pressureMinPeep);
    servoPwmSignalOut(_exhalationValvePWMOpen);
  }
  else{
    float targetPressure = pressureMinPeep;
    VentilatorControl::controlSignalBlowerPeep(targetPressure,pressureGaugeAirLine,loopFrequency,pressureMaxPip,pressureMinPeep);
    servoPwmSignalOut(_exhalationValvePWMOpen);
    ClockBreathCycle.clockReset();
  }
  //alarmHighPressure(pressureGaugeAirLine,pressureMaxPip,pressureMinPeep);
  alarmLowPressure(pressureGaugeAirLine,pressureMaxPip,pressureMinPeep);
  alarmDisconnect(pressureDifferentialAirVenturi,pressureMaxPip,pressureMinPeep);
}
// ---------------------------------------------------------------------
void VentilatorControl::setVentilatorMode(const int pressureMaxPip,const int pressureMinPeep, const float pressureGaugeAirLine, const float pressureDifferentialAirVenturi, const int timePeepToPip, const int timeInhalation, const int timeExhalation){

  int i = 0;
  while (Serial.available()){
    char c = Serial.read();                     //gets one byte from serial buffer
    readString += c;                              //makes the string readString
    if (c == '\n'){
      bufferSerialMonitor[i] = '\0';
      //_mode = atoi(bufferSerialMonitor);
      break;
    }

    bufferSerialMonitor[i] = c;
    i += 1;
        
    delay(2);  //slow looping to allow buffer to fill with next character
  }
  int _mode = atoi(bufferSerialMonitor);

  //Serial.print(millis());
  //Serial.print(",");
  //Serial.print(_mode);
  //Serial.print(",");
  //Serial.println(pressureGaugeAirLine);
  //_escObject.writeMicroseconds(_mode);

  _mode = 1;
  if (_mode == 0){
    _escObject.writeMicroseconds(_escPwmMin);
    servoPwmSignalOut(_exhalationValvePWMOpen);
  }
  if (_mode == 1){
    VentilatorControl::breathCycle(pressureMaxPip,pressureMinPeep,pressureGaugeAirLine,pressureDifferentialAirVenturi,timePeepToPip,timeInhalation,timeExhalation);
  }
}
// ---------------------------------------------------------------------
// ---------------------------------------------------------------------
