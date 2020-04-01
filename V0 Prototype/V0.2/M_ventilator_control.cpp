// ----------------------------------------------------------------------------------------------------------------
// C++ FILE (.cpp)
// ----------------------------------------------------------------------------------------------------------------
#include "M_ventilator_control.h"

VentilatorControl::VentilatorControl(const float KP_AIR_PIP, const float KI_AIR_PIP, const float KD_AIR_PIP, const float KP_AIR_PEEP, const float KI_AIR_PEEP, const float KD_AIR_PEEP, const int escPin, const int escPwmMin, const int escPwmMax, const int exhalationValveServoPin, const int exhalationValvePWMClosed, const int exhalationValvePWMOpen){
  M_clock ClockBreathCycle;
  ClockBreathCycle.clockBegin();
  
  _KP_AIR_PIP = KP_AIR_PIP;
  _KI_AIR_PIP = KI_AIR_PIP;
  _KD_AIR_PIP = KD_AIR_PIP;
  _KP_AIR_PEEP = KP_AIR_PEEP;
  _KI_AIR_PEEP = KI_AIR_PEEP;
  _KD_AIR_PEEP = KD_AIR_PEEP;
  
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
// ---------------------------------------------------------------------
void VentilatorControl::servoPwmSignalOut(int pulseLength){
  _servoObject.writeMicroseconds(pulseLength);
}
// ---------------------------------------------------------------------
void VentilatorControl::controlSignalBlower(float targetPressure, float currentSensorPressure, float loopFrequency, const int pressureMaxPip, const int pressureMinPeep, const int breathStage){

  float pidPressrureError = targetPressure-currentSensorPressure;
  // Proportional -----------------------------------------------------------------------------------------------
  _P = _KP_AIR_PIP*pidPressrureError;
  // Integral -----------------------------------------------------------------------------------------------
  if (pidPressrureError > -1000 && pidPressrureError < 1000){
    _I = _I + _KI_AIR_PIP*pidPressrureError;    
  }
  // Derivative -----------------------------------------------------------------------------------------------
  _D =  _KD_AIR_PIP*(currentSensorPressure - _lastLoopSensorPressure)/(1/loopFrequency);
  _lastLoopSensorPressure = currentSensorPressure;
  // PID signal -----------------------------------------------------------------------------------------------
  float blowerOutputSignal = _P + _I - _D;
  
    //Serial.print(100000*_P,6);
    //Serial.print(",");
    //Serial.print(100000*_I,6);
    //Serial.print(",");
    //Serial.print(-100000*_D,6);
    //Serial.print(",");
  // Signal scaling  -----------------------------------------------------------------------------------------------
  int pulseLengthEsc = (1000/0.01)*blowerOutputSignal + 1000;
  if (pulseLengthEsc > _escPwmMax) pulseLengthEsc = _escPwmMax;
  if (pulseLengthEsc < _escPwmMin) pulseLengthEsc = _escPwmMin;
  
    Serial.println(currentSensorPressure);
    //Serial.print(",");
    //Serial.print(targetPressure);
    //Serial.print(",");
    //Serial.print(pidPressrureError);
    //Serial.print(",");
    //Serial.println(pulseLengthEsc);
  
  VentilatorControl::blowerPwmSignalOut(pulseLengthEsc);

}
// ---------------------------------------------------------------------
void VentilatorControl::controlSignalBlower(){
  
}
// ---------------------------------------------------------------------
void VentilatorControl::breathCycle(const int pressureMaxPip, const int pressureMinPeep, const float pressureGaugeAirLine, const int timePeepToPip, int timeInhalation, int timeExhalation){
  
  float timerBreathCycle = ClockBreathCycle.clockTime();
  float loopFrequency = ClockBreathCycle.clockFreq();
  
  //timeInhalation += timePeepToPip;
  timeExhalation += timeInhalation;

  //if (timerBreathCycle > 0 && timerBreathCycle <= timePeepToPip){                                               // rise - hospital mentioned this would be a nice to have. currently not used due to setting timePeepToPip = 0
  //  float targetPressure = (pressureMaxPip-pressureMinPeep)/timePeepToPip*timerBreathCycle + pressureMinPeep;
  //  VentilatorControl::controlSignalBlower(targetPressure,pressureGaugeAirLine,loopFrequency,pressureMaxPip,pressureMinPeep,0);
  //  servoPwmSignalOut(_exhalationValvePWMClosed);
  //}
  if (timerBreathCycle > timePeepToPip && timerBreathCycle <= timeInhalation){                              // inhalation
    float targetPressure = pressureMaxPip;
    VentilatorControl::controlSignalBlower(targetPressure,pressureGaugeAirLine,loopFrequency,pressureMaxPip,pressureMinPeep,1);
    servoPwmSignalOut(_exhalationValvePWMClosed);
  }
  else if (timerBreathCycle > timeInhalation && timerBreathCycle <= timeExhalation){                            // exhalation
    float targetPressure = pressureMinPeep;
    VentilatorControl::controlSignalBlower(targetPressure,pressureGaugeAirLine,loopFrequency,pressureMaxPip,pressureMinPeep,2);
    servoPwmSignalOut(_exhalationValvePWMOpen);
  }
  else{
    float targetPressure = pressureMinPeep;
    VentilatorControl::controlSignalBlower(targetPressure,pressureGaugeAirLine,loopFrequency,pressureMaxPip,pressureMinPeep,2);
    servoPwmSignalOut(_exhalationValvePWMOpen);
    ClockBreathCycle.clockReset();
    //Serial.print("clock reset");
  }
}
// ---------------------------------------------------------------------
void VentilatorControl::setVentilatorMode(const int pressureMaxPip,const int pressureMinPeep, const float pressureGaugeAirLine, const int timePeepToPip, const int timeInhalation, const int timeExhalation){

  
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
    VentilatorControl::breathCycle(pressureMaxPip,pressureMinPeep,pressureGaugeAirLine,timePeepToPip,timeInhalation,timeExhalation);
  }
}
// ---------------------------------------------------------------------
int VentilatorControl::gaspDetection(){                                           // This function tracks the the breath cycle, and when there is a significant pressure drop (due to interference) the function will reset the breath cycle. 
  //trigger with flow influx
}
// ---------------------------------------------------------------------
// ---------------------------------------------------------------------
