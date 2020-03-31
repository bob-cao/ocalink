// ----------------------------------------------------------------------------------------------------------------
// HEADER FILE (.h)
// ----------------------------------------------------------------------------------------------------------------

#ifndef M_ventilator_control_h                         // To prevent the main program of calling this header (*.h) several times one cas to put in the following two lines before the code and the #endif statement at the end. 
#define M_ventilator_control_h                         // Note the different syntax for the #ifndef (if not defined)

#include <Arduino.h>
#include <M_clock.h>                            // My own clock library

#include <Servo.h>


class VentilatorControl{
  public:
    VentilatorControl(const float KP_AIR_PIP, const float KI_AIR_PIP, const float KD_AIR_PIP, const float KP_AIR_PEEP, const float KI_AIR_PEEP, const float KD_AIR_PEEP, const int escPin, const int escPwmMin, const int escPwmMax, const int exhalationValveServoPin, const int _exhalationValvePWMClosed, const int _iExhalationValvePWMOpen);

    void controlSystemBegin();

    void blowerPwmSignalOut(int iPulseLength);
    void servoPwmSignalOut(int iPulseLength);
    
    void controlSignalBlower(float targetPressure, float currentPressure, float loopFrequency, const int pressureMaxPip, const int pressureMinPeep, const int breathStage);
    void controlSignalBlower();
    void breathCycle(const int pressureMaxPip, const int pressureMinPeep, const float pressureGaugeAirLine, const int timePeepToPip, int timeInhalation, int timeExhalation);

    void setVentilatorMode(const int pressureMaxPip,const int pressureMinPeep, const float pressureGaugeAirLine, const int timePeepToPip, const int timeInhalation, const int timeExhalation);
    int gaspDetection();

  private:
    float _KP_AIR_PIP;
    float _KI_AIR_PIP;
    float _KD_AIR_PIP;
    float _KP_AIR_PEEP;
    float _KI_AIR_PEEP;
    float _KD_AIR_PEEP;

    float _P = 0;
    float _PlastLoop = 0;
    float _I = 0;
    float _D = 0;

    M_clock ClockBreathCycle;
    Servo _escObject;
    Servo _servoObject;
    
    char bufferSerialMonitor[255];
    String readString;
    int _mode = 0;
    
    int _escPin;
    int _escPwmMin;
    int _escPwmMax;

    int _exhalationValveServoPin;
    int _exhalationValvePWMClosed;
    int _exhalationValvePWMOpen;

    float _lastLoopSensorPressure = 0;;
    float _lastLoopPidPressureError = 0;
  
};
#endif
