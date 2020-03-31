// ----------------------------------------------------------------------------------------------------------------
// C++ FILE (.cpp)
// ----------------------------------------------------------------------------------------------------------------
#include "M_KalmanFilter.h"


M_KalmanFilter::M_KalmanFilter(float fScalingFactor, float fSensorError, float fTotalSensorRange){
  
  _fScalingFactor = fScalingFactor;             // This constant scales the transient error added to the covariance matrix in order to increase Kalman Gain so that readings a weighted more again. This requires testing to fine tune
  _fSensorError = fSensorError;                 // This is the manufacturer specified error of a sensor. 
  _fTotalSensorRange = fTotalSensorRange;       // This is the total range that the sensor is expected to operate under. Same units as reading should be inputed. 
  
}

float M_KalmanFilter::getKalmanState(float fSensorReading){
  
  // Predict:
    //_fX = _fX_old;                                                                        // fX = fX_n-1
    // Determine transient error in covariance matrix                                     // fDelta is the calculated change to the covariance matrix so that the Kalman Gain can be increased depending on the difference between current predicted values and current sensor readings
      _fDelta = _fScalingFactor*(pow(((_fX-fSensorReading)/_fTotalSensorRange),2));      // fDelta = const*( (diff/max)^n )
      if (_fDelta > 1) _fDelta = 1;
    _fP = _fP_old + _fDelta;                                                              // fP = fP_n-1 + fDelta
  // Update:
    _fKG = (_fP/(_fP+_fSensorError));                                                      // fK = (fP/(fP+R))
    _fX = _fX_old + _fKG*(fSensorReading - _fX_old);                                        // fX = fX_n-1 + fK*(RawReading - fX_n-1)
    _fX_old = _fX;
  // Reset:
    _fP = (1-_fKG)*_fP;                                                                    // fP = (1-fK)*fP_n-1
    _fP_old = _fP;

    return _fX;
}
void M_KalmanFilter::initialEstimate(float fInitialEstimate){
  _fX_old = fInitialEstimate;
}
