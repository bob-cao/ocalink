// ----------------------------------------------------------------------------------------------------------------
// HEADER FILE (.h)
// ----------------------------------------------------------------------------------------------------------------
/* This library contains a sensor intended Kalman filter including transient reading filtering
 * - A great explination of the workings of the filter can be found here: https://www.youtube.com/watch?v=-cD7WkbAIL0
 * - While this is a 1D model, the terminology of a multidimensional matrix model have been used
 * 
 * M_KalmanFilter(float fScalingFactor, float fSensorError, float fTotalSensorRange);
 * - Calls for 3 inputs
 *    1. Scaling factor: The scaling factor makes the transient response more or less sensitive to a change in the filtered reading. 
 *    2. Sensor Error: This is the manufacrurer defined sensor error. For best sensor results its good practice to take 3 times standard deviation for this value for decent time responses
 *    3. Total Sensor Range: This is used to normalize the transient response
 * 
 * 
 */

#ifndef M_KalmanFilter_h                         // To prevent the main program of calling this header (*.h) several times one cas to put in the following two lines before the code and the #endif statement at the end. 
#define M_KalmanFilter_h                         // Note the different syntax for the #ifndef (if not defined)

#include <Arduino.h>

class M_KalmanFilter{
  public:
    M_KalmanFilter(float fScalingFactor, float fSensorError, float fTotalSensorRange);

    float getKalmanState(float fSensorReading);
    void initialEstimate(float fInitialEstimate);
    

  private:
    float _fX;                  // Estimate or predicted value
    float _fX_old;
    float _fP;                  // Covariance matrix, this is the error in the estimate
    float _fP_old;
    float _fDelta;              // Addition to the covariance matrix for a up-to-date variation in the size of the covariance matrix sothat it can increase the Kalrman Gain when the Delta is large
    float _fKG;                 // Kalman gain
    float _fR;                  // Error in readings or sensor noice covariance matrix
    float _fY;                  // Measurments
  
    float _fScalingFactor;      // This constant scales the transient error added to the covariance matrix in order to increase Kalman Gain so that readings a weighted more again. This requires testing to fine tune
    float _fSensorError;        // This is the manufacturer specified error of a sensor. 
    float _fTotalSensorRange;   // This is the total range that the sensor is expected to operate under. Same units as reading should be inputed. 

};
#endif
