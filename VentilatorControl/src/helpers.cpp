#include "includes.h"

// Floating point implementation of Math.map()
double mapf(double x, double in_min, double in_max, double out_min, double out_max)
{
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// Compensation for DC offset in pressure achieved.
// Found by linear regression of the following experimental datapoints:
// Target Val	Setpoint Req'd
// 15       	16
// 20	        21
// 25	        27.9
// 30	        33.5
// 35	        38.5
// 40	        44.5
// 45	        49.25
// 50	        55

double linearRemapSetpointCompensation (double setpoint)
{
  return (1.116785714*setpoint)-0.5892857143;
}

double returnHighestPipOverPeriod (void)
{
  currentPipPressureCentimetersH2O = PipPressureCentimetersH2O;

  if(isHighLowPressureDoneOneCycle)
  {
    previousPipPressureCentimetersH2O = currentPipPressureCentimetersH2O;
  }
  if(currentPipPressureCentimetersH2O > previousPipPressureCentimetersH2O)
  {
    highestPipPressure = currentPipPressureCentimetersH2O;
  }
  else
  {
    highestPipPressure = previousPipPressureCentimetersH2O;
  }
  return highestPipPressure;
}

double returnLowestPeepOverPeriod (void)
{
  currentPeepPressureCentimetersH2O = PeepPressureCentimetersH2O;

  if(isHighLowPressureDoneOneCycle)
  {
    previousPeepPressureCentimetersH2O = currentPeepPressureCentimetersH2O;
  }
  if(currentPeepPressureCentimetersH2O > previousPeepPressureCentimetersH2O)
  {
    lowestPeepPressure = currentPeepPressureCentimetersH2O;
  }
  else
  {
    lowestPeepPressure = previousPeepPressureCentimetersH2O;
  }
  return lowestPeepPressure;
}
