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

void returnHighestPipOverPeriod (void)
{
  currentPipPressureCentimetersH2O = PipPressureCentimetersH2O;

  if(isHighLowPressureDoneOneCycle)
  {
    previousPipPressureCentimetersH2O = currentPipPressureCentimetersH2O;
  }
  if(currentPipPressureCentimetersH2O > previousPipPressureCentimetersH2O)
  {
    maxPipPressure = currentPipPressureCentimetersH2O;
  }
  else
  {
    maxPipPressure = previousPipPressureCentimetersH2O;
  }
}

void returnLowestPeepOverPeriod (void)
{
  currentPeepPressureCentimetersH2O = PeepPressureCentimetersH2O;

  if(isHighLowPressureDoneOneCycle)
  {
    previousPeepPressureCentimetersH2O = currentPeepPressureCentimetersH2O;
  }
  if(currentPeepPressureCentimetersH2O < previousPeepPressureCentimetersH2O)
  {
    minPeepPressure = currentPeepPressureCentimetersH2O;
  }
  else
  {
    minPeepPressure = previousPeepPressureCentimetersH2O;
  }
}

void returnInstantPressure (void)
{
  instantPressure = pressure_reading;
}

void returnInspiratoryVolume (void)
{
  inspiratoryVolume = instantPressure * InhaleDurationMilliseconds;  // PROPER VALUE??
}

void returnInstantFlow(void)
{
  instantFlow = instantPressure * InhaleDurationMilliseconds;  // PROPER VALUE??
}
