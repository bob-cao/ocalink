#include "includes.h"

// Floating point implementation of Math.map()
double mapf(double x, double in_min, double in_max, double out_min, double out_max)
{
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// Floating point implementation of std::clamp()
double clampf(double x, double out_min, double out_max)
{
    return (x<out_min)?out_min:((x>out_max)?out_max:x);
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
  instantFlow = venturiFlowRateLpm;
}
