// Handy helper functions that don't belong in the main control loop file

#ifndef __HELPERS_H__
#define __HELPERS_H__

double mapf(double x, double in_min, double in_max, double out_min, double out_max);
double linearRemapSetpointCompensation (double setpoint);
void returnHighestPipOverPeriod (void);
void returnLowestPeepOverPeriod (void);

#endif // __HELPERS_H__
