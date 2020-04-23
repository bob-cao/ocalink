#ifndef __ACTUATORS_H__
#define __ACTUATORS_H__

double blowerPressureToBlowerSpeed (double pressure);
void writeBlowerSpeed(void);
void writePinchValveOpenness (double opennessPercentage);
void pinchValveControl (void);

#endif // __ACTUATORS_H__
