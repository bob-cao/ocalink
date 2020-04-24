#ifndef __VENTILATOR_INCLUDES_H__
#define __VENTILATOR_INCLUDES_H__

#include <Arduino.h>

#include "Wire.h"
#include "TCA9548A.h"
#include "AllSensors_DLHR.h"
#include "Servo.h"
#include "PID_v1.h"
#include "Adafruit_NeoPixel.h"

#include "Types.h"
#include "variables.h"
#include "Pins.h"
#include "Constants.h"

#include "timing.h"
#include "compute_comms.h"
#include "helpers.h"
#include "inits.h"
#include "status_led.h"
#include "sensors.h"
#include "actuators.h"
#include "alarms.h"
#include "states.h"
#include "debug.h"

#endif  // __VENTILATOR_INCLUDES_H__
