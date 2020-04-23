#ifndef __TYPES_H__
#define __TYPES_H__

#include "includes.h"

typedef enum{
    INHALE_RAMP,
    INHALE_HOLD,
    EXHALE_RAMP,
    EXHALE_HOLD,
    IDLE
}BreathCycleStep;

#endif  // __TYPES_H__
