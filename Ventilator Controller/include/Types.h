// Define types used across the controller
#ifndef _VENTILATOR_TYPES_H_
#define _VENTILATOR_TYPES_H_

#include <cstdint>

typedef enum{
    INHALE_RAMP,
    INHALE_HOLD,
    EXHALE
}BreathCycleStep;

#define NUM_BREATHCYCLE_STEPS (BreathCycleStep::EXHALE+1)

const char* BreathCycleStepNames[NUM_BREATHCYCLE_STEPS]
{
    "INHALE_RAMP",
    "INHALE_HOLD",
    "EXHALE"
};

typedef struct{
    BreathCycleStep CurrCycleStep;
    uint32_t CycleStartTimeFromSysClockMilliseconds;
    uint32_t CurrTimeInCycleMilliseconds;
    double CurrPressureSetpointCentimetersH2O;
}BreathCycleState;

typedef struct{
    uint32_t InhaleRampDurationMilliseconds;
    uint32_t InhaleDurationMilliseconds;
    uint32_t ExhaleDurationMilliseconds;
    uint32_t BreathCycleDurationMilliseconds;
    double PeepPressureCentimetersH2O;
    double PipPressureCentimetersH2O;
}BreathCycleSettings;

typedef struct{
    double PatientCircuitPressureCentimetersH2O;
    double OxygenFlowRateLitersPerMinute;
    double TotalFlowRateLitersPerMinute;
}SensorReading;

typedef struct{
    double blowerPowerLevelPercent;
    bool expirationValveState;
}ActuatorState;


#endif
