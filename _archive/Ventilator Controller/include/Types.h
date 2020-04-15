// Define types used across the controller
#ifndef _VENTILATOR_TYPES_H_
#define _VENTILATOR_TYPES_H_

#include <cstdint>

// Structs and enums for general control loop operation
#pragma region
typedef enum{
    INHALE_RAMP,
    INHALE_HOLD,
    EXHALE
}BreathCycleStep;

#define NUM_BREATHCYCLE_STEPS (BreathCycleStep::EXHALE+1)

#if SYSTEM__SERIAL_DEBUG_ENABLED
const char* BreathCycleStepNames[NUM_BREATHCYCLE_STEPS]
{
    "INHALE_RAMP",
    "INHALE_HOLD",
    "EXHALE"
}
#endif

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
    double BlowerOutletPressureCentimetersH20;
    double OxygenVenturiDifferentialPressureCentimetersH20;
    double TotalFlowVenturiDifferentialPressureCentimetersH20;
    double OxygenFlowRateLitersPerMinute;
    double TotalFlowRateLitersPerMinute;
}SensorReading;

typedef struct{
    double blowerPowerLevelPercent;
    bool expirationValveState;
}ActuatorState;

#pragma endregion

// Structs and enums for I2C device management
typedef enum{
    DLHR_OXYGEN_VENTURI = 0,
    DLHR_TOTAL_VENTURI,
    DLHR_BLOWER_OUTLET_GAGE,
    DLHR_PATIENT_CIRCUIT_GAGE
}I2cMultiplexerChannels;

#define NUM_I2C_MULTIPLEXER_CHANNELS (I2cMultiplexerChannels::DLHR_PATIENT_CIRCUIT_GAGE + 1)

#endif
