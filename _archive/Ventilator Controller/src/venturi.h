#include "AllSensors_DLHR.h"
#include "includes.h"

#ifndef _VENTILATOR_VENTURI_H_
#define _VENTILATOR_VENTURI_H_

// NTP - Normal Temperature and Pressure
// Is defined as 20C (293.15 K, 68F) and 1 atm ( 101.325 kN/m2, 101.325 kPa, 14.7 psia, 0 psig, 30 in Hg, 760 torr)
#define AIR_DENSITY_AT_NTP__KG_PER_CUBIC_METER 1.2051
#define OXYGEN_DENSITY_AT_NTP__KG_PER_CUBIC_METER 1.3311

#define PANTHEON_HALFINCH_VENTURI_DISCHARGE_COEFFICIENT 0.928

#define DEFAULT_VENTURI_PARAMETERS_AIR (Venturi::VenturiParameters AirParameters){13.29,6.645, AIR_DENSITY_AT_NTP__KG_PER_CUBIC_METER, PANTHEON_HALFINCH_VENTURI_DISCHARGE_COEFFICIENT}
#define DEFAULT_VENTURI_PARAMETERS_OXYGEN (Venturi::VenturiParameters OxygenParameters){13.29,6.645, OXYGEN_DENSITY_AT_NTP__KG_PER_CUBIC_METER, PANTHEON_HALFINCH_VENTURI_DISCHARGE_COEFFICIENT}

#define One_Centimeter_H2O_in_Pascals 98.0665
#define One_Meter_Cubed_in_Liters 1000.0

#define square(x) pow(x,2)
#define M_PI 3.14159265358979323846 /* pi */

class Venturi
{
    public:
    typedef struct {
        float inletDiameterMillileters;
        float throatDiameterMillileters;
        float fluidDensityKilogramsPerMeterCubed;
        float dischargeCoefficient;
    }VenturiParameters;


    Venturi();
    Venturi(TwoWire *bus, I2cMultiplexer *i2cMultiplexer, uint8_t multiplexerChannel, VenturiParameters parameters, double* readingDestination);
    void UpdateReading();
    private:
    double calculateFlowRate(double differentialPressureCentimetersH2O);
    AllSensors_DLHR _attachedSensor;
    VenturiParameters _attachedVenturiParameters;
    I2cMultiplexer *_attachedMultiplexer;
    uint8_t _sensorMultiplexerChannel;
    double _zeroOutOffsetCentimetersH20;
    double* _readingDestination;
};

#endif
