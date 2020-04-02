
#ifndef _VENTILATOR_PRESSURE_TRANSDUCER_H_
#define _VENTILATOR_PRESSURE_TRANSDUCER_H_

#include "AllSensors_DLHR.h"
#include "includes.h"

class PressureTransducer
{
    public:
    PressureTransducer();
    PressureTransducer(TwoWire *bus, uint8_t multiplexerI2cAddress, uint8_t multiplexerChannel, double* readingDestination);
    void UpdateReading();
    private:
    AllSensors_DLHR _attachedSensor;
    uint8_t _sensorMultiplexerChannel;
    uint8_t _multiplexerI2cAddress;
    double _zeroOutOffsetCentimetersH20;
    double* _readingDestination;
};

#endif
