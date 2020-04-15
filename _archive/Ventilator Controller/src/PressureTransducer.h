
#ifndef _VENTILATOR_PRESSURE_TRANSDUCER_H_
#define _VENTILATOR_PRESSURE_TRANSDUCER_H_

#include "AllSensors_DLHR.h"
#include "includes.h"

class PressureTransducer
{
    public:
    PressureTransducer();
    PressureTransducer(TwoWire *bus, I2cMultiplexer *i2cMultiplexer, uint8_t multiplexerChannel, double* readingDestination);
    void UpdateReading();
    private:
    AllSensors_DLHR _attachedSensor;
    I2cMultiplexer *_attachedMultiplexer;
    uint8_t _sensorMultiplexerChannel;
    double _zeroOutOffsetCentimetersH20;
    double* _readingDestination;
};

#endif
