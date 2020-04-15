#include "PressureTransducer.h"
PressureTransducer::PressureTransducer()
{}

PressureTransducer::PressureTransducer(TwoWire *bus,  I2cMultiplexer *i2cMultiplexer, uint8_t multiplexerChannel, double* readingDestination)
    {
        _attachedMultiplexer = i2cMultiplexer;
        _sensorMultiplexerChannel = multiplexerChannel;
        _attachedSensor = *(new AllSensors_DLHR_L60D_6(bus));
        _readingDestination = readingDestination;

        _attachedMultiplexer->tca_select(_sensorMultiplexerChannel);

        _attachedSensor.setPressureUnit(AllSensors_DLHR::PressureUnit::CM_H2O);
        _attachedSensor.startMeasurement(DHLR_PRESSURE_SENSOR__DEFAULT_READING_TYPE);
        bool sensorWasBusy = true;
        uint32_t sensorPollStartTimeMilliseconds = millis();
        while(sensorWasBusy &&
              ((sensorPollStartTimeMilliseconds-millis()) < SYSTEM_TIMINGS__DLHR_SENSOR_POLL_TIMEOUT__MILLISECONDS  ))
        {
            sensorWasBusy = _attachedSensor.readData();
        }

        #if SYSTEM__SERIAL_DEBUG__PRESSURE_TRANSDUCERS
            if(sensorWasBusy)
            {
                Serial.print("DHLR Sensor timed out at Channel: ");
                Serial.println(_sensorMultiplexerChannel);
            }
            else
            {
                Serial.print("DHLR Sensor succesfully started at Channel: ");
                Serial.println(_sensorMultiplexerChannel);
            } 
        #endif

        _zeroOutOffsetCentimetersH20 = _attachedSensor.pressure;
        _attachedSensor.startMeasurement(DHLR_PRESSURE_SENSOR__DEFAULT_READING_TYPE);
    }

void PressureTransducer::UpdateReading()
    {
        _attachedMultiplexer->tca_select(_sensorMultiplexerChannel);
        if(!_attachedSensor.readData())
        {
            *_readingDestination = _attachedSensor.pressure-_zeroOutOffsetCentimetersH20;
            _attachedSensor.startMeasurement(DHLR_PRESSURE_SENSOR__DEFAULT_READING_TYPE);
        }
    }
