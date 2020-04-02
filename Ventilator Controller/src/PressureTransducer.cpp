#include "PressureTransducer.h"
PressureTransducer::PressureTransducer()
{}

PressureTransducer::PressureTransducer(TwoWire *bus, uint8_t multiplexerI2cAddress, uint8_t multiplexerChannel, double* readingDestination)
    {
        _multiplexerI2cAddress = multiplexerI2cAddress;
        _sensorMultiplexerChannel = multiplexerChannel;
        _attachedSensor = *(new AllSensors_DLHR_L60D_6(bus));
        _readingDestination = readingDestination;

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
                Serial.print("DHLR Sensor timed out at Address: ");
                Serial.print(_multiplexerI2cAddress);
                Serial.print(" / Channel: ");
                Serial.println(_sensorMultiplexerChannel);
            }
            else
            {
                Serial.print("DHLR Sensor succesfully started at Address: ");
                Serial.print(_multiplexerI2cAddress);
                Serial.print(" / Channel: ");
                Serial.println(_sensorMultiplexerChannel);
            } 
        #endif

        _zeroOutOffsetCentimetersH20 = _attachedSensor.pressure;
        UpdateReading();
    }

void PressureTransducer::UpdateReading()
    {
        if(!_attachedSensor.readData())
        {
            *_readingDestination = _attachedSensor.pressure;
            _attachedSensor.startMeasurement(DHLR_PRESSURE_SENSOR__DEFAULT_READING_TYPE);
        }
    }
