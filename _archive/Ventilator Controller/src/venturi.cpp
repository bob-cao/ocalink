#include "venturi.h"
#include <cmath>

Venturi::Venturi()
{}

Venturi::Venturi(TwoWire *bus, I2cMultiplexer *i2cMultiplexer, uint8_t multiplexerChannel, VenturiParameters parameters, double* readingDestination)
    {
        _attachedMultiplexer = i2cMultiplexer;
        _sensorMultiplexerChannel = multiplexerChannel;
        _attachedSensor = *(new AllSensors_DLHR_L60D_6(bus));
        _attachedVenturiParameters = parameters;
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

void Venturi::UpdateReading()
    {
        _attachedMultiplexer->tca_select(_sensorMultiplexerChannel);
        if(!_attachedSensor.readData())
        {
            *_readingDestination = calculateFlowRate(_attachedSensor.pressure-_zeroOutOffsetCentimetersH20);
            _attachedSensor.startMeasurement(DHLR_PRESSURE_SENSOR__DEFAULT_READING_TYPE);
        }
    }

double Venturi::calculateFlowRate(double differentialPressureCentimetersH2O)
{
    // Converting Differential pressure into pascals for unit consistency
    double differentialPressurePascals = differentialPressureCentimetersH2O * One_Centimeter_H2O_in_Pascals;

    // Area ratio of 2 circles is equal to the ratio of the diameters squared
    // Area ratio ends up being unitless so no issues there.
    double areaRatioSquared = square(square(_attachedVenturiParameters.inletDiameterMillileters)/square(_attachedVenturiParameters.throatDiameterMillileters));

    // Outlet velocity Squared in m^2/s^s
    double outletVelocitySquared = (2*differentialPressurePascals)/(_attachedVenturiParameters.fluidDensityKilogramsPerMeterCubed*(areaRatioSquared-1));

    // Outlet velocity in m/s
    double outletVelocityMetersPerSecond = sqrt(outletVelocitySquared);

    // Outlet flow rate in m^3/s
    double outletFlowRateMetersCubedPerSecond = outletVelocityMetersPerSecond * M_PI * square(_attachedVenturiParameters.inletDiameterMillileters/1000) / 4;

    double outletFlowRateLitersPerSecond = outletFlowRateMetersCubedPerSecond * One_Meter_Cubed_in_Liters;

    return outletFlowRateLitersPerSecond;
}