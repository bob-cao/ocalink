#include "includes.h"

double getPressureReadings (void)
{
  I2CMux.openChannel(PATIENT_CIRCUIT_PRESSURE_MUX_CHANNEL); 
  if(!patientCircuitPressure.readData(false))
  {
    // Get a pressure reading and convert to units of cmH2O
    if( patientCircuitPressure.pressure > 0 )
      pressure_reading = patientCircuitPressure.pressure * INCHES_2_CM;
    patientCircuitPressure.startMeasurement();
  }
  I2CMux.closeChannel(PATIENT_CIRCUIT_PRESSURE_MUX_CHANNEL);

  I2CMux.openChannel(VENTURI_DIFFERENTIAL_PRESSURE_MUX_CHANNEL);
  if(!venturiDifferentialPressure.readData(false))
  {
    // Get a pressure reading and convert to units of cmH2O
    if( venturiDifferentialPressure.pressure > 0 )
    {
      venturiDifferentialPressureReading = venturiDifferentialPressure.pressure * INCHES_2_CM;
      // Transfer function found experimentally through bench testing
      // Flow [lpm] = f(pressure [cmH20])
      // see here for data:
      // https://docs.google.com/spreadsheets/d/1bI_WWhnsqxKvRou-n7BQC0ML1uTgSl8-lZcnprczsNk/edit?usp=sharing
      venturiFlowRateLpm = 26.486*sqrt(venturiDifferentialPressureReading)-0.276;
      
      // lowest reported flow rate = 0lpm
      venturiFlowRateLpm = venturiFlowRateLpm>0?venturiFlowRateLpm:0;
    }
    venturiDifferentialPressure.startMeasurement();
  }
  I2CMux.closeChannel(VENTURI_DIFFERENTIAL_PRESSURE_MUX_CHANNEL);
  return pressure_reading;
}
