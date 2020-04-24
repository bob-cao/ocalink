#include "includes.h"

void blowerEscInit (void)
{
  analogWriteResolution(10);
  // DO NOT need to use pinMode() for using DAC on this plaform
  analogWrite(BLOWER_SPEED_PIN, 0);
}

void alarmLedInit (void)
{
  //AlarmLED.begin();
}

void alarmsInit (void)
{
  alarmLedInit();

  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);  // Buzzer is a piezo with a built in driver

  pinMode(BATTERY_ALARM_PIN, INPUT);  // Battery UPS backup has a NC relay
  pinMode(BATTERY_SHUTDOWN_PIN, INPUT);
}

void pinchValveInit (void)
{
  pinMode(PINCH_VALVE_PIN, OUTPUT);
  PinchValve.attach(PINCH_VALVE_PIN);
  PinchValve.writeMicroseconds(PINCH_VALVE_DRIVER_FULL_OPEN_PULSE_MICROSECONDS);
}

void pressureSensorsInit (void)
{
  // Init the I2C bus
  // Each pressure sensor will be unique I2C addresses based on MPN in future
  I2CMux.begin(Wire);
  I2CMux.closeAll();

  //Setup the Patient circuit pressure sensor
  I2CMux.openChannel(PATIENT_CIRCUIT_PRESSURE_MUX_CHANNEL);
  patientCircuitPressure.setPressureUnit(AllSensors_DLHR::PressureUnit::IN_H2O);
  patientCircuitPressure.startMeasurement();
  I2CMux.closeChannel(PATIENT_CIRCUIT_PRESSURE_MUX_CHANNEL);

  // Setup the Venturi differential pressure sensor
  I2CMux.openChannel(VENTURI_DIFFERENTIAL_PRESSURE_MUX_CHANNEL);
  venturiDifferentialPressure.setPressureUnit(AllSensors_DLHR::PressureUnit::IN_H2O);
  patientCircuitPressure.startMeasurement();
  I2CMux.closeChannel(VENTURI_DIFFERENTIAL_PRESSURE_MUX_CHANNEL);

  // Setup rate limiting for reads to the pressure sensors
  PressureSensorLastStatusRead = micros();
}

void pidInit (void)
{
  Blower_PID.SetMode(AUTOMATIC);      // Set PID Mode to Automatic, may change later
  Blower_PID.SetOutputLimits(-MAX_BLOWER_PRESSURE_OFFSET, MAX_BLOWER_PRESSURE_OFFSET);
  Blower_PID.SetSampleTime(DEFAULT_PID_SAMPLE_TIME);

  PinchValve_PID.SetMode(AUTOMATIC);  // Set PID Mode to Automatic, may change later
  PinchValve_PID.SetOutputLimits(MIN_PERCENTAGE, MAX_PERCENTAGE);
  PinchValve_PID.SetSampleTime(DEFAULT_PID_SAMPLE_TIME);
}

void inits (void)
{
  blowerEscInit();
  alarmLedInit();
  alarmsInit();
  pinchValveInit();
  pressureSensorsInit();
  pidInit();

  // Start cycle state in IDLE state
  CurrCycleStep = IDLE;
}
