// ---------------------------VENTILATOR CONTROL MAIN-------------------------------- //

#include "includes.h"

void setup (void)
{
  // Serial initialization
  Serial.begin(DEFAULT_BAUD_RATE);

  // Initializations
  inits();

  breathCycleTimerReset(true);
}

void loop (void)
{
  pressure_system_input = getPressureReadings();

  cycleStateHandler();

  cycleStateSetpointHandler();

  pinchValveControl();

  writeBlowerSpeed();

  alarmsHandler();

  // alarmsVisualAudioHandler();

  computeSerialReceive();

  // computeSerialSend();

  // debugPrintCurrentValues();
}

// ---------------------------VENTILATOR CONTROL MAIN-------------------------------- //
