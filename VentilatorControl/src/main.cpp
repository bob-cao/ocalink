// ---------------------------VENTILATOR CONTROL MAIN-------------------------------- //

#include "includes.h"

void setup()
{
  inits();

  breathCycleTimerReset(true);
}

void loop()
{
  pressure_system_input = getPressureReadings();

  cycleStateHandler();

  cycleStateSetpointHandler();

  pinchValveControl();

  writeBlowerSpeed();

  alarmsHandler();

  alarmsVisualAudioHandler();

  computeSerialReceive();

  computeSerialSend();

  // printPidSetpointAndCurrentValues();
}

// ---------------------------VENTILATOR CONTROL MAIN-------------------------------- //
