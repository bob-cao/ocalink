// ---------------------------VENTILATOR CONTROL MAIN-------------------------------- //

#include "includes.h"

void setup (void)
{
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

  alarmsVisualAudioHandler();

  computeSerialReceive();

  computeSerialSend();

  // printPidSetpointAndCurrentValues();
}

// ---------------------------VENTILATOR CONTROL MAIN-------------------------------- //
