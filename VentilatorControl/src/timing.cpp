#include "includes.h"

void breathCycleTimerReset (bool hardreset)
{
  CurrTimeInCycleMilliseconds = 0;
  CycleStartTimeFromSysClockMilliseconds = millis();

  isHighLowPressureDoneOneCycle = true;

  if(hardreset)
  {
    ControlLoopStartTimeMilliseconds = CycleStartTimeFromSysClockMilliseconds;
  }
}

void alarm2MinuteSnoozeTimer (void)
{
  static unsigned long alarmSnoozeTimer = 0;

  if((millis() - alarmSnoozeTimer >= SNOOZE_TIME)
    && (isBatterySnoozeTriggered || isDisconnectSnoozeTriggered))
  {
    isBatterySnoozeTriggered = false;
    isDisconnectSnoozeTriggered = false;

    alarmSnoozeTimer = millis();
  }
}
