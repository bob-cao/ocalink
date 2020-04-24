#include "includes.h"

// ALARM CODES
// %<property>,<value>@
// %ALARMS,A@
// A	Battery Backup Activated Alarm
// B	Disconnect Alarm
// C	High PIP Alarm
// D	Low PIP Alarm
// E	High PEEP Alarm
// F	Low PEEP Alarm
// G	Apnea Alarm
// H	High/Low RR Alarm
// I	I:E Ratio Alarm

void alarmsHandler (void)
{
  static unsigned long alarmTimer = 0;

  static unsigned long PrevAlarmTimePipError = 0;
  static unsigned long PrevAlarmTimePeepError = 0;
  static unsigned long PrevAlarmTimeDisconnectError = 0;

  if( millis() - alarmTimer >= ALARM_TIME)
  {
    alarmTimer = millis();

    // Ventilator specific alarms (battery backup activated)
    // isBatteryActivated = digitalRead(BATTERY_ALARM_PIN)?false:true;
    // if(isBatteryActivated && !isBatterySnoozeTriggered)
    // {
    //   alarm_state = 1;
    //   Serial.write("$ALARMS,A*");  // BATTERY BACKUP ALARM
    // }
    // Disconnect Alarm
    if(((millis()-PrevAlarmTimeDisconnectError > DisconnectAlarmTimer)
        && (CurrCycleStep == EXHALE_HOLD && CurrCycleStep != INHALE_RAMP)
        && (pressure_system_input >= -peep_low_alarm
        && pressure_system_input <= peep_low_alarm)) && !isDisconnectSnoozeTriggered)
    {
      // make sound and send Raspberry Pi alarm status flag
      alarm_state = 1;
      Serial.write("$ALARMS,B*");  // DISCONNECT ALARM
      PrevAlarmTimeDisconnectError = millis();
    }
    // High and Low PIP Alarms
    if((millis()-PrevAlarmTimePipError > PipAlarmTimer)
        && (CurrCycleStep == INHALE_HOLD && CurrCycleStep != EXHALE_RAMP && CurrCycleStep != INHALE_RAMP)
        && (pressure_system_input <= PipPressureCentimetersH2O - pip_alarm
        || pressure_system_input >= PipPressureCentimetersH2O + pip_alarm))
    {
      // make sound and send Raspberry Pi alarm status flag
      if(alarm_state<2)
        alarm_state = 2;
      if(pressure_system_input >= PipPressureCentimetersH2O + pip_alarm)
      {
        Serial.write("$ALARMS,C*");  // HIGH PIP ALARM
        // exhale immedietly to PEEP pressure and continue breathing cycle, don't reset alarm
        CurrCycleStep = EXHALE_RAMP;
      }
      else if(pressure_system_input <= PipPressureCentimetersH2O - pip_alarm)
      {
        Serial.write("$ALARMS,D*");  // LOW PIP ALARM
      }

      PrevAlarmTimePipError = millis();
    }
    // High and Low PEEP Alarms
    if((millis()-PrevAlarmTimePeepError > PeepAlarmTimer)
        && (CurrCycleStep == INHALE_HOLD && CurrCycleStep != EXHALE_RAMP && CurrCycleStep != INHALE_RAMP)
        && (pressure_system_input <= PeepPressureCentimetersH2O - peep_alarm
        || pressure_system_input >= PeepPressureCentimetersH2O + peep_alarm))
    {
      // make sound and send Raspberry Pi alarm status flag
      if(alarm_state<2)
        alarm_state = 2;

      if(pressure_system_input >= PeepPressureCentimetersH2O + peep_alarm)
      {
        Serial.write("$ALARMS,E*");  // HIGH PEEP ALARM
      }
      else if(pressure_system_input <= PeepPressureCentimetersH2O - peep_alarm)
      {
        Serial.write("$ALARMS,F*");  // LOW PEEP ALARM
      }

      PrevAlarmTimePeepError = millis();
    }
    else
    {
      alarm_state = 3;
    }

    // TODO: Add Apnea Alarm
    // error caused by no spontaneous breath for x amount of time
    // Time is user set, still deciding on venturi flow or intake pressure as trigger
    // if(millis()-PrevAlarmTimeApneaError > ApneaTimer)
    // {
    //   PrevAlarmTimeApneaError = millis();
    //   buzzerToggle();
    //   Serial.write("$ALARMS,G*");  // APNEA ALARM
    // }

    // TODO: Add High/Low RR Alarm
    // if(millis()-PrevAlarmTimeRRError > RRTimer)
    // {
    //   PrevAlarmTimeRRError = millis();
    //   buzzerToggle();
    //   Serial.write("$ALARMS,H*");  // High/Low RR ALARM
    // }

    // TODO: Add I:E ratio Alarm
    // if(millis()-PrevAlarmTimeIEError > IETimer)
    // {
    //   PrevAlarmTimeIEError = millis();
    //   buzzerToggle();
    //   Serial.write("$ALARMS,I*");  // I:E Ratio ALARM
    // }
  }
}

void alarmsVisualAudioHandler (void)
{
  switch (alarm_state)
  {
    case 1:
      // chase(red, low_red, LED_ON_TIME); // red
      digitalWrite(ALARM_STATE_1_PIN, HIGH);
      digitalWrite(ALARM_STATE_2_PIN, LOW);
      digitalWrite(ALARM_STATE_3_PIN, LOW);
      if(!isBatterySnoozeTriggered)
      {
        digitalWrite(BUZZER_PIN, HIGH);
      }
      break;
    case 2:
      // chase(amber, low_amber, LED_ON_TIME); // amber
      digitalWrite(ALARM_STATE_2_PIN, HIGH);
      digitalWrite(ALARM_STATE_1_PIN, LOW);
      digitalWrite(ALARM_STATE_3_PIN, LOW);
      if(!isBatterySnoozeTriggered)
      {
        //buzzerToggle();
        digitalWrite(BUZZER_PIN, HIGH);
      }
      break;
    case 3:
    default:
      //chase(green, low_green, LED_ON_TIME); // green
      digitalWrite(ALARM_STATE_3_PIN, HIGH);
      digitalWrite(ALARM_STATE_2_PIN, LOW);
      digitalWrite(ALARM_STATE_1_PIN, LOW);
      digitalWrite(BUZZER_PIN, LOW);
      break;
  }
}
