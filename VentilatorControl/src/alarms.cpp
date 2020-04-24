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

  static bool disconnectalarmtimer_running = false;
  static unsigned long disconnectalarmtimer_starttime;
  
  static bool pipovershoottimer_running = false;
  static unsigned long pipovershoottimer_starttime;

  static bool lowpiptimer_running = false;
  static unsigned long lowpiptimer_starttime;

  static bool highpeeptimer_running = false;
  static unsigned long highpeeptimer_starttime;

  static bool peepundershoottimer_running = false;
  static unsigned long peepundershoottimer_starttime;




      // Disconnect Alarm
      if((pressure_reading<2) && !disconnectalarmtimer_running)
      {
        disconnectalarmtimer_starttime = millis();
        disconnectalarmtimer_running = true;
      }
      if((pressure_reading>2) && disconnectalarmtimer_running)
      {
        disconnectalarmtimer_running = false;
      }
      // High PIP Alarm
      if((pressure_reading>(PipPressureCentimetersH2O+pip_alarm_overshoot)) && !pipovershoottimer_running)
      {
        pipovershoottimer_starttime = millis();
        pipovershoottimer_running = true;
      }
      if((pressure_reading<(PipPressureCentimetersH2O+pip_alarm_overshoot)) && pipovershoottimer_running)
      {
        pipovershoottimer_running = false;
      }

      // Low PIP Alarm
      if((pressure_reading<(PipPressureCentimetersH2O-pip_alarm_overshoot)) && !lowpiptimer_running)
      {
        lowpiptimer_starttime = millis();
        lowpiptimer_running = true;
      }
      if((pressure_reading>(PipPressureCentimetersH2O-pip_alarm_overshoot)) && lowpiptimer_running)
      {
        lowpiptimer_running = false;
      }

      // High PEEP Alarm
      if((pressure_reading>(PeepPressureCentimetersH2O+peep_alarm_undershoot)) && !highpeeptimer_running)
      {
        highpeeptimer_starttime = millis();
        highpeeptimer_running = true;
      }
      if((pressure_reading<(PeepPressureCentimetersH2O+peep_alarm_undershoot)) && highpeeptimer_running)
      {
        highpeeptimer_running = false;
      }

      // Low PEEP alarm
      if((pressure_reading<(PeepPressureCentimetersH2O-peep_alarm_undershoot)) && !peepundershoottimer_running)
      {
        peepundershoottimer_starttime = millis();
        peepundershoottimer_running = true;
      }
      if((pressure_reading>(PeepPressureCentimetersH2O-peep_alarm_undershoot)) && peepundershoottimer_running)
      {
        peepundershoottimer_running = false;
      }


      

      if( millis() - alarmTimer >= ALARM_TIME)
      {
        // Battery backup alarm
//      Serial.write("$ALARMS,A*");  // BATTERY BACKUP ALARM

        // disconnect alarm
        if(disconnectalarmtimer_running && (millis()-disconnectalarmtimer_starttime)>DisconnectAlarmTimer)
        {
          alarm_state = 1;
          Serial.write("$ALARMS,B*");  // DISCONNECT ALARM
        }
        // PIP overshoot alarm
        else if(pipovershoottimer_running && (millis()-pipovershoottimer_starttime)>PipAlarmTimer)
        {
          alarm_state = 2;
          Serial.write("$ALARMS,C*");  // HIGH PIP ALARM
        }
        // Low PIP alarm
        else if(lowpiptimer_running && (millis()-lowpiptimer_starttime)>PipAlarmTimer)
        {
          alarm_state = 2;
          Serial.write("$ALARMS,D*");  // LOW PIP ALARM
        }
        // High Peep alarm
        else if(highpeeptimer_running && (millis()-highpeeptimer_starttime)>PeepAlarmTimer)
        {
          alarm_state = 2;
          Serial.write("$ALARMS,E*");  // HIGH PEEP ALARM
        }
        // Peep Undershoot alarm
        else if(peepundershoottimer_running && (millis()-peepundershoottimer_starttime)>PeepAlarmTimer)
        {
          alarm_state = 2;
          Serial.write("$ALARMS,F*");  // LOW PEEP ALARM
        }
        // DISCONNECT Alarm

        alarmTimer = millis();
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
