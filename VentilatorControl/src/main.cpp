// ---------------------------VENTILATOR CONTROL MAIN-------------------------------- //

#include "includes.h"

void inits (void)
{
  // Initializations
  blowerEscInit();
  alarmLedInit();
  alarmsInit();
  pinchValveInit();
  pressureSensorsInit();
  pidInit();

  // Start cycle state in IDLE state
  CurrCycleStep = IDLE;

  // Serial initialization
  #if SYSTEM__SERIAL_DEBUG__STATEMACHINE
  Serial.begin(DEFAULT_BAUD_RATE);
  #endif
}

// void printPidSetpointAndCurrentValues (void)
// {
//   if( CurrCycleStep != IDLE )
//   {
//     if( millis() % 50 == 0 )
//     {
//       Serial.print(pressure_system_input);
//       Serial.print(" ");
//       switch(CurrCycleStep)
//       {
//         case INHALE_RAMP:
//           Serial.print(CurrPressureSetpointCentimetersH2O);
//         break;
//         case INHALE_HOLD:
//           Serial.print(PipPressureCentimetersH2O);
//         break;
//         case EXHALE_RAMP:
//         case EXHALE_HOLD:
//         case IDLE:
//         default:
//           Serial.print(PeepPressureCentimetersH2O);
//         break;
//       }
//       Serial.print(" ");
//       Serial.print(blower_output_speed_in_percentage);
//       Serial.print(" ");
//       Serial.print(venturiDifferentialPressureReading);
//       Serial.println();
//     }
//   }
// }

void cycleStateHandler (void)
{
  if( CurrCycleStep != IDLE )
  {
    CurrTimeInCycleMilliseconds = millis()-CycleStartTimeFromSysClockMilliseconds;
    if(millis()-ControlLoopStartTimeMilliseconds > ControlLoopInitialStabilizationTimeMilliseconds)
    {
      if(CurrTimeInCycleMilliseconds <= InhaleRampDurationMilliseconds)
      {
        CurrCycleStep = INHALE_RAMP;
      }
      else if((InhaleRampDurationMilliseconds < CurrTimeInCycleMilliseconds) &&
              (CurrTimeInCycleMilliseconds <= InhaleDurationMilliseconds))
      { 
        CurrCycleStep = INHALE_HOLD;
      }
      else if((InhaleDurationMilliseconds < CurrTimeInCycleMilliseconds) &&
            (CurrTimeInCycleMilliseconds <= BreathCycleDurationMilliseconds))
      {
        if( pressure_system_input > (PeepPressureCentimetersH2O + 5) )
        {
          CurrCycleStep = EXHALE_RAMP;
        }
        else
        {
          CurrCycleStep = EXHALE_HOLD;
        }
      }
      else if(CurrTimeInCycleMilliseconds > BreathCycleDurationMilliseconds)
      {
        CurrCycleStep = INHALE_RAMP;
        breath_cycle_timer_reset();
      }
    }
    else // if idle == true
    {
      CurrCycleStep = EXHALE_HOLD;
    }
  }
}

// Compensation for DC offset in pressure achieved.
// Found by linear regression of the following experimental datapoints:
// Target Val	Setpoint Req'd
// 15       	16
// 20	        21
// 25	        27.9
// 30	        33.5
// 35	        38.5
// 40	        44.5
// 45	        49.25
// 50	        55

double linearRemapSetpointCompensation (double setpoint)
{
  return (1.116785714*setpoint)-0.5892857143;
}

void cycleStateSetpointHandler(void)
{
  // TODO: HIGH breakout into separate file and rewrite for readability
  // TODO: MEDIUM put gains in a header, not in the source

  //Recompute Setpoints
  switch(CurrCycleStep)
  {
    case INHALE_RAMP:
      CurrPressureSetpointCentimetersH2O = (((float)CurrTimeInCycleMilliseconds/(float)InhaleRampDurationMilliseconds)*(PipPressureCentimetersH2O-PeepPressureCentimetersH2O))+PeepPressureCentimetersH2O;
    break;
    case INHALE_HOLD:
      CurrPressureSetpointCentimetersH2O = PipPressureCentimetersH2O;
    break;
    case EXHALE_RAMP:
    case EXHALE_HOLD:
    case IDLE:
    default:
      CurrPressureSetpointCentimetersH2O = PeepPressureCentimetersH2O;
    break;
  }

  CurrPressureSetpointCentimetersH2O = linearRemapSetpointCompensation(CurrPressureSetpointCentimetersH2O);
}

void buzzerToggle (void)
{
  static unsigned long lastBuzzerToggle = 0;
  if( millis()-lastBuzzerToggle > 500 )
  {
    digitalWrite(BUZZER_PIN, buzzer_state);
    buzzer_state = !buzzer_state;
    lastBuzzerToggle = millis();
  }
}

// ALARM CODES
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
  static unsigned long PrevAlarmTimePipError = 0;
  static unsigned long PrevAlarmTimePeepError = 0;
  static unsigned long PrevAlarmTimeDisconnectError = 0;

  // Ventilator specific alarms (battery backup activated)
  isBatteryActivated = digitalRead(BATTERY_ALARM_PIN);
  if(isBatteryActivated)
  {
    alarm_state = 1;
    Serial.write("$ALARMS,A*");  // BATTERY BACKUP ALARM
  }
  // Disconnect Alarm
  if((millis()-PrevAlarmTimeDisconnectError > DisconnectAlarmTimer)
      && (CurrCycleStep == EXHALE_HOLD && CurrCycleStep != INHALE_RAMP)
      && (pressure_system_input >= -peep_low_alarm
      && pressure_system_input <= peep_low_alarm))
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

void alarmsVisualAudioHandler (void)
{
  switch (alarm_state)
  {
    case 1:
      chase(red, low_red, LED_ON_TIME); // red
      digitalWrite(BUZZER_PIN, HIGH);
      break;
    case 2:
      chase(amber, low_amber, LED_ON_TIME); // amber
      buzzerToggle();
      break;
    case 3:
    default:
      chase(green, low_green, LED_ON_TIME); // green
      digitalWrite(BUZZER_PIN, LOW);
      break;
  }
}

void computeSerialSend (void)
{
  // Send values to Raspberry Pi
  // Serial.println();  // Populate with values once that is figured out
}

void setup()
{
  inits();

  breath_cycle_timer_reset(true);
}

void loop()
{
  pressure_system_input = getPressureReadings();

  cycleStateHandler();

  cycleStateSetpointHandler();

  pinchValveControl();

  writeBlowerSpeed();

  // printPidSetpointAndCurrentValues();

  alarmsHandler();

  alarmsVisualAudioHandler();

  computeSerialReceive();

  computeSerialSend();
}
