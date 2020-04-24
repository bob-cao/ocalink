#include "includes.h"

void computeSerialSend (void)
{
  // Send values to Raspberry Pi
  // $MAX_PIP,MIN_PEEP,INSTANT_PRESSURE,INSTANT_VOLUME,INSTANT_FLOW*

  static unsigned long computeSerialSendTimer = 0;
  returnHighestPipOverPeriod();
  returnLowestPeepOverPeriod();
  returnInstantPressure();
  returnInspiratoryVolume();
  
  
  if( (millis() - computeSerialSendTimer) >= SERIAL_SEND_TIME)
  {
    isHighLowPressureDoneOneCycle = false;
    Serial.print("$");
    Serial.print(String(maxPipPressure, 2));
    Serial.print(",");
    Serial.print(String(minPeepPressure, 2));
    Serial.print(",");
    Serial.print(String(instantPressure, 2));
    Serial.print(",");
    Serial.print(String(inspiratoryVolume, 2));  //TODO
    Serial.print(",");
    Serial.print(String(venturiFlowRateLpm, 2));  //TODO
    Serial.println("*");

    computeSerialSendTimer = millis();
  }
}

void computeSerialReceive (void)
{
  // TODO: HIGH Break out serial protocol into seperate file
  // TODO: HIGH ensure serial operations do not dirupt control loop
  // TODO: Ensure that all fields are populated before going into running test?

  // $<property_name>,<argument_value><LF>

  if (Serial.available())
  {
    string_from_pi = Serial.readStringUntil('*');
    if(string_from_pi[0] == '$')
    {
      propertyNameAlarms = string_from_pi.substring(string_from_pi.indexOf('$') + 1, string_from_pi.indexOf(','));
      argumentAlarms = string_from_pi.substring(string_from_pi.indexOf(',') + 1, string_from_pi.indexOf('*'));

      if( propertyNameAlarms.equalsIgnoreCase("ALARMS") )
      {
        if(argumentAlarms == "A")
        {
          isBatterySnoozeTriggered = true;
        }
        if(argumentAlarms == "B")
        {
          isDisconnectSnoozeTriggered = true;
        }
        else
        {
          Serial.println("ALARM STATE INPUT OUT OF BOUNDS!");  // Alarm out of bounds error
        }
      }
      property_name = string_from_pi.substring(string_from_pi.indexOf('$') + 1, string_from_pi.indexOf(','));
      argument = string_from_pi.substring(string_from_pi.indexOf(',') + 1, string_from_pi.indexOf('*'));

      if(isDigit(argument[0]))
      {
        argument_value = argument.toFloat();
      }
      else
      {
        argument_value = NAN;
      }
      if(property_name.equalsIgnoreCase("PIP") && !isnan(argument_value))
      {
        if(argument_value >= PIP_MIN_RECEIVE && argument_value <= PIP_MAX_RECEIVE)
        {
          PipPressureCentimetersH2O = argument_value;  // PIP Value
          Serial.print("PIP: ");
          Serial.println(PipPressureCentimetersH2O);
        }
        else
        {
          Serial.println("PIP INPUT OUT OF BOUNDS!");
        }
      }
      else if(property_name.equalsIgnoreCase("PEEP") && !isnan(argument_value))
      {
        if(argument_value >= PEEP_MIN_RECEIVE && argument_value <= PEEP_MAX_RECEIVE)
        {
          PeepPressureCentimetersH2O = argument_value;  // PEEP Value
          Serial.print("PEEP: ");
          Serial.println(PeepPressureCentimetersH2O);
        }
        else
        {
          Serial.println("PEEP INPUT OUT OF BOUNDS!");
        }
      }
      // else if(property_name.equalsIgnoreCase("FIO2") && !isnan(argument_value))
      // {
      //   double fio2_requested = argument_value / DEFAULT_FIO2_SCALING_FACTOR;  // Multipled by DEFAULT_FIO2_SCALING_FACTOR on Raspberry Pi
      //   if(fio2_requested >= FIO2_MIN_RECEIVE && fio2_requested <= FIO2_MAX_RECEIVE)
      //   {
      //     FlowOfOxygen = fio2_requested;  // Flow of O2 in %
      //     Serial.print("FIO2: ");
      //     Serial.println(FlowOfOxygen);
      //   }
      // }
      else if(property_name.equalsIgnoreCase("TI") && !isnan(argument_value))
      {
        timeInspiratoryRequested = argument_value / DEFAULT_INSP_SCALING_FACTOR;
        if(timeInspiratoryRequested >= INSP_MIN_RECEIVE && timeInspiratoryRequested <= INSP_MAX_RECEIVE)
        {
          InhaleDurationMilliseconds = timeInspiratoryRequested * SEC_TO_MS;  // Rise time in ms
          Serial.print("InhaleDuration (TI): ");
          Serial.println(InhaleDurationMilliseconds);
        }
        else
        {
          Serial.println("TI INPUT OUT OF BOUNDS!");
        }
      }
      else if(property_name.equalsIgnoreCase("RR") && !isnan(argument_value))
      {
        if(argument_value >= RR_MIN_RECEIVE && argument_value <= RR_MAX_RECEIVE)
        {
          BreathCycleDurationMilliseconds = argument_value / BREATHS_PER_MINUTE_TO_SEC;
          ExhaleDurationMilliseconds = BreathCycleDurationMilliseconds - InhaleDurationMilliseconds;
          Serial.print("BreathCycleDuration: ");
          Serial.print(BreathCycleDurationMilliseconds);
          Serial.print("ExhaleDuration: ");
          Serial.print(ExhaleDurationMilliseconds);
        }
        else
        {
          Serial.println("RR INPUT OUT OF BOUNDS!");
        }
      }
      else if( property_name.equalsIgnoreCase("CMD") )
      {
        if(argument.equalsIgnoreCase("START") || argument.equalsIgnoreCase("STOP"))
        {
          if(argument.equalsIgnoreCase("STOP"))
          {
            Serial.println("TEST STOPPED, GOING INTO IDLE");  // Test was requested to stop, go to IDLE
            CurrCycleStep = IDLE;
          }
          else if(argument.equalsIgnoreCase("START"))
          {
            if(CurrCycleStep == IDLE)  // To start test, cycle state needs to be in IDLE
            {
              CurrCycleStep = EXHALE_HOLD;
              breathCycleTimerReset(true);
              Serial.println("TEST STARTED");
              Serial.print("PEEP: ");         Serial.print(PeepPressureCentimetersH2O);                     Serial.println("cmH20");
              Serial.print("PIP: ");          Serial.print(PipPressureCentimetersH2O);                      Serial.println("cmH20");
              // Serial.print("FIO2: ");         Serial.print(FlowOfOxygen);                                   Serial.println("cmH20");
            }
          }
          else
          {
            Serial.println("CMD CODE INPUT OUT OF BOUNDS!");  // Command code is out of bounds
          }
        }
      }
      else
      {
        Serial.println("UNKNOWN MESSAGE");  // A message starting with '$' was not valid
      }
    }
  }
}
