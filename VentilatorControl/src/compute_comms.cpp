#include "includes.h"

void computeSerialSend (void)
{
  // Send values to Raspberry Pi
  // $MAX_PIP,MIN_PEEP,INSTANT_PRESSURE,INSTANT_VOLUME,INSTANT_FLOW*

  static unsigned long computeSerialSendTimer = 0;

  if( millis() - computeSerialSendTimer >= SERIAL_SEND_TIME)
  {
    Serial.println("$10.0,10.0,10.0,10.0,10.0*");
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
    string_from_pi = Serial.readStringUntil('@');
    if(string_from_pi[0] == '%')
    {
      propertyNameAlarms = string_from_pi.substring(string_from_pi.indexOf('%') + 1, string_from_pi.indexOf(','));
      argumentAlarms = string_from_pi.substring(string_from_pi.indexOf(',') + 1, string_from_pi.indexOf('@'));

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
    }

    string_from_pi = Serial.readStringUntil('*');
    if(string_from_pi[0] == '$')
    {
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
      else if(property_name.equalsIgnoreCase("TRISE") && !isnan(argument_value))
      {
        double trise_requested = argument_value / DEFAULT_TRISE_SCALING_FACTOR;  // Multipled by DEFAULT_TRISE_SCALING_FACTOR on Raspberry Pi
        if(trise_requested >= TRISE_MIN_RECEIVE && trise_requested <= TRISE_MAX_RECEIVE)
        {
          InhaleRampDurationMilliseconds = trise_requested * SEC_TO_MS;  // Rise time in ms
          Serial.print("TRISE: ");
          Serial.println(InhaleRampDurationMilliseconds);
        }
        else
        {
          Serial.println("TRISE INPUT OUT OF BOUNDS!");
        }
      }
      else if(property_name.equalsIgnoreCase("RR") && !isnan(argument_value))
      {
        if(argument_value >= RR_MIN_RECEIVE && argument_value <= RR_MAX_RECEIVE)
        {
          RespritoryRate = argument_value;  // Respritory Rate in breathes per minute
          Serial.print("RR: ");
          Serial.println(RespritoryRate);
        }
        else
        {
          Serial.println("RR INPUT OUT OF BOUNDS!");
        }
      }
      else if(property_name.equalsIgnoreCase("IE") && !isnan(argument_value))
      {
        double ie_requested = argument_value / DEFAULT_IE_SCALING_FACTOR;  // Multipled by DEFAULT_IE_SCALING_FACTOR on Raspberry Pi
        if(ie_requested >= IE_MIN_RECEIVE && ie_requested <= IE_MAX_RECEIVE)
        {
          InhalationExhalationRatio = ie_requested;  // Inhalation/Exhalation Ratio
          InhaleDurationMilliseconds = (BREATHS_PER_MINUTE_TO_SEC * SEC_TO_MS) / ((InhalationExhalationRatio + 1.0) * RespritoryRate);
          ExhaleDurationMilliseconds = (BREATHS_PER_MINUTE_TO_SEC * SEC_TO_MS * (1.0 - (1.0 / (InhalationExhalationRatio + 1.0)))) / RespritoryRate;
          Serial.print("IE: ");
          Serial.println(InhalationExhalationRatio);
          Serial.println("InhaleDurationMilliseconds: ");
          Serial.println(InhaleDurationMilliseconds);
          Serial.println("ExhaleDurationMilliseconds: ");
          Serial.println(ExhaleDurationMilliseconds);
        }
        else
        {
          Serial.println("IE INPUT OUT OF BOUNDS!");
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
              Serial.print("TRISE: ");        Serial.print(InhaleRampDurationMilliseconds);                 Serial.println("ms");
              Serial.print("RR: ");           Serial.print(RespritoryRate);                                 Serial.println("b/m");
              Serial.print("IE: ");           Serial.print((1.00 / InhalationExhalationRatio) * RATIO_TO_PERCENTAGE);    Serial.println("%");
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
