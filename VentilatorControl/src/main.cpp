// ------------------------------VENTILATOR CONTROL---------------------------------- //

#include "includes.h"

void blower_esc_init (void)
{
  analogWriteResolution(10);
  // DO NOT need to use pinMode() for using DAC on this plaform
  analogWrite(BLOWER_SPEED_PIN,0);
}

void AlarmLEDInit (void)
{
  AlarmLED.begin();
}

static void chase(uint32_t primary, uint32_t secondary, int cycleDelay)
{
  for(uint16_t i = 0; i < AlarmLED.numPixels() + 4; i++)
  {
    AlarmLED.setPixelColor(i  , primary);     // Draw new pixel
    AlarmLED.setPixelColor(i - 8, secondary); // Erase pixel a few steps back
    AlarmLED.show();
    delay(cycleDelay);
  }
}

void alarms_init (void)
{
  AlarmLEDInit();

  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);  // Buzzer is a piezo with a built in driver

  pinMode(BATTERY_ALARM_PIN, INPUT);  // Battery UPS backup has a NC relay
}

void pinch_valve_init (void)
{
  pinMode(PINCH_VALVE_PIN, OUTPUT);
  pinch_valve.attach(PINCH_VALVE_PIN);
  pinch_valve.writeMicroseconds(PINCH_VALVE_DRIVER_FULL_OPEN_PULSE_MICROSECONDS);
}

void pressure_sensors_init (void)
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
  I2CMux.openChannel(VENTRUI_DIFFERENTIAL_PRESSURE_MUX_CHANNEL);
  venturiDifferentialPressure.setPressureUnit(AllSensors_DLHR::PressureUnit::IN_H2O);
  patientCircuitPressure.startMeasurement();
  I2CMux.closeChannel(VENTRUI_DIFFERENTIAL_PRESSURE_MUX_CHANNEL);

  // Setup rate limiting for reads to the pressure sensors
  PressureSensorLastStatusRead = micros();
}

void pid_init (void)
{
  Blower_PID.SetMode(AUTOMATIC);      // Set PID Mode to Automatic, may change later
  Blower_PID.SetOutputLimits(MIN_PERCENTAGE, MAX_PERCENTAGE);
  Blower_PID.SetSampleTime(DEFAULT_PID_SAMPLE_TIME);

  PinchValve_PID.SetMode(AUTOMATIC);  // Set PID Mode to Automatic, may change later
  PinchValve_PID.SetOutputLimits(MIN_PERCENTAGE, MAX_PERCENTAGE);
  PinchValve_PID.SetSampleTime(DEFAULT_PID_SAMPLE_TIME);
}

void inits (void)
{
  // Initializations
  blower_esc_init();
  alarms_init();
  pinch_valve_init();
  pressure_sensors_init();
  pid_init();

  // Start cycle state in IDLE state
  CurrCycleStep = IDLE;

  // Serial initialization
  #if SYSTEM__SERIAL_DEBUG__STATEMACHINE
  Serial.begin(DEFAULT_BAUD_RATE);
  #endif
}

double get_pressure_reading (void)
{
  if(micros() - PressureSensorLastStatusRead >= 1000)
  {
    I2CMux.openChannel(PATIENT_CIRCUIT_PRESSURE_MUX_CHANNEL);
    uint8_t sensorStatus = patientCircuitPressure.readStatus();
    if( !patientCircuitPressure.isBusy(sensorStatus) && !patientCircuitPressure.isError(sensorStatus) )
    {  
      if(!patientCircuitPressure.readData(false))
      {
        // Get a pressure reading and convert to units of cmH2O
        if( patientCircuitPressure.pressure > 0 )
          pressure_reading = patientCircuitPressure.pressure * INCHES_2_CM;
        patientCircuitPressure.startMeasurement();
      }
    }
    I2CMux.closeChannel(PATIENT_CIRCUIT_PRESSURE_MUX_CHANNEL);

    I2CMux.openChannel(VENTRUI_DIFFERENTIAL_PRESSURE_MUX_CHANNEL);
    sensorStatus = venturiDifferentialPressure.readStatus();
    if( !venturiDifferentialPressure.isBusy(sensorStatus) && !venturiDifferentialPressure.isError(sensorStatus) )
    {  
      if(!venturiDifferentialPressure.readData(false))
      {
        // Get a pressure reading and convert to units of cmH2O
        if( venturiDifferentialPressure.pressure > 0 )
        {
          venturiDifferentialPressureReading = venturiDifferentialPressure.pressure * INCHES_2_CM;
          // Transfer function found experimentally through bench testing
          // Flow [lpm] = f(pressure [cmH20])
          // see here for data:
          // https://docs.google.com/spreadsheets/d/1bI_WWhnsqxKvRou-n7BQC0ML1uTgSl8-lZcnprczsNk/edit?usp=sharing
          venturiFlowRateLpm = 26.486*sqrt(venturiDifferentialPressureReading)-0.276;
          
          // lowest reported flow rate = 0lpm
          venturiFlowRateLpm = venturiFlowRateLpm>0?venturiFlowRateLpm:0;
        }
        venturiDifferentialPressure.startMeasurement();
      }
    }
    I2CMux.closeChannel(VENTRUI_DIFFERENTIAL_PRESSURE_MUX_CHANNEL);
    PressureSensorLastStatusRead = micros();
  }
  return pressure_reading;
}

void print_pid_setpoint_and_current_value (void)
{
  if( CurrCycleStep != IDLE )
  {
    if( millis() % 50 == 0 )
    {
      Serial.print(pressure_system_input);
      Serial.print(" ");
      switch(CurrCycleStep)
      {
        case INHALE_RAMP:
          Serial.print(CurrPressureSetpointCentimetersH2O);
        break;
        case INHALE_HOLD:
          Serial.print(PipPressureCentimetersH2O);
        break;
        case EXHALE_RAMP:
        case EXHALE_HOLD:
        case IDLE:
        default:
          Serial.print(PeepPressureCentimetersH2O);
        break;
      }
      Serial.print(" ");
      Serial.print(blower_output_speed_in_percentage);
      Serial.print(" ");
      Serial.print(venturiDifferentialPressureReading);
      Serial.println();
    }
  }
}

void reset_blower_pid_integrator (void)
{
  Blower_PID.SetMode(MANUAL);
  blower_output_speed_in_percentage = 0;
  Blower_PID.SetMode(AUTOMATIC);
}

// Found via regression analysis from static pressure testing of sample blower
// r^2 = 0.99 for this regression on the blower tested.
// see data at https://docs.google.com/spreadsheets/d/1GVmF7gMihPsArEmjk8MefS8RsMpl3gKHL-qg1a0rInc/edit?usp=sharing
double blowerPressureToBlowerSpeed (double pressure)
{
    return sqrt(pressure)*14.16199 + pressure*0.07209 + 3.6105;
}

void write_calculated_pid_blower_speed(void)
{
  blower_speed = blowerPressureToBlowerSpeed(CurrPressureSetpointCentimetersH2O);
  
  blower_speed = mapf(blower_speed,
                    MIN_PERCENTAGE,
                    MAX_PERCENTAGE,
                    0,
                    1024);

  analogWrite(BLOWER_SPEED_PIN,(uint32_t)blower_speed);
}

// Write percent openness to the pinch valve
// 0% = closed
// 100% = wide open
void write_pinch_valve_openness (double opennessPercentage)
{
  long pinchValvePulseLengthMicroseconds = 
  map(opennessPercentage,
      MIN_PERCENTAGE,
      MAX_PERCENTAGE,
      PINCH_VALVE_DRIVER_FULL_CLOSE_PULSE_MICROSECONDS,
      PINCH_VALVE_DRIVER_FULL_OPEN_PULSE_MICROSECONDS);
  pinch_valve.writeMicroseconds(pinchValvePulseLengthMicroseconds);
}

void pinch_valve_control (void)
{
  switch (CurrCycleStep)
  {
  case EXHALE_RAMP:
    pinch_valve_output_openness_in_percentage = 100.0;
    break;
  case EXHALE_HOLD:
    PinchValve_PID.SetTunings(PinchValve_Kp,PinchValve_Ki,PinchValve_Kd);
    PinchValve_PID.Compute();
    break;
  case INHALE_HOLD:
  case INHALE_RAMP:
    pinch_valve_output_openness_in_percentage = 0.0;
    break;
  default:
    pinch_valve_output_openness_in_percentage = 100.0;
    break;
  }

  write_pinch_valve_openness(pinch_valve_output_openness_in_percentage);  
}

void cycle_state_handler (void)
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

double linear_remap_setpoint_compensation (double setpoint)
{
  return (1.116785714*setpoint)-0.5892857143;
}

void cycle_state_setpoint_handler(void)
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

  CurrPressureSetpointCentimetersH2O = linear_remap_setpoint_compensation(CurrPressureSetpointCentimetersH2O);
}

void buzzer_toggle (void)
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

void alarms_handler (void)
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
  //   buzzer_toggle();
  //   Serial.write("$ALARMS,G*");  // APNEA ALARM
  // }

  // TODO: Add High/Low RR Alarm
  // if(millis()-PrevAlarmTimeRRError > RRTimer)
  // {
  //   PrevAlarmTimeRRError = millis();
  //   buzzer_toggle();
  //   Serial.write("$ALARMS,H*");  // High/Low RR ALARM
  // }

  // TODO: Add I:E ratio Alarm
  // if(millis()-PrevAlarmTimeIEError > IETimer)
  // {
  //   PrevAlarmTimeIEError = millis();
  //   buzzer_toggle();
  //   Serial.write("$ALARMS,I*");  // I:E Ratio ALARM
  // }
}

void alarm_audible_and_visual_select (void)
{
  switch (alarm_state)
  {
    case 1:
      chase(red, low_red, LED_ON_TIME); // red
      digitalWrite(BUZZER_PIN, HIGH);
      break;
    case 2:
      chase(amber, low_amber, LED_ON_TIME); // amber
      buzzer_toggle();
      break;
    case 3:
    default:
      chase(green, low_green, LED_ON_TIME); // green
      digitalWrite(BUZZER_PIN, LOW);
      break;
  }
}

void send_values_to_raspberry_pi (void)
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
  pressure_system_input = get_pressure_reading();

  cycle_state_handler();

  cycle_state_setpoint_handler();

  pinch_valve_control();

  write_calculated_pid_blower_speed();

  // print_pid_setpoint_and_current_value();

  alarms_handler();

  alarm_audible_and_visual_select();

  get_values_from_raspberry_pi();

  send_values_to_raspberry_pi();
}
