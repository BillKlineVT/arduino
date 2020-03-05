/**
 * The MySensors Arduino library handles the wireless radio link and protocol
 * between your home built sensors/actuators and HA controller of choice.
 * The sensors forms a self healing radio network with optional repeaters. Each
 * repeater and gateway builds a routing tables in EEPROM which keeps track of the
 * network topology allowing messages to be routed to nodes.
 *
 * Created by Henrik Ekblad <henrik.ekblad@mysensors.org>
 * Copyright (C) 2013-2015 Sensnology AB
 * Full contributor list: https://github.com/mysensors/Arduino/graphs/contributors
 *
 * Documentation: http://www.mysensors.org
 * Support Forum: http://forum.mysensors.org
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 *******************************
 *
 * REVISION HISTORY
 * Version 1.0 - Henrik Ekblad
 * 
 * DESCRIPTION
 * Example sketch showing how to control physical relays. 
 * This example will remember relay state after power failure.
 * http://www.mysensors.org/build/relay
 */ 

// Enable debug prints to serial monitor
#define MY_DEBUG 

#define MY_NODE_ID 9

// Enable and select radio type attached
#define MY_RADIO_NRF24
//#define MY_RADIO_RFM69

// Enable repeater functionality for this node
#define MY_REPEATER_FEATURE

#include <SPI.h>
#include <MyConfig.h>
#include <MySensors.h>
#include <DallasTemperature.h>
#include <OneWire.h>

#define RELAY_1  5//7  // Arduino Digital I/O pin number for first relay
#define RELAY_2  6//8  // Arduino Digital I/O pin number for second relay
#define NUMBER_OF_RELAYS 1 // Total number of attached relays
#define RELAY_ON 1//0  // GPIO value to write to turn on attached relay
#define RELAY_OFF 0//1 // GPIO value to write to turn off attached relay
#define RELAY_PIN_FAN  5//7  // Arduino Digital I/O pin number for relay that triggers the circulating fans between fridge and freezer
#define RELAY_PIN_COMPRESSOR  6//8  // Arduino Digital I/O pin number for relay that triggers the compressor

#define ONE_WIRE_BUS 4 // Pin where dallas sensor is connected 
#define MAX_ATTACHED_DS18B20 2

#define CHILD_ID_TEMP_FRIDGE 0
#define CHILD_ID_TEMP_FREEZER 1
#define CHILD_ID_POURS_1 2
#define CHILD_ID_KICKS_1 3
#define CHILD_ID_POURS_2 4
#define CHILD_ID_KICKS_2 5
#define CHILD_ID_POURS_3 6
#define CHILD_ID_KICKS_3 7
#define CHILD_ID_POURS_4 8
#define CHILD_ID_KICKS_4 9
#define CHILD_ID_SETPOINT 10
#define CHILD_ID_THERMOSWITCH 11

//This line is the number of flow sensors connected.
const uint8_t numFlowSensors = 4;
//This line initializes an array with the pins connected to the flow sensors
uint8_t pulsePin[] = {0,1,2,3};
//number of milliseconds to wait after pour before sending message
unsigned int pourMsgDelay = 300;
unsigned int pulseCount[numFlowSensors];
unsigned int kickedCount[numFlowSensors];
unsigned long nowTime;
unsigned long lastPourTime = 0;
unsigned long lastPinStateChangeTime[numFlowSensors];
int lastPinState[numFlowSensors];

bool state;
String system_state = "Off";
String thermoMode = "Off";
int numTempSensors = 0;
float lastTemperature[MAX_ATTACHED_DS18B20];
float temperature = 0;
float fridge_temperature = 0;
float freezer_temperature = 0;
float setPoint = 38;
float temp_delta_threshold = 2;
float swing_high = 1; // swing range of thermostat on the high side of the range
float swing_low = 1; // swing range of thermostat on the low side of the range
unsigned long compressor_delay = 600000; // 10 minutes
unsigned long lastCoolStopTime = 0;
unsigned long circulate_min_time = 60000; // 1 minute
unsigned long lastCirculateStartTime = 0;
float recievedvalue;
boolean metric = false;
unsigned long SLEEP_TIME = 10000; // Sleep time between reads (in milliseconds)
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
unsigned long CHECK_TIME = millis();
// hard-code the addresses of the temp sensors.  0=fridge sensor, 1=freezer sensor
byte D[2][8] = {
{ 0x28, 0x92, 0x71, 0xCD, 0x04, 0x00, 0x00, 0x5B },
{ 0x28, 0xFE, 0x65, 0xCD, 0x04, 0x00, 0x00, 0x86 }
};
bool debug_messages = true;

MyMessage msgFridgeTemp(CHILD_ID_TEMP_FRIDGE, V_TEMP);
MyMessage msgFreezerTemp(CHILD_ID_TEMP_FREEZER, V_TEMP);
MyMessage msgSetpoint(CHILD_ID_SETPOINT, V_HVAC_SETPOINT_COOL);
MyMessage msgThermoSwitch(CHILD_ID_THERMOSWITCH, V_HVAC_FLOW_STATE);
MyMessage msgPourData_1(CHILD_ID_POURS_1, V_VAR1);
MyMessage msgKickedData_1(CHILD_ID_KICKS_1, V_VAR1);
MyMessage msgPourData_2(CHILD_ID_POURS_2, V_VAR1);
MyMessage msgKickedData_2(CHILD_ID_KICKS_2, V_VAR1);
MyMessage msgPourData_3(CHILD_ID_POURS_3, V_VAR1);
MyMessage msgKickedData_3(CHILD_ID_KICKS_3, V_VAR1);
MyMessage msgPourData_4(CHILD_ID_POURS_4, V_VAR1);
MyMessage msgKickedData_4(CHILD_ID_KICKS_4, V_VAR1);

void setup() {
  //Serial.begin(57600);
  //printf_begin();
  // startup onewire
  sensors.begin();
    
  Serial.flush();
  for( int i = 0; i < numFlowSensors; i++ ) {
    /*pinMode(pulsePin[i], INPUT);
    digitalWrite(pulsePin[i], HIGH);
    kickedCount[i] = 0;
    lastPinState[i] = digitalRead(pulsePin[i]);*/
    pinMode(pulsePin[i], INPUT_PULLUP);
  }  
  attachInterrupt(digitalPinToInterrupt(pulsePin[0]), onPulse_1, FALLING);
  attachInterrupt(digitalPinToInterrupt(pulsePin[1]), onPulse_2, FALLING);
  attachInterrupt(digitalPinToInterrupt(pulsePin[2]), onPulse_3, FALLING);
  attachInterrupt(digitalPinToInterrupt(pulsePin[3]), onPulse_4, FALLING);

  // Make sure relays are off when starting up
  digitalWrite(RELAY_PIN_FAN, RELAY_OFF);
  digitalWrite(RELAY_PIN_COMPRESSOR, RELAY_OFF);
  delay(500);
  // Then set relay pins in output mode
  pinMode(RELAY_PIN_FAN, OUTPUT);
  pinMode(RELAY_PIN_COMPRESSOR, OUTPUT);
  
  // Set setPoint to last known state (using eeprom storage)
  setPoint = loadState(CHILD_ID_SETPOINT);
  Serial.print("Last known SetPoint: ");
  Serial.println(setPoint);
  send(msgSetpoint.set(setPoint, true));

  if (setPoint > 120 || setPoint < 32)
  {
    setPoint = 38;
    saveState(CHILD_ID_SETPOINT, setPoint);
  }
}

void presentation()  
{   
  // Send the sketch version information to the gateway and Controller
  sendSketchInfo("KegeratorSensor_FridgeFreezerControl", "1.0");

  numTempSensors = sensors.getDeviceCount();
  Serial.print("numTempSensors Found: ");
  Serial.println(numTempSensors);
  // Register all sensors to gw (they will be created as child devices)

  // Present 1-wire sensors to controller
  for (int i = 0; i < numTempSensors && i < MAX_ATTACHED_DS18B20; i++)
  {
    present(i, S_TEMP);
  }
  present(CHILD_ID_POURS_1, S_WATER);
  present(CHILD_ID_POURS_2, S_WATER);
  present(CHILD_ID_POURS_3, S_WATER);
  present(CHILD_ID_POURS_4, S_WATER);

  present(CHILD_ID_THERMOSWITCH, S_HVAC);
}


void loop() 
{
  // Check Temp sensor every set ammount of time
  unsigned long NOW_TIME = millis();
  if(NOW_TIME - CHECK_TIME >= SLEEP_TIME)
  {
    getTemps();
    CHECK_TIME = NOW_TIME;
  }
  // --- Check Flow Sensors for activity ---
  nowTime = millis();
  //pollPins();
  if ( (nowTime - lastPourTime) > pourMsgDelay && lastPourTime > 0) {
    //only send pour messages after all taps have stopped pulsing for a short period
    //use lastPourTime=0 to ensure this code doesn't get run constantly
    lastPourTime = 0;
    checkPours();
//    checkKicks();
  }
  // --- End Check Flow Sensors ---
}

// quick filter of invalid sensor data
bool check_temp_valid(float temperature_value)
{
  if (temperature_value > -50 && temperature_value < 140)
    return true;
  else
    return false;
}

void getTemps()
{

  // Fetch temperatures from Dallas sensors
  sensors.requestTemperatures();
  wait(1000);
   // query conversion time and sleep until conversion completed
  //int16_t conversionTime = sensors.millisToWaitForConversion(sensors.getResolution());
  // sleep() call can be replaced by wait() call if node need to process incoming messages (or if node is repeater)
  //wait(conversionTime);

  // Read temperatures and send them to controller
  for (int i = 0; i < numTempSensors && i < MAX_ATTACHED_DS18B20; i++)
  {
    // Fetch and round temperature to one decimal
    //temperature = static_cast<float>(static_cast<int>((sensors.requestTemperaturesByAddress(D[i])) * 10.)) / 10.;
    float temperature = sensors.getTempF(D[i]);
    
    // Only send data if temperature has changed and no error
    if (lastTemperature[i] != temperature && temperature > 0.0 && temperature < 140.0)
    {
      // Send in the new temperature
      //send(msgTemp.setSensor(CHILD_ID_TEMP).set(temperature, 1));
      if (i== CHILD_ID_TEMP_FRIDGE) send(msgFridgeTemp.setSensor(i).set(temperature, 1));
      else if (i== CHILD_ID_TEMP_FREEZER) send(msgFreezerTemp.setSensor(i).set(temperature, 1));
      Serial.print("T: ");
      Serial.println(temperature);
      lastTemperature[i] = temperature;
      if (i == 0) 
      {
        fridge_temperature = temperature;
        Serial.print("Fridge: ");
        Serial.println(fridge_temperature);
      }
      else if (i == 1) 
      {
        freezer_temperature = temperature;
        Serial.print("Freezer: ");
        Serial.println(freezer_temperature);
      }
    }
  }
  if (debug_messages == true)
  {
    Serial.println("getTemps() called.  Current variable status: ");
    Serial.println("System State: ");
    Serial.println(system_state);
    Serial.println("Fridge Temp: ");
    Serial.println(fridge_temperature);
    Serial.println("Freezer Temp: ");
    Serial.println(freezer_temperature);
    Serial.println("temp_delta_threshold: ");
    Serial.println(temp_delta_threshold);
    Serial.println("Fridge Temp Validity: ");
    Serial.println(check_temp_valid(fridge_temperature));
    Serial.println("Freezer Temp Validity: ");
    Serial.println(check_temp_valid(freezer_temperature));
    Serial.println("Set Point: ");
    Serial.println(setPoint);
    Serial.println("Swing High: ");
    Serial.println(swing_high);
    Serial.println("Swing Low: ");
    Serial.println(swing_low);
    Serial.println("Compressor Delay: ");
    Serial.println(compressor_delay);
    Serial.println("Last Cool Stop Time: ");
    Serial.println(lastCoolStopTime);
    Serial.println("Min Circulate Time: ");
    Serial.println(circulate_min_time);
    Serial.println("Last Circulate Start Time: ");
    Serial.println(lastCirculateStartTime);
    Serial.println("Current Time: ");
    Serial.println(millis());
    
  }
  // send current setpoint to refresh display
  //send(msgSetpoint.set(setPoint, true));
  
  // ENTER CIRCULATE MODE HERE
  // if difference between fridge temp and freezer temp is greater than threshold, turn on circulating fans.
  if ( (system_state != "Circulate" && system_state != "Cool") && // make sure not already in a mode that should be lowering the temp
        abs(fridge_temperature - freezer_temperature) > temp_delta_threshold && //delta is above threshold
        check_temp_valid(fridge_temperature) && check_temp_valid(freezer_temperature)) //temp values are believable
  {
    Serial.println("Circulate Mode starting.  Fridge/Freezer temperature delta threshold exceeded, turning on circulating fans...");
    digitalWrite(RELAY_PIN_FAN, RELAY_ON);
    lastCirculateStartTime = millis();
    wait(500);
    system_state = "Circulate";
    Serial.println("Sending new system state to GW: ");
    Serial.println(system_state);
    send(msgThermoSwitch.set(system_state.c_str()), true);
  }

  // ENTER COOL MODE HERE
  // kick on compressor to cool off the freezer and turn on fans to equalize fridge/freezer temp delta
  if ( system_state != "Cool" && // make sure not already in this mode
  fridge_temperature > (setPoint + swing_high) && // temp is above set point plus swing range buffer
  (fridge_temperature + freezer_temperature)/2 > (setPoint + swing_high) &&  //if avg temp is below set temp, just needs to equalize via circulation
  check_temp_valid(fridge_temperature) && check_temp_valid(freezer_temperature)) //temp values are believable
  {
    nowTime = millis();
    // check to see how long ago last time the compressor ended running to set time delay to avoid over-cycling
    if ( (nowTime - lastCoolStopTime) > compressor_delay || lastCoolStopTime == 0) 
    {
      Serial.println("Cool Mode starting.  Turning on compressor...");
      digitalWrite(RELAY_PIN_COMPRESSOR, RELAY_ON);
      // also turn on circulating fan to make sure fridge gets equal cooling
      digitalWrite(RELAY_PIN_FAN, RELAY_ON);
      wait(500);
      system_state = "Cool";
      Serial.println("Sending new system state to GW: ");
      Serial.println(system_state);
      send(msgThermoSwitch.set(system_state.c_str()), true);
    }
    else
    {
      Serial.println("Cool Mode needed, but waiting for compressor delay to expire...");
    }
  }
  
  // EXIT COOL MODE HERE
  if (system_state == "Cool" && 
      //fridge_temperature < (setPoint - swing_low) && // fridge temp achieves set temp minus swing range buffer
      (fridge_temperature + freezer_temperature)/2 < (setPoint - swing_low) && // avg temp of both is less than setpoint, that way can circulate to even out
      check_temp_valid(fridge_temperature) && check_temp_valid(freezer_temperature)) //temp values are believable
  {
      Serial.println("Set temp achieved, turning OFF compressor and fans");
      lastCoolStopTime = millis(); // log when compressor turns off to be used for compressor delay check
      digitalWrite(RELAY_PIN_COMPRESSOR, RELAY_OFF);
      // turn off fans too.  If delta temp is above threshold, will jump to "circulate mode" at next assessment of status
      digitalWrite(RELAY_PIN_FAN, RELAY_OFF);
      wait(500);
      system_state = "Off";
      Serial.println("Sending new system state to GW: ");
      Serial.println(system_state);
      send(msgThermoSwitch.set(system_state.c_str()), true);
  }

  // EXIT CIRCULATE MODE HERE
  if (system_state == "Circulate" &&
      abs(fridge_temperature - freezer_temperature) <= temp_delta_threshold && //delta is below threshold
      (millis() - lastCirculateStartTime) > circulate_min_time && // don't keep cycling on/off so have min run time
      check_temp_valid(fridge_temperature) && check_temp_valid(freezer_temperature)) //temp values are believable
   {
      Serial.println("Circulate Mode finished.  Turning off fans...");
      digitalWrite(RELAY_PIN_FAN, RELAY_OFF);
      wait(500);
      system_state = "Off";
      Serial.println("Sending new system state to GW: ");
      Serial.println(system_state);
      send(msgThermoSwitch.set(system_state.c_str()), true);
   }
}
/*void pollPins() {
  for ( int i = 0; i < numFlowSensors; i++ ) {
    int pinState = digitalRead(pulsePin[i]);
    if ( pinState != lastPinState[i] ) {
      if ( pinState == HIGH ) {
        //separate high speed pulses to detect kicked kegs
        if( nowTime - lastPinStateChangeTime[i] > 0 ){
          pulseCount[i] ++;
        }
        else{
          kickedCount[i] ++;
        }
        lastPinStateChangeTime[i] = nowTime;
        lastPourTime = nowTime;
      }
      lastPinState[i] = pinState;
    }
  }
}*/

void onPulse_1()     
{
  pulseCount[0]++; 
  lastPourTime = millis();
}

void onPulse_2()     
{
  pulseCount[1]++; 
  lastPourTime = millis();
}

void onPulse_3()     
{
  pulseCount[2]++; 
  lastPourTime = millis();
}

void onPulse_4()     
{
  pulseCount[3]++; 
  lastPourTime = millis();
}

void checkPours() {
  for( int i = 0; i < numFlowSensors; i++ ) {
    if ( pulseCount[i] > 0 ) {
      if ( pulseCount[i] > 100 ) {
      //filter out tiny bursts
        //sendPulseCount(pulsePin[i], pulseCount[i]);
          switch(i)
          {
            case 0:
              send(msgPourData_1.set(pulseCount[i]));
              break;
            case 1:
              send(msgPourData_2.set(pulseCount[i]));
              break;
            case 2:
              send(msgPourData_3.set(pulseCount[i]));
              break;
            case 3:
              send(msgPourData_4.set(pulseCount[i]));
              break;
          }
      }
      pulseCount[i] = 0;
    }
  }
}

/*void checkKicks() {
  for( int i = 0; i < numFlowSensors; i++ ) {
    if ( kickedCount[i] > 0 ) {
      if ( kickedCount[i] > 30 ) {
        //if there are enough high speed pulses, send a kicked message
        sendKickedMsg(pulsePin[i]);
      }
      //reset the counter if any high speed pulses exist
      kickedCount[i] = 0;
    }
  }
}*/

/*void sendPulseCount(unsigned int pinNumber, unsigned int pulseCount)
{
  pulsePin_1 = pulsePin[0];
  pulsePin_2 = pulsePin[1];
  pulsePin_3 = pulsePin[2];
  pulsePin_4 = pulsePin[3];
  switch(pinNumber)
  {
  case pulsePin[0]:
    send(msgPourData_1.set(pulseCount));
    break;
  case pulsePin[1]:
    send(msgPourData_2.set(pulseCount));
    break;
  case pulsePin[2]:
    send(msgPourData_3.set(pulseCount));
    break;
  case pulsePin[3]:
    send(msgPourData_4.set(pulseCount));
    break;
  }
}*/

/*void sendKickedMsg(unsigned int pinNumber)
{
  switch(pinNumber)
  {
  case pulsePin[0]:
    send(msgKickedData_1.set(1));
    break;
  case pulsePin[1]:
    send(msgKickedData_2.set(1));
    break;
  case pulsePin[2]:
    send(msgKickedData_3.set(1));
    break;
  case pulsePin[3]:
    send(msgKickedData_4.set(1));
    break;
  }
}*/


void receive(const MyMessage &message)
{
  if (message.type == V_HVAC_SETPOINT_COOL)
  {
    
    recievedvalue = String(message.data).toFloat();

    
    if (recievedvalue > 0)
    {
      Serial.println(String(message.data));
      setPoint = recievedvalue;
      Serial.println("New SetPoint Temp recieved: " + String(setPoint));
      send(msgSetpoint.set(setPoint, 1));
      //Store state in eeprom
      saveState(CHILD_ID_SETPOINT, setPoint);
    }
  }
  /*if (message.type == V_HVAC_FLOW_STATE)
  {
    Serial.println("Flow State Message received");
    Serial.println(String(message.data));

    if(String(message.data) == "Circulate")
    {
      //digitalWrite(RELAY_PIN, RELAY_ON);
      wait(500);
      system_state = "Circulate";
    }

    if(String(message.data) == "Cool")
    {
      //digitalWrite(RELAY_PIN, RELAY_ON);
      wait(500);
      system_state = "Cool";
    }

    if(String(message.data) == "Off")
    {
      //digitalWrite(RELAY_PIN, RELAY_OFF);
      wait(500);
      system_state = "Off";
    }
  }*/
}


