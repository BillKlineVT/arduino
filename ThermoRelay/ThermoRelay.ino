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

#define MY_NODE_ID 1

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

#define RELAY_1  8  // Arduino Digital I/O pin number for first relay (second on pin+1 etc)
#define NUMBER_OF_RELAYS 1 // Total number of attached relays
#define RELAY_ON 0  // GPIO value to write to turn on attached relay
#define RELAY_OFF 1 // GPIO value to write to turn off attached relay
#define RELAY_PIN  8  // Arduino Digital I/O pin number for relay 
#define ONE_WIRE_BUS 3 // Pin where dallase sensor is connected 
#define MAX_ATTACHED_DS18B20 2
#define CHILD_ID_THERMOSWITCH 2
#define CHILD_ID_TEMP 0
#define CHILD_ID_SETPOINT 1

bool state;
String SWstate = "Off";
int numSensors = 0;
float lastTemperature[MAX_ATTACHED_DS18B20];
float temperature = 0;
String thermoMode = "Off";
float setPoint = 70;
float recievedvalue;
boolean metric = false;
unsigned long SLEEP_TIME = 10000; // Sleep time between reads (in milliseconds)
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
unsigned long CHECK_TIME = millis();

MyMessage msgTemp(CHILD_ID_TEMP, V_TEMP);
MyMessage msgSetpoint(CHILD_ID_SETPOINT, V_HVAC_SETPOINT_COOL);
MyMessage msgThermoSwitch(CHILD_ID_THERMOSWITCH, V_HVAC_FLOW_STATE);

void setup() {
  // startup onewire
  sensors.begin();
  
  // Make sure relays are off when starting up
  digitalWrite(RELAY_PIN, RELAY_OFF);
  delay(500);
  // Then set relay pins in output mode
  pinMode(RELAY_PIN, OUTPUT);

  // Set setPoint to last known state (using eeprom storage)
  setPoint = loadState(CHILD_ID_SETPOINT);
  Serial.print("Last known SetPoint: ");
  Serial.println(setPoint);

  if (setPoint > 120 || setPoint < 32)
  {
    setPoint = 71;
    saveState(CHILD_ID_SETPOINT, setPoint);
  }
  //send(msgSetpoint.set(setPoint, 1));
}

void presentation()  
{   
  // Send the sketch version information to the gateway and Controller
  sendSketchInfo("ThermoRelay", "1.0");

  numSensors = sensors.getDeviceCount();
  Serial.print("NumSensors Found: ");
  Serial.println(numSensors);
  // Register all sensors to gw (they will be created as child devices)

  // Present 1-wire sensors to controller
  for (int i = 0; i < numSensors && i < MAX_ATTACHED_DS18B20; i++)
  {
    present(i, S_TEMP);
  }

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
}

void getTemps()
{

  // Fetch temperatures from Dallas sensors
  sensors.requestTemperatures();

  // Read temperatures and send them to controller
  for (int i = 0; i < numSensors && i < MAX_ATTACHED_DS18B20; i++)
  {

    // Fetch and round temperature to one decimal
    //temperature = static_cast<float>(static_cast<int>((getConfig().isMetric ? sensors.getTempCByIndex(i) : sensors.getTempFByIndex(i)) * 10.)) / 10.;
    temperature = static_cast<float>(static_cast<int>(sensors.getTempFByIndex(i) * 10.)) / 10.;
    // Only send data if temperature has changed and no error
    if (lastTemperature[i] != temperature && temperature > 32.0 && temperature < 140.0)
    {

      // Send in the new temperature
      send(msgTemp.setSensor(CHILD_ID_TEMP).set(temperature, 1));
      send(msgTemp.setSensor(i).set(temperature, 1));
      Serial.print("T: ");
      Serial.println(temperature);
      lastTemperature[i] = temperature;
      // send current mode state and setpoint
      //send(msgThermoSwitch.set(SWstate.c_str()), true);
      //send(msgSetpoint.set(setPoint, 1));
    }
  }

  if (SWstate == "Off" && (temperature - 1 > setPoint) && temperature > 32.0 && temperature < 140.0)
  {
    Serial.println("Fan ON by temperature");
    digitalWrite(RELAY_PIN, RELAY_ON);
    wait(500);
    SWstate = "CoolOn";
    Serial.println("Sending new switch state to GW: ");
    Serial.println(SWstate);
    send(msgThermoSwitch.set(SWstate.c_str()), true);
  }

  if (SWstate == "CoolOn" && (temperature + 1 < setPoint) && temperature > 32.0 && temperature < 140.0)
  {
    Serial.println("Fan OFF by temperature");
    digitalWrite(RELAY_PIN, RELAY_OFF);
    wait(500);
    SWstate = "Off";
    Serial.println("Sending new switch state to GW: ");
    Serial.println(SWstate);
    send(msgThermoSwitch.set(SWstate.c_str()), true);
  }

}

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
  if (message.type == V_HVAC_FLOW_STATE)
  {
    Serial.println("Flow State Message received");
    Serial.println(String(message.data));


    if(String(message.data) == "CoolOn")
    {
      digitalWrite(RELAY_PIN, RELAY_ON);
      wait(500);
      SWstate = "CoolOn";
    }

    if(String(message.data) == "Off")
    {
      digitalWrite(RELAY_PIN, RELAY_OFF);
      wait(500);
      SWstate = "Off";
    }
  }
}

