#include "mqtt_irrigator.h"

#include <ESP8266WiFi.h>
#include <PubSubClient.h>
//needed for library
#include <DNSServer.h>
#include <ESP8266WebServer.h>
#include <WiFiManager.h>         //https://github.com/tzapu/WiFiManager
#include <EEPROM.h>
#include "hx711/hx711.h"

#define DEBUG

// GPIO defines
#define PIN_PUMP         D7                // nodemcu GPIO13
#define PIN_LED          LED_BUILTIN //D0  // nodemcu GPIO16 built in LED
#define PIN_BUTTON       D3                // nodemcu GPIO0 flash button
#define PIN_HX711_DOUT   D2                // nodemcu GPIO4
#define PIN_HX711_SCK    D1                // nodemcu GPIO5
//boot pins
//GPIO15 | D8 NodeMcu | MUST keep LOW,
//GPIO0  | D3 NodeMcu | HIGH ->RUN MODE, LOW -> FLASH MODE. flash button
//GPIO2  | D4 NodeMcu | MUST keep HIGH

//free GPIO

//WIFI and MQTT
WiFiClient espClient;
PubSubClient client(espClient);

// Update these with values suitable for your network.
const char* ssid = "........";
const char* password = "........";
const char* mqtt_server = "broker.mqtt-dashboard.com";

//LoadCell
HX711 loadcell(PIN_HX711_DOUT, PIN_HX711_SCK);

//IRRIGATOR
#define IRRIGATION_TIME        10 // irrigation time in sec
#define IRRIGATION_PAUSE_TIME  300 // irrigation pause time in sec - only for auto irrigator

// irrigator state
typedef enum {
  s_idle             = 0,  // irrigation idle
  s_irrigation_start = 1,  // start irrigation
  s_irrigation       = 2,  // irrigate
  s_irrigation_stop  = 3,  // irrigation stop
} e_state;


#define CONFIG_START 0
#define CONFIG_VERSION "v01"

StoreStruc storage = {
  .version = CONFIG_VERSION,
  // The default module 0
  .moduleId = {'0'},  // module id
  .hx711_cal = {0, 0},
};

#define PARAM_HUMIDITY_TRESHOLD   "Sensor.Parameter1"
#define PARAM_MANUAL_AUTO_MODE    "Sensor.Parameter2"
#define PARAM_PUMP_ON             "Sensor.Parameter3"
#define PARAM_HUMIDITY            "Sensor.Parameter4"

#define MS_IN_SEC  1000 // 1S  

// intvariables
e_state state;
float soilHumidityThreshold;
bool autoMode;
String valueStr("");
String topic("");
boolean result;

float lastAnalogReading;
bool autoModeOld;
float soilHumidityThresholdOld;
unsigned long startTime;
float soilHum;
int irrigatorCounter;

void setup() {
  state = s_idle;
  pinMode(PIN_PUMP, OUTPUT); 
  pinMode(PIN_LED, OUTPUT);
  pinMode(PIN_BUTTON, INPUT);

  autoMode = false;
  soilHumidityThresholdOld = -1;
  startTime = millis();
  soilHum = -1;
  
  Serial.begin(115200);

  //WiFiManager
  //Local intialization. Once its business is done, there is no need to keep it around
  WiFiManager wifiManager;
  //reset saved settings
  //wifiManager.resetSettings();

  //set custom ip for portal
  //wifiManager.setAPConfig(IPAddress(10,0,1,1), IPAddress(10,0,1,1), IPAddress(255,255,255,0));

  //fetches ssid and pass from eeprom and tries to connect
  //if it does not connect it starts an access point with the specified name
  //here  "AutoConnectAP"
  //and goes into a blocking loop awaiting configuration
  //wifiManager.autoConnect("AutoConnectAP");
  //or use this for auto generated name ESP + ChipID
  wifiManager.autoConnect();
  //if you get here you have connected to the WiFi
  Serial.println("connected...yeey :)");

  uint8_t mac[WL_MAC_ADDR_LENGTH];
  WiFi.macAddress(mac);
  macToStr(mac);

  EEPROM.begin(512);
  loadConfig(&storage);

  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);

  delay(500);

  //create module if necessary
  if (strcmp(storage.moduleId, ""))
  {
	  //TODO: calibrate module
#ifdef EASYIOT
    //create module
    Serial.println("create module: /NewModule");
    storage.moduleId = myMqtt.NewModule();

    if (storage.moduleId == 0)
    {
      Serial.println("Module NOT created. Check module limit");
      while(1)
        delay(100);
    }

   // set module type
    Serial.println("Set module type");
    myMqtt.SetModuleType(storage.moduleId, "ZMT_IRRIGATOR");

    // create Sensor.Parameter1 - humidity treshold value
    Serial.println("new parameter: /"+String(storage.moduleId)+ "/" +PARAM_HUMIDITY_TRESHOLD);
    myMqtt.NewModuleParameter(storage.moduleId, PARAM_HUMIDITY_TRESHOLD);

    // set IsCommand
    Serial.println("set isCommand: /"+String(storage.moduleId)+ "/" + PARAM_HUMIDITY_TRESHOLD);
    myMqtt.SetParameterIsCommand(storage.moduleId, PARAM_HUMIDITY_TRESHOLD, true);


    // create Sensor.Parameter2
    // Sensor.Parameter2 - manual/auto mode 0 - manual, 1 - auto mode
    Serial.println("new parameter: /"+String(storage.moduleId)+ "/" + PARAM_MANUAL_AUTO_MODE);
    myMqtt.NewModuleParameter(storage.moduleId, PARAM_MANUAL_AUTO_MODE);

    // set IsCommand
    Serial.println("set isCommand: /"+String(storage.moduleId)+ "/" + PARAM_MANUAL_AUTO_MODE);
    myMqtt.SetParameterIsCommand(storage.moduleId, PARAM_MANUAL_AUTO_MODE, true);


    // create Sensor.Parameter3
    // Sensor.Parameter3 - pump on/ pump off
    Serial.println("new parameter: /"+String(storage.moduleId)+ "/" + PARAM_PUMP_ON);
    myMqtt.NewModuleParameter(storage.moduleId, PARAM_PUMP_ON);


    // set IsCommand
    Serial.println("set isCommand: /"+String(storage.moduleId)+ "/" + PARAM_PUMP_ON);
    myMqtt.SetParameterIsCommand(storage.moduleId, PARAM_PUMP_ON, true);


    // create Sensor.Parameter4
    // Sensor.Parameter4 - current soil humidity
    Serial.println("new parameter: /"+String(storage.moduleId)+ "/" + PARAM_HUMIDITY);
    myMqtt.NewModuleParameter(storage.moduleId, PARAM_HUMIDITY);


    // set Description
    Serial.println("set description: /"+String(storage.moduleId)+ "/" + PARAM_HUMIDITY);
    myMqtt.SetParameterDescription(storage.moduleId, PARAM_HUMIDITY, "Soil moist.");

    // set Unit
    Serial.println("set Unit: /"+String(storage.moduleId)+ "/" + PARAM_HUMIDITY);
    myMqtt.SetParameterUnit(storage.moduleId, PARAM_HUMIDITY, "%");

    // set dbLogging
    Serial.println("set Unit: /"+String(storage.moduleId)+ "/" + PARAM_HUMIDITY);
    myMqtt.SetParameterDBLogging(storage.moduleId, PARAM_HUMIDITY, true);
#endif
    // save new module id
    saveConfig(&storage);
  }

  subscribe();

  loadcell.set_cal(&storage.hx711_cal);
  lastAnalogReading = loadcell.get_weight();
  Serial.print("Load_cell output val: ");
  Serial.println(lastAnalogReading);

  autoModeOld = !autoMode;
}

void loop() {

  if (!client.connected()) {
	digitalWrite(PIN_LED, LOW);
    reconnect();
  }
  digitalWrite(PIN_LED, HIGH);
  client.loop();

  int in = digitalRead(PIN_BUTTON);

  //Serial.println(in);
  if (in == 0)
  {
    while(digitalRead(PIN_BUTTON) == 0)
      delay(100);

    if (state == s_idle || state == s_irrigation_start)
      state = s_irrigation_start;
    else
      state = s_irrigation_stop;
  }
  
  // post auto mode changes
  if (autoModeOld != autoMode)
  {
    autoModeOld = autoMode;

    if (autoMode)    
      valueStr = String("1");
    else
      valueStr = String("0");
    
    topic  = "/"+String(storage.moduleId)+ "/" + PARAM_MANUAL_AUTO_MODE;
//    result = myMqtt.publish(topic, valueStr, 0, 1);
    result = client.publish(topic.c_str(), (const uint8_t *)valueStr.c_str() , 1, 0);

    Serial.print("Publish topic: ");
    Serial.print(topic);
    Serial.print(" value: ");
    Serial.println(valueStr);  
  }

  // post treshold changes
  if (soilHumidityThreshold != soilHumidityThresholdOld)
  {
    soilHumidityThresholdOld = soilHumidityThreshold;
    valueStr = String(soilHumidityThreshold);
    
    topic  = "/"+String(storage.moduleId)+ "/"+ PARAM_HUMIDITY_TRESHOLD;
//    result = myMqtt.publish(topic, valueStr, 0, 1);

    Serial.print("Publish topic: ");
    Serial.print(topic);
    Serial.print(" value: ");
    Serial.println(valueStr);  
  }
  
  if (IsTimeout())
  {
    startTime = millis();
    // process every second

    float aireading = loadcell.get_weight();
    Serial.print("Load_cell output val: ");
    Serial.println(aireading);

    Serial.print("Analog value: ");
    Serial.print(aireading);
    Serial.print(" ");
    // filter s
    lastAnalogReading += (aireading - lastAnalogReading) / 10;  
    Serial.print(lastAnalogReading); 
   
   // calculate soil humidity in %
   float newSoilHum = lastAnalogReading;
 
   // report soil humidity if changed
   if (soilHum != newSoilHum)
   {
     soilHum = newSoilHum;
     //esp.send(msgHum.set(soilHum)); 
     
     valueStr = String(soilHum);
     topic  = "/"+String(storage.moduleId)+ "/" + PARAM_HUMIDITY;
//     result = myMqtt.publish(topic, valueStr, 0, 1);

     Serial.print("Publish topic: ");
     Serial.print(topic);
     Serial.print(" value: ");
     Serial.println(valueStr);  
   }
   
   // irrigator state machine
   switch(state)
   {
     case s_idle:     
       if (irrigatorCounter <= IRRIGATION_PAUSE_TIME)
         irrigatorCounter++;
       
       if (irrigatorCounter >= IRRIGATION_PAUSE_TIME && autoMode)
       {
         if (soilHum <= soilHumidityThreshold)
           state = s_irrigation_start;       
       }         
       break;
     case s_irrigation_start:
       irrigatorCounter = 0;
       digitalWrite(PIN_PUMP, HIGH);
       //esp.send(msgMotorPump.set((uint8_t)1));       
       valueStr = String(1);
       topic  = "/"+String(storage.moduleId)+ "/" + PARAM_PUMP_ON;
//       result = myMqtt.publish(topic, valueStr, 0, 1);

       Serial.print("Publish topic: ");
       Serial.print(topic);
       Serial.print(" value: ");
       Serial.println(valueStr);  
 
       state = s_irrigation;
       break;
     case s_irrigation:
       if (irrigatorCounter++ > IRRIGATION_TIME)
         state = s_irrigation_stop;
       break;
     case s_irrigation_stop:
       irrigatorCounter = 0;
       state = s_idle;
       //esp.send(msgMotorPump.set((uint8_t)0));
       valueStr = String(0);
       topic  = "/"+String(storage.moduleId)+ "/" + PARAM_PUMP_ON;
//       result = myMqtt.publish(topic, valueStr, 0, 1);

       digitalWrite(PIN_PUMP, LOW);
       break;
   }
  }
}

void loadConfig(StoreStruc *pStorage) {
  // To make sure there are settings, and they are YOURS!
  // If nothing is found it will use the default settings.
	if (EEPROM.read(CONFIG_START + 0) == CONFIG_VERSION[0] &&
			EEPROM.read(CONFIG_START + 1) == CONFIG_VERSION[1] &&
			EEPROM.read(CONFIG_START + 2) == CONFIG_VERSION[2]) {
		for (unsigned int t=0; t<sizeof(StoreStruc); t++)
			*((char*)pStorage + t) = EEPROM.read(CONFIG_START + t);
	}
	else {
		//return default settings
		memset(pStorage, 0x00, sizeof(StoreStruc));

		pStorage->hx711_cal.factor = 1;
		pStorage->hx711_cal.offset = 0;
	}
}

void saveConfig(StoreStruc *pStorage) {
  for (unsigned int t=0; t<sizeof(storage); t++)
    EEPROM.write(CONFIG_START + t, *((char*)pStorage + t));

  EEPROM.commit();
}

String macToStr(const uint8_t* mac)
{
  String result;
  for (int i = 0; i < 6; ++i) {
    result += String(mac[i], 16);
    if (i < 5)
      result += ':';
  }
  return result;
}

boolean IsTimeout()
{
  unsigned long now = millis();
  if (startTime <= now)
  {
    if ( (unsigned long)(now - startTime )  < MS_IN_SEC ) 
      return false;
  }
  else
  {
    if ( (unsigned long)(startTime - now) < MS_IN_SEC ) 
      return false;
  }

  return true;
}


void subscribe()
{
	if (!strcmp(storage.moduleId, ""))
	{

		// Sensor.Parameter1 - humidity treshold value
		topic = "/"+String(storage.moduleId)+ "/" + PARAM_HUMIDITY_TRESHOLD;
		client.subscribe(topic.c_str());

		// Sensor.Parameter1 - humidity treshold value
		topic = "/"+String(storage.moduleId)+ "/" + PARAM_HUMIDITY_TRESHOLD;
		client.subscribe(topic.c_str());

		// Sensor.Parameter2 - manual/auto mode 0 - manual, 1 - auto mode
		topic = "/"+String(storage.moduleId)+ "/" + PARAM_MANUAL_AUTO_MODE;
		client.subscribe(topic.c_str());

		// Sensor.Parameter3 - pump on/ pump off
		topic = "/"+String(storage.moduleId)+ "/" + PARAM_PUMP_ON;
		client.subscribe(topic.c_str());
	}
}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect("ESP8266Client")) {
      Serial.println("connected");
      // Once connected, publish an announcement...
      client.publish("outTopic", "hello world");
      // ... and resubscribe
      client.subscribe("inTopic");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (unsigned int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();

  String data = String((char*)payload);

  if (String("/"+String(storage.moduleId)+ "/" + PARAM_HUMIDITY_TRESHOLD).compareTo(topic))
  {
    soilHumidityThreshold = data.toFloat();
    Serial.println("soilHumidityThreshold");
    Serial.println(data);
  }
  else if (String("/"+String(storage.moduleId)+ "/" + PARAM_MANUAL_AUTO_MODE).compareTo(topic))
  {
    autoMode = (data == String("1"));
    Serial.println("Auto mode");
    Serial.println(data);
  }
  else if (String("/"+String(storage.moduleId)+ "/" + PARAM_PUMP_ON).compareTo(topic))
  {
    //switchState = (data == String("1"))? true: false;
    if (data == String("1"))
      state = s_irrigation_start;
    else
      state = s_irrigation_stop;
    Serial.println("Pump");
    Serial.println(data);
  }
}
