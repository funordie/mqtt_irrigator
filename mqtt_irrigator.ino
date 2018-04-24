#include "mqtt_irrigator.h"
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
//needed for library
#include <DNSServer.h>
#include <ESP8266WebServer.h>
#include <WiFiManager.h>         //https://github.com/tzapu/WiFiManager
#include <EEPROM.h>
#include "hx711/hx711.h"

// GPIO defines
#define PIN_PUMP         D7                // nodemcu GPIO13
#define PIN_LED          D6                // nodemcu GPIO12
#define PIN_BUTTON       D3                // nodemcu GPIO00 flash button
#define PIN_HX711_DOUT   D2                // nodemcu GPIO04
#define PIN_HX711_SCK    D1                // nodemcu GPIO05
#define PIN_CAL          D5                // nodemcu GPI014

//boot pins
//GPIO15 | D8 NodeMcu | MUST keep LOW,
//GPIO00  | D3 NodeMcu | HIGH ->RUN MODE, LOW -> FLASH MODE. flash button
//GPIO02  | D4 NodeMcu | MUST keep HIGH

//free GPIO
//GPIO16 | D0 NodeMcu | built in LED | Wake Up

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
#define IRRIGATION_TIME        5000 // irrigation time in msec

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
        .version = "v00",
        // The default module 0
        .moduleId = {0},  // module id
        .hx711_cal = {0, 0},
};

#define PARAM_HUMIDITY_TRESHOLD   "Sensor.Parameter1"
#define PARAM_MANUAL_AUTO_MODE    "Sensor.Parameter2"
#define PARAM_PUMP_ON             "Sensor.Parameter3"
#define PARAM_HUMIDITY            "Sensor.Parameter4"
#define PARAM_HUMIDITY_SETPOINT   "Sensor.Parameter5"

#define MS_IN_SEC  1000 // 1S  
#define CAL_VALUE 1000 //value in [gr]

// intvariables
e_state state;
float soilHumidityThreshold;
float soilHumiditySetPoint;
bool autoMode;

float soilHum;

String valueStr("");
String topic("");

void led_blink(uint8_t gpio, uint16_t count, uint32_t on_time_ms, uint32_t off_time_ms) {

    delay(1000); //1s delay before start

    while(count > 0) {
        digitalWrite(gpio, HIGH);
        delay(on_time_ms);
        digitalWrite(gpio, LOW);
        delay(off_time_ms);
    }
}

class Interval {
public:
    Interval();
    ~Interval();
    unsigned long getValue();
    void setStart();
private:
    unsigned long m_start;
};

Interval::Interval() {
    m_start = 0;
}
Interval::~Interval() {

}
void Interval::setStart() {
    m_start = millis();
}

unsigned long Interval::getValue() {
    unsigned long now = millis();
    return (now - m_start);
}

Interval interval;

void setup() {
    state = s_idle;
    pinMode(PIN_PUMP, OUTPUT);
    pinMode(PIN_LED, OUTPUT);
    pinMode(PIN_BUTTON, INPUT);
    pinMode(PIN_CAL, INPUT);

    autoMode = false;

    Serial.begin(115200);

    /*
     * WIFI
    */

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

    /*
     * MQTT
    */

    client.setServer(mqtt_server, 1883);
    client.setCallback(callback);
    subscribe();

    delay(500);

    /*
     * EEPROM
    */
    EEPROM.begin(512);
    loadConfig(&storage);

    //create module if necessary
    if (strcmp(storage.version, CONFIG_VERSION))
    {
        Serial.printf("Calibration is missing!!!!\n");
        while(digitalRead(PIN_CAL) == 1) {
            Serial.printf("Wait for cal zero PIN_CAL == 1");
            delay(100);
        }
        //calibrate with ZERO and FULL load
        //wait to release PIN_CAL
        while(digitalRead(PIN_CAL) == 0) {
            Serial.printf("Wait for cal zero PIN_CAL == 0");
            delay(100);
        }

        //blink 2 time to indicate cal zero
        led_blink(PIN_LED, 2, 1000, 1000);

        //get zero cal
        long zero = loadcell.get_raw();

        //wait PIN_CAL == 1
        while(digitalRead(PIN_CAL) == 1) {
            Serial.printf("Wait for cal FULL[%d gr] PIN_CAL == 1", CAL_VALUE);
            delay(100);
        }

        //wait PIN_CAL == 0
        while(digitalRead(PIN_CAL) == 0) {
            Serial.printf("Wait for cal FULL[%d gr] PIN_CAL == 0", CAL_VALUE);
            delay(100);
        }

        //blink 2 time to indicate cal CAL_VALUE
        led_blink(PIN_LED, 2, 1000, 1000);

        //get CAL_VALUE cal
        long full = loadcell.get_raw();

        //fill storage structure
        storage.hx711_cal.factor = (full - zero)/CAL_VALUE;
        storage.hx711_cal.offset = zero;
        memcpy(storage.version, CONFIG_VERSION, sizeof(storage.version));
        memcpy(storage.moduleId, mac, sizeof(storage.moduleId));

        // save new module id
        saveConfig(&storage);
    }

    loadcell.set_cal(&storage.hx711_cal);
    soilHum = loadcell.get_weight();
    Serial.printf("Load_cell output val:%f\n\n\n", soilHum);
}

void loop() {

    boolean result;
    static float soilHumiditySetpointOld;
    static float soilHumidityThresholdOld;
    static bool autoModeOld;
    static float soilHumOld;

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
    }

    // post auto mode changes
    if (autoModeOld != autoMode)
    {

        if(autoMode == 0 && autoModeOld == 1) {
            //if mode is change from auto to manual -> stop the pump
            state = s_irrigation_stop;
        }
        autoModeOld = autoMode;

        if (autoMode)
            valueStr = String("1");
        else
            valueStr = String("0");

        topic  = "/"+String((char*)storage.moduleId)+ "/" + PARAM_MANUAL_AUTO_MODE;
        result = client.publish(topic.c_str(), (const uint8_t *)valueStr.c_str() , 1, 0);
        Serial.printf("Publish topic:%s value:%s res:%d\n", topic.c_str(), valueStr.c_str(), result);
    }

    // post treshold changes
    if (soilHumidityThreshold != soilHumidityThresholdOld)
    {
        soilHumidityThresholdOld = soilHumidityThreshold;
        valueStr = String(soilHumidityThreshold);

        topic  = "/"+String((char*)storage.moduleId)+ "/"+ PARAM_HUMIDITY_TRESHOLD;
        result = client.publish(topic.c_str(), (const uint8_t *)valueStr.c_str() , 1, 0);
        Serial.printf("Publish topic:%s value:%s res:%d\n", topic.c_str(), valueStr.c_str(), result);
    }

    // post setpoint changes
    if (soilHumiditySetPoint != soilHumiditySetpointOld)
    {
        soilHumiditySetpointOld = soilHumiditySetPoint;
        valueStr = String(soilHumiditySetPoint);

        topic  = "/"+String((char*)storage.moduleId)+ "/"+ PARAM_HUMIDITY_SETPOINT;
        result = client.publish(topic.c_str(), (const uint8_t *)valueStr.c_str() , 1, 0);
        Serial.printf("Publish topic:%s value:%s res:%d\n", topic.c_str(), valueStr.c_str(), result);
    }

    if (IsTimeout())
    {
        // process every second
        soilHum = loadcell.get_weight();

        // report soil humidity if changed
        if (soilHum != soilHumOld)
        {
            soilHumOld = soilHum;
            //esp.send(msgHum.set(soilHum));

            valueStr = String(soilHum);
            topic  = "/"+String((char*)storage.moduleId)+ "/" + PARAM_HUMIDITY;
            result = client.publish(topic.c_str(), (const uint8_t *)valueStr.c_str() , 1, 0);
            Serial.printf("Publish topic:%s value:%s res:%d\n", topic.c_str(), valueStr.c_str(), result);
        }

        // irrigator state machine
        switch(state)
        {
        case s_idle:
        {
            if (autoMode && ((soilHum + soilHumidityThreshold) < soilHumiditySetPoint))
                    state = s_irrigation_start;

            break;
        }
        case s_irrigation_start:
        {
            interval.setStart();
            digitalWrite(PIN_PUMP, HIGH);
            //esp.send(msgMotorPump.set((uint8_t)1));
            valueStr = String(1);
            topic  = "/"+String((char*)storage.moduleId)+ "/" + PARAM_PUMP_ON;
            result = client.publish(topic.c_str(), (const uint8_t *)valueStr.c_str() , 1, 0);
            Serial.printf("Publish topic:%s value:%s res:%d\n", topic.c_str(), valueStr.c_str(), result);

            state = s_irrigation;
            break;
        }
        case s_irrigation:
        {
            if (autoMode && (interval.getValue() >= IRRIGATION_TIME))
                    state = s_irrigation_stop;

            break;
        }
        case s_irrigation_stop:
        {
            if ( autoMode && (soilHum <= soilHumiditySetPoint)) {
                //start irrigation again, if Humidity is below set point
                state = s_irrigation_start;
            }
            else {
                digitalWrite(PIN_PUMP, LOW);
                state = s_idle;

                valueStr = String(0);
                topic  = "/"+String((char*)storage.moduleId)+ "/" + PARAM_PUMP_ON;
                result = client.publish(topic.c_str(), (const uint8_t *)valueStr.c_str() , 1, 0);
                Serial.printf("Publish topic:%s value:%s res:%d\n", topic.c_str(), valueStr.c_str(), result);
            }
            break;
        }
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

        Serial.printf("loadConfig: factor:%f offset:%f\n",
                pStorage->hx711_cal.factor,
                pStorage->hx711_cal.offset);
    }
    else {
        //return zero settings
        memset(pStorage, 0x00, sizeof(StoreStruc));

        Serial.printf("loadConfig: return ZERO\n");
    }
}

void saveConfig(StoreStruc *pStorage) {

    Serial.printf("saveConfig: factor:%f offset:%f\n",
            pStorage->hx711_cal.factor,
            pStorage->hx711_cal.offset);

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
    static unsigned long prev = 0;
    unsigned long now = millis();

    if ((now - prev) < MS_IN_SEC) {
        return false;
    }

    prev = now;
    return true;
}

void subscribe()
{

    // Sensor.Parameter1 - humidity treshold value
    topic = "/"+String((char*)storage.moduleId)+ "/" + PARAM_HUMIDITY_TRESHOLD;
    client.subscribe(topic.c_str());

    // Sensor.Parameter1 - humidity treshold value
    topic = "/"+String((char*)storage.moduleId)+ "/" + PARAM_HUMIDITY_TRESHOLD;
    client.subscribe(topic.c_str());

    // Sensor.Parameter2 - manual/auto mode 0 - manual, 1 - auto mode
    topic = "/"+String((char*)storage.moduleId)+ "/" + PARAM_MANUAL_AUTO_MODE;
    client.subscribe(topic.c_str());

    // Sensor.Parameter3 - pump on/ pump off
    topic = "/"+String((char*)storage.moduleId)+ "/" + PARAM_PUMP_ON;
    client.subscribe(topic.c_str());

    // Sensor.Parameter5 - humidity setpoint value
    topic = "/"+String((char*)storage.moduleId)+ "/" + PARAM_HUMIDITY_SETPOINT;
    client.subscribe(topic.c_str());
}

void publish() {

    valueStr = String(soilHum);
    topic  = "/"+String((char*)storage.moduleId)+ "/" + PARAM_HUMIDITY;
    Serial.printf("Publish topic:%s value:%s\n", topic.c_str(), valueStr.c_str());
}

void reconnect() {
    // Loop until we're reconnected
    while (!client.connected()) {
        Serial.printf("Attempting MQTT connection...\n");
        // Attempt to connect
        if (client.connect("ip_ESP8266")) {
            Serial.printf("connected\n");
//            // Once connected, publish an announcement...
//            client.publish("outTopic", "hello world");
//            // ... and resubscribe
//            client.subscribe("inTopic");
            subscribe();
        } else {
            Serial.printf("failed, rc=%d try again in 5 seconds\n", client.state());
            // Wait 5 seconds before retrying
            delay(5000);
        }
    }
}

void callback(char* topic, byte* payload, unsigned int length) {
    Serial.printf("callback: Message arrived [%s] ", topic);
    for (unsigned int i = 0; i < length; i++) {
        Serial.print((char)payload[i]);
    }
    Serial.println();

    String data = String((char*)payload);

    if (String("/"+String((char*)storage.moduleId)+ "/" + PARAM_HUMIDITY_TRESHOLD).compareTo(topic))
    {
        soilHumidityThreshold = data.toFloat();
        Serial.printf("callback: soilHumidityThreshold:%s\n", data.c_str());
    }
    else if (String("/"+String((char*)storage.moduleId)+ "/" + PARAM_HUMIDITY_SETPOINT).compareTo(topic))
    {
        soilHumiditySetPoint = data.toFloat();
        Serial.printf("callback: soilHumiditySetPoint:%s\n", data.c_str());
    }
    else if (String("/"+String((char*)storage.moduleId)+ "/" + PARAM_MANUAL_AUTO_MODE).compareTo(topic))
    {
        autoMode = (data == String("1"));
        Serial.printf("callback: Auto mode:%s", data.c_str());
    }
    else if (String("/"+String((char*)storage.moduleId)+ "/" + PARAM_PUMP_ON).compareTo(topic))
    {
        if (data == String("1"))
            state = s_irrigation_start;
        else
            state = s_irrigation_stop;
        Serial.printf("callback: Pump:%s\n", data.c_str());
    }
}
