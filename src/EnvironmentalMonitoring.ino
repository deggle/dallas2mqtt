#include <WiFi.h>
extern "C" {
  #include "freertos/FreeRTOS.h"
  #include "freertos/timers.h"
}

#include <Secrets.h>
#include <AsyncMqttClient.h>
#include <OneWire.h>
#include <DallasTemperature.h>

//#define MQTT_HOST "iot.preventx.net"
#define MQTT_HOST "10.0.3.1"
#define MQTT_PORT 1883
#define MQTT_PREFIX "px/temperature/"

#define ONE_WIRE_BUS 26
#define TEMPERATURE_PRECISION 9

#define BUTTON_PIN 39

OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
int numberOfDevices;

AsyncMqttClient mqttClient;
TimerHandle_t mqttReconnectTimer;
TimerHandle_t wifiReconnectTimer;

unsigned long previousMillis = 0;   // Stores last time temperature was published
const long interval = 10000;        // Interval at which to publish sensor readings

void connectToWifi() {
  Serial.println("Connecting to Wi-Fi...");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
}

void connectToMqtt() {
  Serial.println("Connecting to MQTT...");
  mqttClient.connect();
}

void WiFiEvent(WiFiEvent_t event) {
  Serial.printf("[WiFi Event] event: %d\n", event);
  switch(event) {
    case SYSTEM_EVENT_STA_GOT_IP:
      Serial.println("Wi-Fi connected.");
      Serial.println("IP address: ");
      Serial.println(WiFi.localIP());
      connectToMqtt();
      break;
    case SYSTEM_EVENT_STA_DISCONNECTED:
      Serial.println("Wi-Fi lost connection.");
      xTimerStop(mqttReconnectTimer, 0); // Avoid reconnecting to MQTT while Wi-Fi is down.
      xTimerStart(wifiReconnectTimer, 0);
      break;
  }
}

void onMqttConnect(bool sessionPresent) {
  Serial.println("Connected to MQTT.");
  Serial.print("Session present: ");
  Serial.println(sessionPresent);
}

void onMqttDisconnect(AsyncMqttClientDisconnectReason reason) {
  Serial.println("Disconnected from MQTT.");
  if (WiFi.isConnected()) {
    xTimerStart(mqttReconnectTimer, 0);
  }
}

/*void onMqttSubscribe(uint16_t packetId, uint8_t qos) {
  Serial.println("Subscribe acknowledged.");
  Serial.print("  packetId: ");
  Serial.println(packetId);
  Serial.print("  qos: ");
  Serial.println(qos);
}
void onMqttUnsubscribe(uint16_t packetId) {
  Serial.println("Unsubscribe acknowledged.");
  Serial.print("  packetId: ");
  Serial.println(packetId);
}*/

void onMqttPublish(uint16_t packetId) {
  Serial.println("Publish acknowledged.");
  Serial.print("  packetId: ");
  Serial.println(packetId);
}

void loopDevices() {


    
    Serial.println("Searching for 1-wire devices...");




  // Grab a count of devices on the wire
    numberOfDevices = sensors.getDeviceCount();

    Serial.print("Found ");
    Serial.print(numberOfDevices, DEC);
    Serial.println(" devices.");


    // // Search for connected devices...
    // uint8_t address[8];
    // uint8_t count = 0;

    // if (oneWire.search(address)) {
      
    //   do {
    //     count++;
    //     Serial.println("  {");
    //     for (uint8_t i = 0; i < 8; i++)
    //     {
    //       if (address[i] < 0x10) Serial.print("0");
    //       Serial.print(address[i], HEX);
    //     }
    //     Serial.println("...");
    //   } while (oneWire.search(address));

    //   Serial.println(" # ");
    //   Serial.print("// nr devices found: ");
    //   Serial.println(count);


      
    // } else {
    //   Serial.println("No devices found.");
    // }
    
}

void setup() {
 
  Serial.begin(115200);

  connectToWifi();

  mqttReconnectTimer = xTimerCreate("mqttTimer", pdMS_TO_TICKS(2000), pdFALSE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(connectToMqtt));
  wifiReconnectTimer = xTimerCreate("wifiTimer", pdMS_TO_TICKS(2000), pdFALSE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(connectToWifi));

  WiFi.onEvent(WiFiEvent);

  mqttClient.onConnect(onMqttConnect);
  mqttClient.onDisconnect(onMqttDisconnect);

  mqttClient.onPublish(onMqttPublish);
  mqttClient.setServer(MQTT_HOST, MQTT_PORT);
  //mqttClient.setCredentials("USER", "PASSW");
   
  // Start the DS18B20 sensor
  sensors.begin();
  loopDevices();

  
}

void loop() {
 
  // unsigned long currentMillis = millis();
  // // Every X number of seconds (interval = 10 seconds) 
  // // it publishes a new MQTT message

  // if (currentMillis - previousMillis >= interval) {
  //   // Save the last time a new reading was published
  //   previousMillis = currentMillis;
  //   // New temperature readings
  //   sensors.requestTemperatures(); 
  //   // Temperature in Celsius degrees
  //   temp = sensors.getTempCByIndex(0);
    
  //   // Publish an MQTT message on topic esp32/ds18b20/temperature
  //   uint16_t packetIdPub1 = mqttClient.publish(MQTT_PREFIX, 1, true, String(temp).c_str());                            
  //   Serial.printf("Publishing on topic %s at QoS 1, packetId: ", MQTT_PREFIX);
  //   Serial.println(packetIdPub1);
  //   Serial.printf("Message: %.2f \n", sensors.getTempCByIndex(0));

  // }

}
