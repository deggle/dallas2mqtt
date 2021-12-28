#include <WiFi.h>
#include <Secrets.h>
#include <AsyncMqttClient.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Button2.h>

#define FASTLED_INTERNAL
#include <FastLED.h> 

extern "C" {
  #include "freertos/FreeRTOS.h"
  #include "freertos/timers.h"
}

#define ONE_WIRE_BUS 26
#define TEMPERATURE_PRECISION 12
#define PIN_LEDATOM 27
#define BUTTON_PIN 39

CRGB ledAtom[1];
Button2 button;
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

AsyncMqttClient mqttClient;
TimerHandle_t mqttReconnectTimer;
TimerHandle_t wifiReconnectTimer;

int numberOfDevices;                // Number of devices found at boot time.
unsigned long previousMillis = 0;   // Stores last time temperature was published.
const long interval = 10000;        // Interval at which to publish sensor readings.
DeviceAddress tempDeviceAddress;    // Holds current address while looping.

void connectToWifi() {
  Serial.println("[WiFi  ] Connecting to Wi-Fi...");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
}

void connectToMqtt() {
  Serial.println("[MQTT  ] Connecting to MQTT...");
  mqttClient.connect();
}

void WiFiEvent(WiFiEvent_t event) {
  switch(event) {
    case SYSTEM_EVENT_STA_GOT_IP:
      Serial.println("[WiFi  ] Wi-Fi connected.");
      Serial.print("[WiFi  ] IP address: ");
      Serial.println(WiFi.localIP());
      connectToMqtt();
      break;
    case SYSTEM_EVENT_STA_DISCONNECTED:
      Serial.println("[WiFi  ] Wi-Fi connection lost.");
      ledAtom[0].setRGB(255,0,0); FastLED.show();
      xTimerStop(mqttReconnectTimer, 0); // Avoid reconnecting to MQTT while Wi-Fi is down.
      xTimerStart(wifiReconnectTimer, 0);
      break;
    default:
      break;
  }
}

void onMqttConnect(bool sessionPresent) {
  Serial.print("[MQTT  ] Connected to MQTT, Client ID: ");
  Serial.println(mqttClient.getClientId());
}

void onMqttDisconnect(AsyncMqttClientDisconnectReason reason) {
  Serial.println("[MQTT  ] Disconnected from MQTT.");
  ledAtom[0].setRGB(255,0,0); FastLED.show();
  if (WiFi.isConnected()) {
    xTimerStart(mqttReconnectTimer, 0);
  }
}

void onMqttPublish(uint16_t packetId) {
  Serial.print("[MQTT  ] Publish acknowledged, packet ID=");
  Serial.print(packetId);
  Serial.println(".");
}

void searchDevices() {

  Serial.println("[Sensor] Searching for 1-wire devices...");

  // Grab a count of devices on the wire
  sensors.begin();
  sensors.begin(); // For some reason, this only works when called twice!?

  numberOfDevices = sensors.getDeviceCount();
  int numberOfDS18 = sensors.getDS18Count();

  Serial.print("[Sensor] Found ");
  Serial.print(numberOfDevices, DEC);
  Serial.print(" devices (of which ");
  Serial.print(numberOfDS18, DEC);
  Serial.println(" are DS18).");

  // Loop through each device, print out address and set resolution...
  for(int i=0;i<numberOfDevices; i++)
  {
    if(sensors.getAddress(tempDeviceAddress, i))
    {
      Serial.print("[Sensor] Found device ");
      Serial.print(i, DEC);
      Serial.print(" with address " + getAddress(tempDeviceAddress));
      Serial.println("."); 

      Serial.print("[Sensor] Setting resolution to ");
      Serial.print(TEMPERATURE_PRECISION, DEC);
      Serial.println(" bits.");
      sensors.setResolution(tempDeviceAddress, TEMPERATURE_PRECISION);
      
      Serial.print("[Sensor] Resolution currently set to: ");
      Serial.print(sensors.getResolution(tempDeviceAddress), DEC); 
      Serial.println();
    }else{
      Serial.print("[Sensor] Found ghost device at ");
      Serial.print(i, DEC);
      Serial.print(" but could not detect address (check power and cabling).");
    }
  }
    
}

void setup() {

  FastLED.addLeds<NEOPIXEL, PIN_LEDATOM>(ledAtom, 1);
  ledAtom[0].setRGB(255,0,0); FastLED.show();

  Serial.begin(115200);
  delay(50);
  Serial.println();
  Serial.println();

  button.begin(BUTTON_PIN);
  button.setLongClickTime(1000);
  button.setDoubleClickTime(400);

  Serial.println("[Button] DoubleClick Time: " + String(button.getDoubleClickTime()) + "ms");
  Serial.println("[Button] Longpress Time: " + String(button.getLongClickTime()) + "ms");
  
  button.setClickHandler(click);
  button.setLongClickDetectedHandler(longClickDetected);
  button.setDoubleClickHandler(doubleClick);

  // Search for devices...
  searchDevices();

  mqttReconnectTimer = xTimerCreate("mqttTimer", pdMS_TO_TICKS(2000), pdFALSE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(connectToMqtt));
  wifiReconnectTimer = xTimerCreate("wifiTimer", pdMS_TO_TICKS(2000), pdFALSE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(connectToWifi));

  mqttClient.onConnect(onMqttConnect);
  mqttClient.onDisconnect(onMqttDisconnect);
  mqttClient.onPublish(onMqttPublish);
  mqttClient.setServer(MQTT_HOST, MQTT_PORT);
  //mqttClient.setCredentials("USER", "PASS");

  WiFi.onEvent(WiFiEvent);
  connectToWifi();

}

void loop() {
  
  button.loop();

  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    readSensors();
  }

}

void readSensors() {

  ledAtom[0].setRGB(0,50,0); FastLED.show();

  Serial.println("[Sensor] Requesting temperatures...");
  sensors.requestTemperatures();
  Serial.println("[Sensor] Temperature  request complete.");

  // Loop through each device and print out temperature data...
  for(int i=0;i<numberOfDevices; i++)
  {

    // Search the wire for the address...
    if(sensors.getAddress(tempDeviceAddress, i))
    {

      // Get address string and read the current temperature...
      String strAddress = getAddress(tempDeviceAddress);
      float tempC = sensors.getTempC(tempDeviceAddress);
      
      if(tempC == DEVICE_DISCONNECTED_C) 
      {
        // Report if there is an error reading from the device...
        Serial.println("[Sensor] Error: Could not read temperature data for " + strAddress + ".");
      } else {
      
        // Log the address and reading...
        Serial.println("[Sensor] Temperature for device " + strAddress + " is " + tempC + " C.");
        
        // Publish an MQTT message...
        String topic = MQTT_PREFIX + strAddress + "/temp";
        uint16_t packetIdPub = mqttClient.publish(topic.c_str(), 1, true, String(tempC).c_str());                            
        Serial.printf("[MQTT  ] Publishing on topic %s, payload ", topic.c_str());
        Serial.printf("%.2f, packet ID=", tempC);
        Serial.print(packetIdPub);
        Serial.println(".");
        
      }
      
    } 

  }

  // Return the status LED to standby...
  ledAtom[0].setRGB(0,0,50); FastLED.show();

}

// Convert device address to string...
String getAddress(DeviceAddress deviceAddress)
{
  char address[17];
  char buffer[3];
  for (int i = 0; i < 8; i++) {
    sprintf(buffer, "%02x", deviceAddress[i]);
    address[(i*2)]=buffer[0];
    address[(i*2)+1]=buffer[1];
  }
  address[16] = '\0';
  return String(address);
}

// Button Click Functions...

void click(Button2& btn) {
    Serial.println("[Button] Click");
}

void longClickDetected(Button2& btn) {
    Serial.println("[Button] Long Click");
    searchDevices();
}

void doubleClick(Button2& btn) {
    Serial.println("[Button] Double Click");
}