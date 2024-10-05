#include "dallas2mqtt.h"
#include "Secrets.h"

CRGB ledAtom[1];                      // FastLED array for 1 LED
Button2 button;                       // Button object
OneWire oneWire(ONE_WIRE_BUS);        // OneWire instance on the specified pin
DallasTemperature sensors(&oneWire);  // DallasTemperature instance

AsyncMqttClient mqttClient;           // MQTT client instance
TimerHandle_t mqttReconnectTimer;     // Timer for MQTT reconnect
TimerHandle_t wifiReconnectTimer;     // Timer for Wi-Fi reconnect

int numberOfDevices;                  // Number of DS18B20 sensors found
unsigned long previousMillis = 0;     // Used for timing sensor reads
const long interval = 10000;          // Publish interval for sensor readings
DeviceAddress tempDeviceAddress;      // Address for current DS18B20 sensor
String deviceAddresses[MAX_SENSORS];  // Array to store sensor addresses
bool alarmState = false;              // Global alarm state

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
      setLED(255,0,0);
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

  // Search for devices...
  searchDevices();

  // Let's go right for a reading...
  readSensors();

}

void onMqttDisconnect(AsyncMqttClientDisconnectReason reason) {
  Serial.println("[MQTT  ] Disconnected from MQTT.");
  setLED(255,0,0);
  if (WiFi.isConnected()) {
    xTimerStart(mqttReconnectTimer, 0);
  }
}

void onMqttPublish(uint16_t packetId) {
  Serial.print("[MQTT  ] Publish acknowledged, packet ID=" + String(packetId) + ".\n");
}

void onMqttSubscribe(uint16_t packetId, uint8_t qos) {
  Serial.print("[MQTT  ] Subscribe acknowledged, packet ID=");
  Serial.print(packetId);
  Serial.print(", QoS ");
  Serial.print(qos);
  Serial.println(".");
}

void onMqttMessage(char* topic, char* payload, AsyncMqttClientMessageProperties properties, size_t len, size_t index, size_t total) {
  Serial.print("[MQTT  ] Message received on topic ");
  Serial.print(topic);
  Serial.print(", payload ");
  Serial.print(String(payload));
  Serial.println(".");

  // Is this a low/high alarm set point?
  if (String(topic).endsWith("/set-low")) {

    // Get the address of the device to set...
    String deviceAddress = String(topic);
    deviceAddress.replace(MQTT_PREFIX, "");
    deviceAddress.replace("/set-low","");
    deviceAddress.toLowerCase();

    Serial.println("[Sensor] Setting low alarm setpoint for device " + deviceAddress + " to " + String(payload) + " C.");

    setAlarmTemp(true, deviceAddress, String(payload).toInt());
    
    Serial.println("[Sensor] Low alarm setpoint for device " + deviceAddress + " set to " +  String(payload).toInt() + ".");
  
  } else if (String(topic).endsWith("/set-high")) {

    // Get the address of the device to set...
    String deviceAddress = String(topic);
    deviceAddress.replace(MQTT_PREFIX, "");
    deviceAddress.replace("/set-high","");
    deviceAddress.toLowerCase();

    Serial.println("[Sensor] Setting high alarm setpoint for device " + deviceAddress + " to " + String(payload) + " C.");

    setAlarmTemp(false, deviceAddress, String(payload).toInt());

    Serial.println("[Sensor] High alarm setpoint for device " + deviceAddress + " set to " +  String(payload).toInt() + ".");
    
  }

}

void setAlarmTemp(bool isLow, String deviceAddress, int8_t temp) {

    // Loop through each device and print out temperature data...
    for(int i=0;i<numberOfDevices; i++)
    {

      // Search the wire for the address...
      if(sensors.getAddress(tempDeviceAddress, i))
      {

        // Check for mismatch, and re-boot if needed...
        String strAddress = addressToString(tempDeviceAddress);
        if (strAddress != deviceAddresses[i]) { ESP.restart(); }

        if (isLow) {
          sensors.setLowAlarmTemp(tempDeviceAddress, temp);
        } else {
          sensors.setHighAlarmTemp(tempDeviceAddress, temp);
        }

      }

    }

}

void searchDevices() {

  // Reset arrays...
  resetArrays();

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
      deviceAddresses[i] = addressToString(tempDeviceAddress);

      Serial.print("[Sensor] Found device ");
      Serial.print(i, DEC);
      Serial.print(" with address " + deviceAddresses[i]);
      Serial.println("."); 

      Serial.print("[Sensor] Setting resolution to ");
      Serial.print(TEMPERATURE_PRECISION, DEC);
      Serial.println(" bits.");
      sensors.setResolution(tempDeviceAddress, TEMPERATURE_PRECISION);
      
      Serial.print("[Sensor] Resolution currently set to: ");
      Serial.print(sensors.getResolution(tempDeviceAddress), DEC); 
      Serial.println();

      // Subscribing to alarm set temp...
      mqttSubscribe(MQTT_PREFIX + deviceAddresses[i] + "/set-low");
      mqttSubscribe(MQTT_PREFIX + deviceAddresses[i] + "/set-high");

      // Send the HA discovery packet if desired...
      if (HA_DISCOVERY) {
        publishDiscovery(deviceAddresses[i]);
      }

      // Publish alarm states...
      publishPayload(MQTT_PREFIX + deviceAddresses[i] + "/alarm-low", String(sensors.getLowAlarmTemp(tempDeviceAddress)));
      publishPayload(MQTT_PREFIX + deviceAddresses[i] + "/alarm-high", String(sensors.getHighAlarmTemp(tempDeviceAddress)));

    }else{
      Serial.print("[Sensor] Found ghost device at ");
      Serial.print(i, DEC);
      Serial.print(" but could not detect address (check power and cabling).");
    }
  }
    
}

void resetArrays() {
  for(int i = 0; i < MAX_SENSORS; i++) {
    deviceAddresses[i] = "";
  }
}

void setLED(uint8_t nr, uint8_t ng, uint8_t nb) {
  
  // Are we in standby?
  if (nr == 0 && ng == 0 && nb == 0) {
    
    // Is there an alarm?
    if (alarmState) {
      nr = 255;
    } else {
      ng = 50;
    }

  }

  ledAtom[0].setRGB(nr,ng,nb);
  FastLED.show();

}

void setup() {

  FastLED.addLeds<NEOPIXEL, PIN_LEDATOM>(ledAtom, 1);
  setLED(255,0,0);

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

  mqttReconnectTimer = xTimerCreate("mqttTimer", pdMS_TO_TICKS(2000), pdFALSE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(connectToMqtt));
  wifiReconnectTimer = xTimerCreate("wifiTimer", pdMS_TO_TICKS(2000), pdFALSE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(connectToWifi));

  mqttClient.onConnect(onMqttConnect);
  mqttClient.onDisconnect(onMqttDisconnect);
  mqttClient.onPublish(onMqttPublish);
  mqttClient.onSubscribe(onMqttSubscribe);
  mqttClient.onMessage(onMqttMessage);
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

  setLED(0,0,50);

  Serial.println("[Sensor] Requesting temperatures...");
  sensors.requestTemperatures();
  Serial.println("[Sensor] Temperature  request complete.");
  
  // Reset the alarm...
  alarmState = false;

  // Loop through each device and print out temperature data...
  for(int i=0;i<numberOfDevices; i++)
  {

    // Search the wire for the address...
    if(sensors.getAddress(tempDeviceAddress, i))
    {

      // Check for mismatch, and re-boot if needed...
      String strAddress = addressToString(tempDeviceAddress);
      if (strAddress != deviceAddresses[i]) { ESP.restart(); }

      // Read the current temperature...
      float tempC = sensors.getTempC(tempDeviceAddress);
      tempC = round(tempC*10)/10;

      if(tempC == DEVICE_DISCONNECTED_C) 
      {
        // Report if there is an error reading from the device...
        Serial.println("[Sensor] Error: Could not read temperature data for " + strAddress + ".");
      } else {
      
        // Log the address and reading...
        Serial.println("[Sensor] Temperature for device " + strAddress + " is " + tempC + " C.");
        
        // Check the alarm...
        bool isAlarming = sensors.hasAlarm(tempDeviceAddress);

        // Set the global alarm state (for the LED)...
        alarmState = alarmState | isAlarming;

        // Publish an MQTT message...
        publishPayload(MQTT_PREFIX + strAddress + "/temp", String(tempC));
        publishPayload(MQTT_PREFIX + strAddress + "/alarm", String(isAlarming));

      }
      
    } 

  }

  // Return the status LED to standby...
  setLED(0,0,0);

}

void mqttSubscribe(String topic) {
      Serial.print("[MQTT  ] Subscribing to ");
      Serial.print(topic.c_str());
      Serial.println(".");
      mqttClient.subscribe(topic.c_str(), 2);
}

void publishPayload(String topic, String payload) {
    uint16_t packetIdPub = mqttClient.publish(topic.c_str(), 1, true, payload.c_str());
    Serial.print("[MQTT  ] Publishing on topic " + topic + ", payload " + payload + ", packet ID=" + String(packetIdPub) + ".\n");
}

void publishDiscovery(String device) {

  // Send HA discovery packet for temperature sensor...
  String discoveryTopic = "homeassistant/sensor/" + String(HA_PREFIX) + device + "/config";
  String discoveryPayload = "{\"device_class\": \"temperature\",\"unique_id\": \"" + String(HA_PREFIX) + device + "-temp\",\"name\": \"Temperature (" + device + ")\",\"state_topic\": \"" + MQTT_PREFIX + device + "/temp\",\"unit_of_measurement\": \"°C\",\"value_template\": \"{{ value | round(1) }}\"}";
  publishPayload(discoveryTopic, discoveryPayload);

  // Send HA discovery packet for alarm setpoints...
  discoveryTopic = "homeassistant/sensor/" + String(HA_PREFIX) + device + "-alarm-low/config";
  discoveryPayload = "{\"device_class\": \"temperature\",\"unique_id\": \"" + String(HA_PREFIX) + device + "-alarm-low\",\"name\": \"Low Alarm Setpoint (" + device + ")\",\"state_topic\": \"" + MQTT_PREFIX + device + "/alarm-low\",\"unit_of_measurement\": \"°C\"}";
  publishPayload(discoveryTopic, discoveryPayload);
  discoveryTopic = "homeassistant/sensor/" + String(HA_PREFIX) + device + "-alarm-high/config";
  discoveryPayload = "{\"device_class\": \"temperature\",\"unique_id\": \"" + String(HA_PREFIX) + device + "-alarm-high\",\"name\": \"High Alarm Setpoint (" + device + ")\",\"state_topic\": \"" + MQTT_PREFIX + device + "/alarm-high\",\"unit_of_measurement\": \"°C\"}";
  publishPayload(discoveryTopic, discoveryPayload);

  // Send HA discovery packet for alarm state...
  discoveryTopic = "homeassistant/binary_sensor/" + String(HA_PREFIX) + device + "/config";
  discoveryPayload = "{\"device_class\": \"safety\",\"unique_id\": \"" + String(HA_PREFIX) + device + "-alarm\",\"name\": \"Alarm (" + device + ")\",\"state_topic\": \"" + MQTT_PREFIX + device + "/alarm\", \"payload_off\": \"0\", \"payload_on\": \"1\"}";
  publishPayload(discoveryTopic, discoveryPayload);

}

// Convert device address to string...
String addressToString(DeviceAddress deviceAddress)
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
    readSensors();
}

void longClickDetected(Button2& btn) {
    Serial.println("[Button] Long Click");
    //searchDevices();
    ESP.restart();
}

void doubleClick(Button2& btn) {
    Serial.println("[Button] Double Click");
}