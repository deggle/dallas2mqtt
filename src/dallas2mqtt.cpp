#include "dallas2mqtt.h"
#include "secrets.h"

#define DEBUG 1  // Set to 1 to enable debugging, or 0 to disable

#ifdef DEBUG
  #define DEBUG_PRINT(...)  Serial.print(__VA_ARGS__)
  #define DEBUG_PRINTLN(...)  Serial.println(__VA_ARGS__)
#else
  #define DEBUG_PRINT(...)
  #define DEBUG_PRINTLN(...)
#endif

const long interval = 30000;          // Publish interval for sensor readings
const bool debug = true;

CRGB ledAtom[1];                      // FastLED array for 1 LED
Button2 button;                       // Button object
OneWire oneWire(ONE_WIRE_BUS);        // OneWire instance on the specified pin
DallasTemperature sensors(&oneWire);  // DallasTemperature instance

AsyncMqttClient mqttClient;           // MQTT client instance
TimerHandle_t mqttReconnectTimer;     // Timer for MQTT reconnect
TimerHandle_t wifiReconnectTimer;     // Timer for Wi-Fi reconnect

int numberOfDevices;                  // Number of DS18B20 sensors found
unsigned long previousMillis = 0;     // Used for timing sensor reads
DeviceAddress deviceAddresses[MAX_SENSORS];  // Array to store sensor addresses
String deviceAddressStrings[MAX_SENSORS];  // Array to store sensor addresses as strings
bool alarmState = false;              // Global alarm state

void connectToWifi() {
  DEBUG_PRINTLN("[WiFi  ] Connecting to Wi-Fi...");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
}

void connectToMqtt() {
  DEBUG_PRINTLN("[MQTT  ] Connecting to MQTT...");
  mqttClient.connect();
}

void WiFiEvent(WiFiEvent_t event) {
  switch(event) {
    case SYSTEM_EVENT_STA_GOT_IP:
      DEBUG_PRINTLN("[WiFi  ] Wi-Fi connected.");
      DEBUG_PRINT("[WiFi  ] IP address: ");
      DEBUG_PRINTLN(WiFi.localIP());
      connectToMqtt();
      break;
    case SYSTEM_EVENT_STA_DISCONNECTED:
      DEBUG_PRINTLN("[WiFi  ] Wi-Fi connection lost.");
      setLED(255,0,0);
      xTimerStop(mqttReconnectTimer, 0); // Avoid reconnecting to MQTT while Wi-Fi is down.
      xTimerStart(wifiReconnectTimer, 0);
      break;
    default:
      break;
  }
}

void onMqttConnect(bool sessionPresent) {
  DEBUG_PRINT("[MQTT  ] Connected to MQTT, Client ID: ");
  DEBUG_PRINTLN(mqttClient.getClientId());

  // Search for devices...
  searchDevices();

  // Let's go right for a reading...
  readSensors();

}

void onMqttDisconnect(AsyncMqttClientDisconnectReason reason) {
  DEBUG_PRINTLN("[MQTT  ] Disconnected from MQTT.");
  setLED(255,0,0);
  if (WiFi.isConnected()) {
    xTimerStart(mqttReconnectTimer, 0);
  }
}

void onMqttPublish(uint16_t packetId) {
  DEBUG_PRINT("[MQTT  ] Publish acknowledged, packet ID=" + String(packetId) + ".\n");
}

void onMqttSubscribe(uint16_t packetId, uint8_t qos) {
  DEBUG_PRINT("[MQTT  ] Subscribe acknowledged, packet ID=");
  DEBUG_PRINT(packetId);
  DEBUG_PRINT(", QoS ");
  DEBUG_PRINT(qos);
  DEBUG_PRINTLN(".");
}

void onMqttMessage(char* topic, char* payload, AsyncMqttClientMessageProperties properties, size_t len, size_t index, size_t total) {
  DEBUG_PRINT("[MQTT  ] Message received on topic ");
  DEBUG_PRINT(topic);
  DEBUG_PRINT(", payload ");
  DEBUG_PRINT(String(payload));
  DEBUG_PRINTLN(".");

  // Is this a low/high alarm set point?
  if (String(topic).endsWith("/set-low")) {

    // Get the address of the device to set...
    String deviceAddress = String(topic);
    deviceAddress.replace(MQTT_PREFIX, "");
    deviceAddress.replace("/set-low","");
    deviceAddress.toLowerCase();

    // Convert payload to int once and store it in a variable for reuse
    int lowSetpoint = String(payload).toInt();
    DEBUG_PRINTLN("[Sensor] Setting low alarm setpoint for device " + deviceAddress + " to " + String(lowSetpoint) + " °C.");
    setAlarmTemp(true, deviceAddress, lowSetpoint);
    DEBUG_PRINTLN("[Sensor] Low alarm setpoint for device " + deviceAddress + " successfully set to " + String(lowSetpoint) + " °C.");

  } else if (String(topic).endsWith("/set-high")) {

    // Get the address of the device to set...
    String deviceAddress = String(topic);
    deviceAddress.replace(MQTT_PREFIX, "");
    deviceAddress.replace("/set-high","");
    deviceAddress.toLowerCase();

    // Convert payload to int once and store it in a variable for reuse
    int highSetpoint = String(payload).toInt();
    DEBUG_PRINTLN("[Sensor] Setting high alarm setpoint for device " + deviceAddress + " to " + String(highSetpoint) + " °C.");
    setAlarmTemp(false, deviceAddress, highSetpoint);
    DEBUG_PRINTLN("[Sensor] High alarm setpoint for device " + deviceAddress + " successfully set to " + String(highSetpoint) + " °C.");
    
  }

}

void setAlarmTemp(bool isLow, String deviceAddressString, int8_t temp) {
    // Create a DeviceAddress array to store the converted address
    DeviceAddress deviceAddress;

    // Convert the string representation to a DeviceAddress (byte array)
    if (!stringToDeviceAddress(deviceAddressString, deviceAddress)) {
        // Handle invalid address format
        DEBUG_PRINTLN("[Error] Invalid device address string: " + deviceAddressString);
        return;  // Exit the function if conversion fails
    }

    // Set the appropriate alarm temperature (low or high)
    if (isLow) {
        sensors.setLowAlarmTemp(deviceAddress, temp);
        DEBUG_PRINTLN("[Sensor] Low alarm set for device: " + deviceAddressString + " at " + String(temp) + " °C.");
        publishPayload(MQTT_PREFIX + deviceAddressString + "/alarm-low", String(sensors.getLowAlarmTemp(deviceAddress)));
    } else {
        sensors.setHighAlarmTemp(deviceAddress, temp);
        DEBUG_PRINTLN("[Sensor] High alarm set for device: " + deviceAddressString + " at " + String(temp) + " °C.");
        publishPayload(MQTT_PREFIX + deviceAddressString + "/alarm-high", String(sensors.getHighAlarmTemp(deviceAddress)));
    }

}

void searchDevices() {

  // Reset arrays...
 // resetArrays();

  DEBUG_PRINTLN("[Sensor] Searching for 1-wire devices...");

  // Grab a count of devices on the wire
  sensors.begin();
  sensors.begin(); // For some reason, this only works when called twice!?

  numberOfDevices = sensors.getDeviceCount();
  int numberOfDS18 = sensors.getDS18Count();

  DEBUG_PRINT("[Sensor] Found ");
  DEBUG_PRINT(numberOfDevices, DEC);
  DEBUG_PRINT(" devices (of which ");
  DEBUG_PRINT(numberOfDS18, DEC);
  DEBUG_PRINTLN(" are DS18).");

  // Loop through each device, print out address and set resolution...
  for(int i=0;i<numberOfDevices; i++)
  {
    if(sensors.getAddress(deviceAddresses[i], i))
    {

      String addressString = addressToString(deviceAddresses[i]);
      deviceAddressStrings[i] = addressString;

      DEBUG_PRINT("[Sensor] Found device ");
      DEBUG_PRINT(i, DEC);
      DEBUG_PRINT(" with address " + addressString);
      DEBUG_PRINTLN("."); 

      DEBUG_PRINT("[Sensor] Setting resolution to ");
      DEBUG_PRINT(TEMPERATURE_PRECISION, DEC);
      DEBUG_PRINTLN(" bits.");
      sensors.setResolution(deviceAddresses[i], TEMPERATURE_PRECISION);
      
      DEBUG_PRINT("[Sensor] Resolution currently set to: ");
      DEBUG_PRINT(sensors.getResolution(deviceAddresses[i]), DEC); 
      DEBUG_PRINTLN();

      // Subscribing to alarm set temp...
      mqttSubscribe(MQTT_PREFIX + addressString + "/set-low");
      mqttSubscribe(MQTT_PREFIX + addressString + "/set-high");

      // Send the HA discovery packet if desired...
      if (HA_DISCOVERY) {
        publishDiscovery(addressString);
      }

      // Publish alarm states...
      publishPayload(MQTT_PREFIX + addressString + "/alarm-low", String(sensors.getLowAlarmTemp(deviceAddresses[i])));
      publishPayload(MQTT_PREFIX + addressString + "/alarm-high", String(sensors.getHighAlarmTemp(deviceAddresses[i])));

    }else{
      DEBUG_PRINT("[Sensor] Found ghost device at ");
      DEBUG_PRINT(i, DEC);
      DEBUG_PRINT(" but could not detect address (check power and cabling).");
    }
  }
    
}

// void resetArrays() {
//   for(int i = 0; i < MAX_SENSORS; i++) {
//     // Reset each byte of the DeviceAddress array to 0
//     for (int j = 0; j < 8; j++) {
//       deviceAddresses[i][j] = 0;  // or you can use 0xFF if you prefer
//     }
//   }
// }


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
  DEBUG_PRINTLN();
  DEBUG_PRINTLN();

  button.begin(BUTTON_PIN);
  button.setLongClickTime(1000);
  button.setDoubleClickTime(400);

  DEBUG_PRINTLN("[Button] DoubleClick Time: " + String(button.getDoubleClickTime()) + "ms");
  DEBUG_PRINTLN("[Button] Longpress Time: " + String(button.getLongClickTime()) + "ms");
  
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
  mqttClient.setCredentials(MQTT_USERNAME, MQTT_PASSWORD);

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

  DEBUG_PRINTLN("[Sensor] Requesting temperatures...");
  sensors.requestTemperatures();
  DEBUG_PRINTLN("[Sensor] Temperature  request complete.");
  
  // Reset the alarm...
  alarmState = false;

  // Loop through each device and print out temperature data...
  for(int i=0;i<numberOfDevices; i++)
  {

      DeviceAddress &deviceAddress = deviceAddresses[i];
      String &deviceAddressString = deviceAddressStrings[i];

      // Read the current temperature...
      float tempC = sensors.getTempC(deviceAddress);
      tempC = round(tempC*10)/10;

      if(tempC == DEVICE_DISCONNECTED_C) 
      {
        // Report if there is an error reading from the device...
        DEBUG_PRINTLN("[Sensor] Error: Could not read temperature data for " + deviceAddressString + ".");
      } else {
      
        // Log the address and reading...
        DEBUG_PRINTLN("[Sensor] Temperature for device " + deviceAddressString + " is " + tempC + " C.");
        
        // Check the alarm...
        bool isAlarming = sensors.hasAlarm(deviceAddress);

        // Set the global alarm state (for the LED)...
        alarmState = alarmState | isAlarming;

        // Publish an MQTT message...
        publishPayload(MQTT_PREFIX + deviceAddressString + "/temp", String(tempC));
        publishPayload(MQTT_PREFIX + deviceAddressString + "/alarm", String(isAlarming));
      
    } 

  }

  // Return the status LED to standby...
  setLED(0,0,0);

}

void mqttSubscribe(String topic) {
      DEBUG_PRINT("[MQTT  ] Subscribing to ");
      DEBUG_PRINT(topic.c_str());
      DEBUG_PRINTLN(".");
      mqttClient.subscribe(topic.c_str(), 2);
}

void publishPayload(String topic, String payload) {
    uint16_t packetIdPub = mqttClient.publish(topic.c_str(), 1, true, payload.c_str());
    DEBUG_PRINT("[MQTT  ] Publishing on topic " + topic + ", payload " + payload + ", packet ID=" + String(packetIdPub) + ".\n");
}

void publishDiscovery(String device) {

  // Define the device object that will be shared across all entities
  String deviceInfo = "{\"identifiers\": [\"" + String(HA_PREFIX) + device + "\"], \"name\": \"" + device + "\", \"model\": \"Temperature Sensor\", \"manufacturer\": \"" + String(HA_MANUFACTURER) + "\"}";

  // Send HA discovery packet for temperature sensor (1 decimal place)...
  String discoveryTopic = "homeassistant/sensor/" + String(HA_PREFIX) + device + "/config";
  String discoveryPayload = "{\"device_class\": \"temperature\",\"unique_id\": \"" + String(HA_PREFIX) + device + "-temp\",\"name\": \"Temperature\",\"state_topic\": \"" + MQTT_PREFIX + device + "/temp\",\"unit_of_measurement\": \"°C\",\"value_template\": \"{{ value | round(1) }}\", \"device\":" + deviceInfo + "}";
  publishPayload(discoveryTopic, discoveryPayload);

  // Send HA discovery packet for alarm setpoints (0 decimal places)...
  discoveryTopic = "homeassistant/sensor/" + String(HA_PREFIX) + device + "-alarm-low/config";
  discoveryPayload = "{\"device_class\": \"temperature\",\"unique_id\": \"" + String(HA_PREFIX) + device + "-alarm-low\",\"name\": \"Low Alarm Setpoint\",\"state_topic\": \"" + MQTT_PREFIX + device + "/alarm-low\",\"unit_of_measurement\": \"°C\",\"value_template\": \"{{ value | round(0) }}\", \"device\":" + deviceInfo + "}";
  publishPayload(discoveryTopic, discoveryPayload);
  
  discoveryTopic = "homeassistant/sensor/" + String(HA_PREFIX) + device + "-alarm-high/config";
  discoveryPayload = "{\"device_class\": \"temperature\",\"unique_id\": \"" + String(HA_PREFIX) + device + "-alarm-high\",\"name\": \"High Alarm Setpoint\",\"state_topic\": \"" + MQTT_PREFIX + device + "/alarm-high\",\"unit_of_measurement\": \"°C\",\"value_template\": \"{{ value | round(0) }}\", \"device\":" + deviceInfo + "}";
  publishPayload(discoveryTopic, discoveryPayload);

  // Send HA discovery packet for alarm state...
  discoveryTopic = "homeassistant/binary_sensor/" + String(HA_PREFIX) + device + "/config";
  discoveryPayload = "{\"device_class\": \"safety\",\"unique_id\": \"" + String(HA_PREFIX) + device + "-alarm\",\"name\": \"Alarm\",\"state_topic\": \"" + MQTT_PREFIX + device + "/alarm\", \"payload_off\": \"0\", \"payload_on\": \"1\", \"device\":" + deviceInfo + "}";
  publishPayload(discoveryTopic, discoveryPayload);

  // Send HA discovery packet for setpoint inputs (low setpoint, 0 decimal places)...
  discoveryTopic = "homeassistant/number/" + String(HA_PREFIX) + device + "-set-low/config";
  discoveryPayload = "{\"device_class\": \"temperature\",\"unique_id\": \"" + String(HA_PREFIX) + device + "-set-low\",\"name\": \"Low Setpoint\",\"command_topic\": \"" + MQTT_PREFIX + device + "/set-low\",\"state_topic\": \"" + MQTT_PREFIX + device + "/alarm-low\",\"unit_of_measurement\": \"°C\",\"min\": 0,\"max\": 100,\"step\": 1, \"value_template\": \"{{ value | round(0) }}\", \"device\":" + deviceInfo + "}";
  publishPayload(discoveryTopic, discoveryPayload);

  // Send HA discovery packet for setpoint inputs (high setpoint, 0 decimal places)...
  discoveryTopic = "homeassistant/number/" + String(HA_PREFIX) + device + "-set-high/config";
  discoveryPayload = "{\"device_class\": \"temperature\",\"unique_id\": \"" + String(HA_PREFIX) + device + "-set-high\",\"name\": \"High Setpoint\",\"command_topic\": \"" + MQTT_PREFIX + device + "/set-high\",\"state_topic\": \"" + MQTT_PREFIX + device + "/alarm-high\",\"unit_of_measurement\": \"°C\",\"min\": 0,\"max\": 100,\"step\": 1, \"value_template\": \"{{ value | round(0) }}\", \"device\":" + deviceInfo + "}";
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
    DEBUG_PRINTLN("[Button] Click");
    readSensors();
}

void longClickDetected(Button2& btn) {
    DEBUG_PRINTLN("[Button] Long Click");
    //searchDevices();
    ESP.restart();
}

void doubleClick(Button2& btn) {
    DEBUG_PRINTLN("[Button] Double Click");
}

// A helper function to convert a hexadecimal string to DeviceAddress (byte array)
bool stringToDeviceAddress(const String &addressString, DeviceAddress &deviceAddress) {
    if (addressString.length() != 16) {
        return false;  // Invalid address length
    }

    for (int i = 0; i < 8; i++) {
        String byteString = addressString.substring(i * 2, i * 2 + 2);
        deviceAddress[i] = (uint8_t) strtol(byteString.c_str(), NULL, 16); // Convert hex string to byte
    }

    return true;
}