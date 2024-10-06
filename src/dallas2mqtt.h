#ifndef DALLAS2MQTT_H
#define DALLAS2MQTT_H

#include <Arduino.h>              // Arduino core for ESP32
#include <WiFi.h>                 // WiFi Library for ESP32
#include <Secrets.h>              // Include your secrets like SSID & password
#include <AsyncMqttClient.h>      // Asynchronous MQTT client library
#include <OneWire.h>              // For 1-Wire communication with sensors
#include <DallasTemperature.h>    // For DallasTemperature sensor library
#include <Button2.h>              // Button handling library
#define FASTLED_INTERNAL
#include <FastLED.h>              // FastLED for controlling RGB LED

extern "C" {
  #include "freertos/FreeRTOS.h"
  #include "freertos/timers.h"    // FreeRTOS timers for reconnect handling
}

#define MAX_SENSORS 10            // Max number of DS18B20 sensors
#define ONE_WIRE_BUS 26           // Pin where the DS18B20 is connected
#define TEMPERATURE_PRECISION 12   // Set temperature precision to 12 bits
#define PIN_LEDATOM 27            // Pin for Atom's LED
#define BUTTON_PIN 39             // Button pin on Atom Matrix

// Function declarations
void connectToWifi();
void connectToMqtt();
void WiFiEvent(WiFiEvent_t event);
void onMqttConnect(bool sessionPresent);
void onMqttDisconnect(AsyncMqttClientDisconnectReason reason);
void onMqttPublish(uint16_t packetId);
void onMqttSubscribe(uint16_t packetId, uint8_t qos);
void onMqttMessage(char* topic, char* payload, AsyncMqttClientMessageProperties properties, size_t len, size_t index, size_t total);
void setAlarmTemp(bool isLow, String deviceAddress, int8_t temp);
void searchDevices();
void resetArrays();
void setLED(uint8_t nr, uint8_t ng, uint8_t nb);
void readSensors();
void mqttSubscribe(String topic);
void publishPayload(String topic, String payload);
void publishDiscovery(String device);
String addressToString(DeviceAddress deviceAddress);
void click(Button2& btn);
void longClickDetected(Button2& btn);
void doubleClick(Button2& btn);
bool stringToDeviceAddress(const String &addressString, DeviceAddress &deviceAddress);

#endif // DALLAS2MQTT_H
