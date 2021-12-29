# ESP32-Environmental

Laboratory environmental monitoring project for Preventx.

## Hardware
This code is designed to run on the [M5Stack Atom Lite](https://shop.m5stack.com/collections/atom-series/products/atom-lite-esp32-development-kit) device, a development microprocessor board based on the ESP32 chipset. This device has been selected for the following reasons:

- It is a compact device with 'production ready' design.
- Simple to power via USB-C or 5V supply.
- Includes an RGB LED for visual status.
- Grove connector can be used for attaching sensors.
- [PoE power supply](https://shop.m5stack.com/products/atom-poe-kit-with-w5500-hy601742e) (with ethernet connectivity) is an option.

![Grove connector with 4.7k pull-up resistor.](/images/m5stack-atom-lite.png)

The client device is paired with low cost DS18B20, digital thermometer with fairly precise readings (±0.5°C over much of the range) and can give up to 12 bits of precision from the onboard digital-to-analog converter. 

The DS18B20 sensors can be attached via `GPIO 26` using the Grove connector. Once the client devices are flashed, multiple temperature probes can be attached to a device.

Each DS18B20 contains a unique 64-bit address and data will be sent via the MQTT messaging platform using these addresses. Therefore a temperature probe can be attached to any client device and readings will be associated with that sensor/probe.

**Note:** This project does not use the DS18B20 internal alarm function, however this could be updated and would enable alarm set-points to be stored in the sensor probe.

## Electronic / Wiring

Due to the hardware selection, the electronics for this project are basic.

The only additional component required is a 4.7kΩ pull-up resistor, to be connected between the DS18B20 data line and VCC supply voltage. For a single sensor, this can be easily included in-line when attaching the Grove connector to provide a professional finish.

![Grove connector with 4.7k pull-up resistor.](/images/grove-pull-up.jpg)

Where multiple sensors are to be connected to a single client device, there should only be a single resistor (not one per sensor) and when using off-the-shelf Grove connectors this may be problematic. Tolerances may allow for two sensors to be attached (thereby halving the resistance) and still function correctly.

> To avoid over-complication, and risk electronic issues, it may be better to wire specific sensor bundles (e.g. three sensors into one Grove connector, with one pull-up resistor) as required.

## MQTT Topics

All topics are based on the ID of the attached DS18B20 devices, allowing sensor traceability. This also allows sensors to be moved from one client device to another, without any configuration required (power cycle, or long-press the button to re-scan attached devices).

The MQTT settings and prefix used for all topics is defined in ```Secrets.h``` header:

```c
#define MQTT_HOST "IP or Host Name"
#define MQTT_PORT 1883
#define MQTT_PREFIX "px/env/"
```

For each attached sensor, the following messages will be published:

```
px/env/28aaebfa1a130268/temp        The temperature reading.
px/env/28aaebfa1a130268/alarm       Alarm state (1/0).
```


## Alarm Set-Point
Each sensor can be configured with an alarm set-point. When the temperature rises above this point the alarm MQTT topic will be sent and the LED on the client device will turn RED.

```
px/env/28aaebfa1a130268/set         Sets the alarm (max) set-point.
```
**Note:** Set-points are *not* stored on the client devices, however can be set at startup via MQTT using retained messages. The required set-point for each sensor address should be sent to the MQTT broker with a `retain=true` flag resulting in the client  devices receiving this message immediately at startup. These retained messages should be sent periodically (e.g. every 300 seconds) in case the MQTT broker restarts (unless persistent storage is configured).
