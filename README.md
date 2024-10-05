# Dallas2MQTT Environmental Monitoring

Laboratory environmental monitoring project for Islestone.

## Hardware
This code is designed to run on the [M5Stack Atom Lite](https://shop.m5stack.com/collections/atom-series/products/atom-lite-esp32-development-kit) device, a development microprocessor board based on the ESP32 chipset. This device has been selected for the following reasons:

- It is a compact device with 'production ready' design.
- Simple to power via USB-C or 5V supply.
- Includes USB<>RS232 chip for easy flashing.
- Includes an RGB LED for visual status.
- Grove connector can be used for attaching sensors.
- [PoE power supply](https://shop.m5stack.com/products/atom-poe-kit-with-w5500-hy601742e) (with ethernet connectivity) is an option.

![M5Stack Atom Lite](/images/m5stack-atom-lite.png)

The client device is paired with low cost DS18B20, digital thermometer with fairly precise readings (±0.5°C over much of the range) and can give up to 12 bits of precision from the onboard digital-to-analog converter. The DS18B20 also includes high and low alarm set-points that are stored in the internal EEPROM.

The DS18B20 sensors can be attached via `GPIO 26` using the Grove connector. Once the client devices are flashed, multiple temperature probes can be attached to a device.

Each DS18B20 contains a unique 64-bit address and data will be sent via the MQTT messaging platform using these addresses. Therefore a temperature probe can be attached to any client device and readings will be associated with that sensor/probe. This is likely helpful for tacking probes though calibration processes.


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
#define MQTT_PREFIX "prefix/env/"
```
For each attached sensor, the following messages will be published **on boot**:

```
prefix/env/28aaebfa1a130268/alarm-low        The sensors alarm high/max set point.
prefix/env/28aaebfa1a130268/alarm-high       The sensors alarm low/min set point.
```

For each attached sensor, the following messages will be published after boot and **periodically**:

```
prefix/env/28aaebfa1a130268/temp        The temperature reading.
prefix/env/28aaebfa1a130268/alarm       Alarm state (1/0).
```

## Alarm Set-Point
Each sensor can be configured with low and high alarm set-points. These are stored within EEPROM inside the sensor (so will operate regardless to which device the temperature probe is attached). When the temperature rises above this point the alarm MQTT topic will be updated and the LED on the client device will turn RED.

The low and high set-points can be set by sending an MQTT payload to the following topics:

```
prefix/env/28aaebfa1a130268/set-low         Sets the alarm low(min) set-point.
prefix/env/28aaebfa1a130268/set-high        Sets the alarm high(max) set-point.
```
**Note:** Set-points are stored within the probe devices, but could be periodically set via MQTT to ensure they are operating at the desired levels. The required set-points for each sensor address could be sent to the MQTT broker with a `retain=true` flag resulting in the client devices receiving this message immediately at startup. Set-points will only be written to EEPROM when changed, to avoid read/write wear.

## Home Assistant Integration

The devices can be configured to automatically setup entities in Home Assistant for displaying the live temperature, alarm state and set-points. Each probe will create 4 entities that can be included on dashboards:

![Home Assistant discovery entities.](/images/homeassistant.png)

The discovery option must be enabled in the configuration...
```
#define HA_DISCOVERY true
#define HA_PREFIX "prefix-env-"
```

The following discovery payloads will be sent on boot...
```
homeassistant/sensor/prefix-env-28aaebfa1a130268/config
homeassistant/binary_sensor/prefix-env-28aaebfa1a130268/config
homeassistant/sensor/prefix-env-28aaebfa1a130268-alarm-low/config
homeassistant/sensor/prefix-env-28aaebfa1a130268-alarm-high/config
```

## Development Roadmap

Add support for the W5500 ethernet chip, as used in the [M5Stack PoE add-on](https://shop.m5stack.com/products/atom-poe-kit-with-w5500-hy601742e). This will provide power 'out of the box' but it's daft to use PoE and then send data over WiFi. Coding to use ethernet (or fall back to WiFi) would improve stability and ease power distribution.

Copyright © Tim Alston