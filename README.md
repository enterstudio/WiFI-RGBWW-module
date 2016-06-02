# H801 Firmware for OpenHAB

Some time ago, the [ESP8266-based H801](http://chaozlabs.blogspot.de/2015/08/esp8266-in-wild-wifi-led-controller-hack.html) module
appeared on AliExpress and became a nice IoT hacker playground.

This project is an ESP8266 firmware for the H801 module, compiled with the Arduino IDE.
It provides the following features:

 * Real-time control via MQTT
 * Easy WiFi setup with [WiFiManager](https://github.com/tzapu/WiFiManager)
 * Smooth fading between colors
 * Colorwheel fading with (almost) constant brightness and variable speed
 * Direct OpenHAB HSV support
 * PWM of HSV, RGB, W1 and W2 channels based on MQTT topics
 * Exposes on-board LEDs and reboot as MQTT topics
 * Use of the more-robust SDK PWM (see below compilation instructions!)
 * Correct CIE-1931 brightness mapping

Open tasks / known issues:
 * Piggy-back MQTT server address into WiFiManager dialog
 * Derive `MQTT_ID` from MAC address / hostname to allow multiple independent devices
 * Refactor ugly MQTT client code
 * Rewrite OpenHAB configuration files
 * Add support for infrared remotes

This is a heavily-modified fork of [4ndreas' sketch](https://github.com/4ndreas/WiFI-RGBWW-module).

# Compilation

You will need to check out the following libraries into `~/Arduino/libraries/` to build the code:

 * https://github.com/Imroy/pubsubclient (**not** the PubSubClient suggested by Arduino IDE)
 * https://github.com/ratkins/RGBConverter
 * https://github.com/tzapu/WiFiManager

Due to [some](https://github.com/esp8266/Arduino/issues/836)
[issues](https://github.com/esp8266/Arduino/issues/1654), this sketch is using
the SDK PWM instead of the built-in Arduino `analogWrite()` PWM. To enable the
SDK PWM, you need to download the git version of the [ESP8266 Arduino
code](https://github.com/esp8266/Arduino.git) and edit the `platform.txt` as follows:

Find the `compiler.c.elf.libs=...` line, and append to its end: ` -lpwm`

Alternatively, you can remove the `#define ESPRESSIF_PWM 1` line in the sketch.
However, this variant isn't well tested.

# MQTT Topics

All topics have the same format. Currently, they all begin with `/openHAB/Bedroom_ESP/` and have the individual channel following:

 * `Reset` - publishing this will perform a reboot of the ESP
 * `RGB` - semicolon-separated triple for the RGB channels with range 0..100, i.e. `100;100;100` for white
 * `HSV` - comma-separated H,S,V values as used by OpenHAB (H: 0..360, S/V: 0..100)
 * `Fader` - color cycling with varying speed, range 0..100. Will be turned off (and re-published) when HSV or RGB are written to
 * `SW1` - W1 output, range 0..100
 * `SW2` - W2 output, range 0..100
 * `LED1` - green on-board LED, range 0..1
 * `LED2` - red on-board LED, range 0..1


# License

The CIE-9131 table generator is MIT-licensed code from http://jared.geek.nz/2013/feb/linear-led-pwm


