#include <FS.h>

// for HSV to RGB
#include <RGBConverter.h>

// for web server based auto-update
#include <ESP8266HTTPUpdateServer.h>

// for MQTT subscription
#include <MQTT.h>
#include <PubSubClient.h>

// for WiFi and config
#include <ESP8266WiFi.h>
#include <WiFiManager.h>
#include <ArduinoJson.h>

#define WIFI_NETWORK "H801-Config"
#define WIFI_PASSWORD "secret password"

#define NEOPIXEL 1
#ifdef NEOPIXEL
#include <NeoPixelBus.h>
#endif // NEOPIXEL

#define LEDon
#define LEDoff
#define LED2on
#define LED2off

#include "cie1931.h"

#define Serial1 Serial

// debug output
#if 0
#define D(...)
#else
#define D(...) Serial1.printf(__VA_ARGS__)
#endif

// use "reset" header for external switch
#define switchPIN 0

bool config_changed = false;
String mqtt_server;
String mqtt_prefix = "/openHAB/ZZZhostnameZZZ/";

WiFiManager wifiManager;
WiFiClient wclient;
PubSubClient client(wclient);

// OTA update
ESP8266WebServer httpServer(80);
ESP8266HTTPUpdateServer httpUpdater;

// called when no wifi present
void wifiConfigCallback(WiFiManager *wifiManager) {
	LEDon;
	LED2on;
	Serial1.println("WiFi configuration mode...");
	Serial1.printf("Network: %s\r\nPassword: %s\r\n\r\n", WIFI_NETWORK, WIFI_PASSWORD);
}
void wifiSaveConfigCallback() {
	Serial1.println("Configuration updated.");
	config_changed = true;
}


int fader_speed = 0;		//< speed of the color change effect

#define CHANNELS 3
#define redPIN 0
#define greenPIN 1
#define bluePIN 2

int ledIs[CHANNELS];		//< current brightness for all channels
int ledTarget[CHANNELS];	//< target brightness for all channels

int switch_pin_state = 0;
int switch_active = 0;

void webServerRoot() {
	String response = "ESP8266 ESP-01 WS2812b module - " __DATE__ "\n\nHostname: " + WiFi.hostname() +
		"\nMQTT server: " + mqtt_server +
		"\nMQTT: " + mqtt_prefix +
		"\n\nFader: " + fader_speed + "\n";
	for (int i = 0; i < CHANNELS; i++)
		response += String("\Channel ") + i + ": cur=" + ledIs[i] + " tgt=" + ledTarget[i];
	httpServer.send(200, "text/plain", response);
}

bool updateLED(int pin, int delta) {
	int val = ledIs[pin];
	// do nothing if the value has been reached
	if (ledTarget[pin] == val)
		return false;

	// slowly increment/decrement value towards target
	if (val+delta < ledTarget[pin])
		val += delta;
	else if (val-delta > ledTarget[pin])
		val -= delta;
	else
		val = ledTarget[pin];
	ledIs[pin] = val;
	return true;
}

// set target value in the range 0..255
void setLEDTarget(int pin, int payload) {
	int val = constrain(payload, 0, 255);
	ledTarget[pin] = val;
}

// set taget value in the range 0..100 (for openHAB)
int setLED100Target(int pin, String payload) {
	int val = map(payload.toInt(), 0, 100, 0, 255);
	setLEDTarget(pin, val);
	return val;
}

RGBConverter converter;

float _h, _s, _v = 0;

// this is a hacked up copy of hsvToRgb that attempts to keep brightness when
// cycling colors. HSV goes from 33% (one channel) to 66% (two channels),
// causing noticable fluctuations. This function always keeps 66% (without
// compensating for the different subjective brightness of R vs G vs B). Still,
// this is better than HSV for a color fader.
void colorloopToRgb(double h, double s, double v, byte rgb[]) {
	double r, g, b;

	int i = int(h * 3);
	double f = h * 3 - i;
	double p = v * (1 - s);
	double q = v * (1 - f * s);
	double t = v * (1 - (1 - f) * s);

	switch(i % 3){
		case 0: r = v, g = t, b = q; break;
		case 1: r = q, g = v, b = t; break;
		case 2: r = t, g = q, b = v; break;
	}

	rgb[0] = r * 255;
	rgb[1] = g * 255;
	rgb[2] = b * 255;
}


// set target values according to HSV triple in openHAB format
void setHSV(float h, float s, float v, bool keep_brightness=false) {
	byte rgb[3];
	_h = h;
	_s = s;
	_v = v;
	if (keep_brightness)
		colorloopToRgb(h/360, s/100, v/100, rgb);
	else
		converter.hsvToRgb(h/360, s/100, v/100, rgb);

	setLEDTarget(redPIN, rgb[redPIN]);
	setLEDTarget(greenPIN, rgb[greenPIN]);
	setLEDTarget(bluePIN, rgb[bluePIN]);
}

// ******************** NeoPixel ********************
// 20 pixels, pin is RXD on default DMA method
#ifdef NEOPIXEL
NeoPixelBus<NeoGrbFeature,NeoEsp8266Uart800KbpsMethod> neopixel(150);
#endif // NEOPIXEL

// process a published MQTT event
void mqtt_event(const MQTT::Publish& pub) {
	Serial1.print(pub.topic());
	Serial1.print(" => ");
	Serial1.println(pub.payload_string());

	String payload = pub.payload_string();
	int lash_slash = pub.topic().lastIndexOf('/');
	if (lash_slash == -1)
		return;
	String topic_path = pub.topic().substring(0, lash_slash);
	String topic_name = pub.topic().substring(lash_slash+1);

	if(topic_name == "Reset"){
		Serial1.println("Resetting CPU!\r\n");
		ESP.restart();
	} else
	if(topic_name == "Config"){
		Serial1.println("Starting configuration portal!\r\n");
		wifiManager.startConfigPortal(WIFI_NETWORK, WIFI_PASSWORD);
	} else
	if(topic_name == "RGB"){
		fader_speed = 0;
		int c1 = payload.indexOf(';');
		int c2 = payload.indexOf(';',c1+1);

		setLED100Target(redPIN, payload);
		setLED100Target(greenPIN, payload.substring(c1+1,c2));
		setLED100Target(bluePIN, payload.substring(c2+1));
		client.publish(mqtt_prefix + "Fader", "0");
	}
	if(topic_name == "HSV"){
		if (payload == "ON") {
			payload = "0,0,100";
			switch_active = 1;
		} else if (payload == "OFF") {
			payload = "0,0,0";
			switch_active = 0;
		}
		int c1 = payload.indexOf(',');
		int c2 = payload.indexOf(',',c1+1);


		setHSV(payload.toFloat(), payload.substring(c1+1,c2).toFloat(), payload.substring(c2+1).toFloat(), false);
		client.publish(mqtt_prefix + "Fader", "0");

	}
	else if(topic_name == "Fader"){
		fader_speed = payload.toInt()*10;
	}
}

unsigned long last_tick, last_shift;	//< timestamp of last loop run, for smooth fading

// perform (re)subscription to MQTT server
void subscribe() {
	LEDon;
	client.set_server(mqtt_server);
	if (client.connect(WiFi.hostname())) {
		mqtt_prefix.replace("ZZZhostnameZZZ", WiFi.hostname());
		client.subscribe(mqtt_prefix + "+");
		Serial1.printf("MQTT connected to %s: %s\r\n", mqtt_server.c_str(), mqtt_prefix.c_str());
		client.publish(mqtt_prefix + "Build", __DATE__);
		client.publish(mqtt_prefix + "IP", WiFi.localIP().toString().c_str());
		LEDoff;
	}
}

void loadConfig() {
	if (!SPIFFS.begin()) {
		Serial1.println("No filesystem.");
		return;
	}
	if (!SPIFFS.exists("/config.json")) {
		Serial1.println("No config.json.");
		return;
	}
	File config = SPIFFS.open("/config.json", "r");
	if (!config) {
		Serial1.println("Could not open config.json.");
		return;
	}
	size_t size = config.size();
	// Allocate a buffer to store contents of the file.
	std::unique_ptr<char[]> buf(new char[size]);

	config.readBytes(buf.get(), size);
	DynamicJsonBuffer jsonBuffer;
	JsonObject& json = jsonBuffer.parseObject(buf.get());
	json.printTo(Serial1);
	if (json.success()) {
		mqtt_server = json["mqtt_server"].as<String>();
	} else {
		Serial1.println("Could not parse config.json.");
	}
}
void saveConfigIfNeeded() {
	if (!config_changed)
		return;

	Serial1.println("Saving config.json...");
	DynamicJsonBuffer jsonBuffer;
	JsonObject& json = jsonBuffer.createObject();
	json["mqtt_server"] = mqtt_server;

	File config = SPIFFS.open("/config.json", "w");
	if (!config) {
		Serial1.println("failed to open config file for writing");
		return;
	}
	json.printTo(Serial1);
	json.printTo(config);
	config.close();
}

void setup() {
	// configure reset as input
	pinMode(switchPIN, FUNCTION_0);
	pinMode(switchPIN, INPUT_PULLUP);

	// Setup console
	Serial1.begin(115200);
	while (!Serial1) /* busy loop for serial to attach */;
	Serial1.printf("\r\nESP RGBWW (C) Andreas H, Georg L. - ESP-01 WS2812b version - " __DATE__ "\r\n\r\n");

#ifdef NEOPIXEL
	// reset pixels to black
	Serial1.println("Activating NeoPixel support...");
	neopixel.Begin();
	neopixel.ClearTo(RgbColor(0, 0, 0));
	neopixel.Show();
#endif // NEOPIXEL

	loadConfig();

	// register MQTT event listener
	client.set_callback(mqtt_event);

	LEDon;

	// start WiFi registration
	Serial1.println("Connecting to WiFi...");
	wifiManager.setConfigPortalTimeout(180);
	wifiManager.setAPCallback(wifiConfigCallback);
	wifiManager.setSaveConfigCallback(wifiSaveConfigCallback);
	WiFiManagerParameter mqtt_server_config("server", "MQTT Server", mqtt_server.c_str(), 40);
	wifiManager.addParameter(&mqtt_server_config);
	if (mqtt_server.length() > 0)
		wifiManager.autoConnect(WIFI_NETWORK, WIFI_PASSWORD);
	else
		wifiManager.startConfigPortal(WIFI_NETWORK, WIFI_PASSWORD);
	mqtt_server = mqtt_server_config.getValue();

	Serial1.printf("\r\nWiFi connected: %s (%s)\r\n",
		WiFi.hostname().c_str(),
		WiFi.localIP().toString().c_str());

	httpUpdater.setup(&httpServer);
	httpServer.on("/", webServerRoot);
	httpServer.begin();

	subscribe();

	saveConfigIfNeeded();

	for (int i=0; i<CHANNELS; i++) {
		ledIs[i] = 0;
		ledTarget[i] = 0;
	}
	LEDoff;
	LED2off;
	last_tick = micros();
}

void processSwitch() {
	int pin = digitalRead(switchPIN);
	if (pin != switch_pin_state) {
		switch_pin_state = pin;
		Serial1.printf("\r\nSwitch pin: %d\r\n", pin);
		if (pin == 0) {
			fader_speed = 0;
			switch_active = 1 - switch_active;
			Serial1.printf("\r\nSwitching light: %d\r\n", switch_active);
			client.publish(mqtt_prefix + "HSV", switch_active ? "ON" : "OFF");
		}
	}

}

void loop() {
	// check connectivity, perform MQTT loop
	if (client.connected())
		client.loop();
	else
		subscribe();

	httpServer.handleClient();

	processSwitch();

	unsigned long t = micros();
	if (fader_speed > 0) {
		// rotate the hue value around the clock
		_h = _h + (t-last_tick)/1000000.0*fader_speed;
		if (_h > 360)
			_h -= 360;

		setHSV(_h, _s, _v, true);
		for (int i=0; i < CHANNELS; i++)
			ledIs[i] = ledTarget[i];
	}

	// update all PWM channels
	bool updated = false;
	for (int i=0; i < CHANNELS; i++)
		updated |= updateLED(i, 1);

#if NEOPIXEL
	if (last_shift < t + 10000) {
		neopixel.RotateRight(1);
		neopixel.ClearTo(RgbColor(cie[ledIs[0]], cie[ledIs[1]], cie[ledIs[2]]), 0, 0);
		neopixel.Show();
		last_shift = t;
	}
#endif

	last_tick = t;
	delay(3);
}
