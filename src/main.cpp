// we have ESP32-POE-WROVER
#define BOARD_HAS_PSRAM

#include "HardwareSerial.h"
#include <Wire.h>
#include <Adafruit_ADS1X15.h>
#include <ETH.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <HAMqttDevice.h>
#include <optional>
#include <string>
#include <sstream>
#include <iomanip>


#define USE_WIFI false

#if USE_WIFI
std::string wifi_ssid = ;
std::string wifi_password = ;
#endif

std::string mqtt_server = "192.168.0.254";
int mqtt_port = 1883;
std::string mqtt_user = "schakelaars_living";
std::string mqtt_password = "schakelaars_living:D";
std::string mqtt_client_id = "schakelaars_living";
std::string topic_availability = mqtt_client_id + "/availability";
std::string topic_ads_availability = mqtt_client_id + "/ads_availability";

std::string MQTT_HA_DISCOVERY_PREFIX = "homeassistant";



Adafruit_ADS1015 ads;

#if USE_WIFI
WiFiClient wifiClient;
PubSubClient mqtt(wifiClient);
#else
WiFiClient ethClient;
PubSubClient mqtt(ethClient);
#endif


template<typename T, bool ADS = false>
struct Input {
	Input(std::string _topic, std::string name, uint8_t pin, HAMqttDevice _device) :
		device(std::move(_device)), pin(pin) {
		// Use device-specific state topic (not discovery namespace)
		topic = mqtt_client_id + "/" + _topic;

		device.addConfigVar("device", "{\"identifiers\":[\"schakelaars_living\"],\"name\":\"Schakelaars Living\",\"model\":\"ESP32 ADS1015\",\"manufacturer\":\"Custom\"}");
		device.addConfigVar("payload_avail", "online");
		device.addConfigVar("payload_not_avail", "offline");
		device.addConfigVar("avty_t", (ADS ? topic_ads_availability : topic_availability).c_str());
		device.addConfigVar("stat_t", topic.c_str());
	}

	HAMqttDevice device;
	uint8_t pin;
	std::optional<T> boolState, lastBoolState;

	std::string topic;

	void publishDiscovery() {
		mqtt.publish(device.getConfigTopic().c_str(), device.getConfigPayload().c_str(), true);
	}

	virtual void read() = 0;

	virtual void publish() {
		if (!lastBoolState.has_value() || boolState != lastBoolState) {
			lastBoolState = boolState;
			mqtt.publish(topic.c_str(), boolState.value() ? "1" : "0", true);
		}
	}

	void update() {
		read();
		publish();
	}
};

template<bool ADS = false>
struct DigitalInput : public Input<bool, ADS> {
	using Input<bool, ADS>::Input;

	DigitalInput(std::string topic, std::string name, uint8_t channel)
		: DigitalInput(topic, name, channel, HAMqttDevice(name.c_str(), HAMqttDevice::BINARY_SENSOR, MQTT_HA_DISCOVERY_PREFIX.c_str())) {
		this->device.addConfigVar("payload_on", "1");
		this->device.addConfigVar("payload_off", "0");
	}
};

struct DigitalGPIOInput : public DigitalInput<false> {
	DigitalGPIOInput(std::string topic, std::string name, uint8_t pin)
		: DigitalInput(topic, name, pin) {
		pinMode(pin, INPUT_PULLDOWN);
	}

	static constexpr unsigned long DEBOUNCE_DELAY_MS = 100;
	unsigned long lastChangeTime = 0;

	void read() override {
		if (millis() - lastChangeTime >= DEBOUNCE_DELAY_MS) {
			boolState = digitalRead(pin);
			if (boolState != lastBoolState)
				lastChangeTime = millis();
		}
	}
};


struct DigitalAdsInput : public DigitalInput<true> {
	using DigitalInput<true>::DigitalInput;

	static constexpr float THRESHOLD_LOW = 3.3f * 0.25f;
	static constexpr float THRESHOLD_HIGH = 3.3f * 0.75f;

	void read() override {
		float voltage = ads.computeVolts(ads.readADC_SingleEnded(pin));
		boolState = voltage > (lastBoolState.value_or(false) ? THRESHOLD_LOW : THRESHOLD_HIGH); // debouncing
	}
};

struct AnalogAdsInput : Input<float, true> {
	std::optional<float> floatState, lastFloatState;

	AnalogAdsInput(std::string topic, std::string name, uint8_t pin)
		: Input(topic, name, pin, HAMqttDevice(name.c_str(), HAMqttDevice::SENSOR, MQTT_HA_DISCOVERY_PREFIX.c_str())) {
		device.addConfigVar("unit_of_measurement", "%");
		device.addConfigVar("state_class", "measurement");
		device.addConfigVar("icon", "mdi:brightness-percent");
	}

	static constexpr float VOLTAGE = 3.3f;
	static constexpr float CROP = 0.025 * VOLTAGE; // 2.5% crop on both ends
	static constexpr float HYSTERESIS_PERCENT = 0.25;

	void read() override {
		float analogVoltage = ads.computeVolts(ads.readADC_SingleEnded(pin));
		floatState = std::clamp((analogVoltage - CROP) / (3.3f - 2 * CROP), 0.0f, 1.0f) * 100.0f;
	}

	void publish() override {
		if (!lastFloatState.has_value() || abs(floatState.value() - lastFloatState.value()) > HYSTERESIS_PERCENT) {
			lastFloatState = floatState;
			std::ostringstream oss;
			oss << std::fixed << std::setprecision(1) << floatState.value();
			mqtt.publish(topic.c_str(), oss.str().c_str(), true);
		}
	}
};



DigitalAdsInput  ain0Input   ("dimmerSwitch", "Living Dimmer Switch", 0);
AnalogAdsInput   dimmerInput ("dimmer",       "Living Dimmer",   1);
DigitalAdsInput  ain2Input   ("switch1",      "Living Switch 1", 2);
DigitalAdsInput  ain3Input   ("switch2",      "Living Switch 2", 3);
DigitalGPIOInput button1Input("switch3",      "Gang Switch 1",   14);
DigitalGPIOInput button2Input("switch4",      "Gang Switch 2",   4);

// Create status sensors to show device availability in HA
HAMqttDevice statusSensor("Status", HAMqttDevice::BINARY_SENSOR, MQTT_HA_DISCOVERY_PREFIX.c_str());
HAMqttDevice adsStatusSensor("ADS1015 Status", HAMqttDevice::BINARY_SENSOR, MQTT_HA_DISCOVERY_PREFIX.c_str());

#if !USE_WIFI
static bool eth_connected = false;

void WiFiEvent(WiFiEvent_t event) {
	switch (event) {
		case ARDUINO_EVENT_ETH_START:
		Serial.println("ETH Started");
		ETH.setHostname(mqtt_client_id.c_str());
		break;
		case ARDUINO_EVENT_ETH_CONNECTED:
		Serial.println("ETH Connected");
		break;
		case ARDUINO_EVENT_ETH_GOT_IP:
		Serial.print("ETH IP: ");
		Serial.println(ETH.localIP());
		Serial.print("ETH MAC: ");
		Serial.println(ETH.macAddress());
		eth_connected = true;
		break;
		case ARDUINO_EVENT_ETH_DISCONNECTED:
		Serial.println("ETH Disconnected");
		eth_connected = false;
		break;
		case ARDUINO_EVENT_ETH_STOP:
		Serial.println("ETH Stopped");
		eth_connected = false;
		break;
		default:
		break;
	}
}
#endif

void mqtt_print_error() {
	int rc = mqtt.state();
	Serial.print("failed, rc=");
	Serial.print(rc);
	Serial.print(" (");
	switch(rc) {
		case -4: Serial.print("TIMEOUT");         break;
		case -3: Serial.print("CONNECTION_LOST"); break;
		case -2: Serial.print("CONNECT_FAILED");  break;
		case -1: Serial.print("DISCONNECTED");    break;
		case 1:  Serial.print("BAD_PROTOCOL");    break;
		case 2:  Serial.print("BAD_CLIENT_ID");   break;
		case 3:  Serial.print("UNAVAILABLE");     break;
		case 4:  Serial.print("BAD_CREDENTIALS"); break;
		case 5:  Serial.print("UNAUTHORIZED");    break;
		default: Serial.print("UNKNOWN");         break;
	}
	Serial.println(")");
	delay(100);
}

void setup() {
	delay(500);  // Give hardware time to stabilize

	Serial.begin(921600);
	Serial.println("Hello! ESP32-POE ADC to MQTT");

#if USE_WIFI
	Serial.print("Connecting to WiFi: ");
	Serial.println(wifi_ssid.c_str());
	WiFi.mode(WIFI_STA);
	WiFi.begin(wifi_ssid.c_str(), wifi_password.c_str());

	while (WiFi.status() != WL_CONNECTED) {
		delay(500);
		Serial.print(".");
	}

	Serial.println("\nWiFi connected!");
	Serial.print("IP address: ");
	Serial.println(WiFi.localIP());
#else
	WiFi.mode(WIFI_OFF);
	Serial.println("Initializing Ethernet...");
	WiFi.onEvent(WiFiEvent);

	ETH.begin();
#endif

	Wire.begin(33, 32);
	Wire.setClock(10000);

	mqtt.setServer(mqtt_server.c_str(), mqtt_port);
	mqtt.setBufferSize(2048);
	mqtt.setCallback([](char* topic, byte* payload, unsigned int length) {});

	// Setup main status sensor (ESP32 connectivity) - no dependencies
	statusSensor.addConfigVar("device", "{\"identifiers\":[\"schakelaars_living\"],\"name\":\"Schakelaars Living\",\"model\":\"ESP32 ADS1015\",\"manufacturer\":\"Custom\"}");
	statusSensor.addConfigVar("payload_on", "online");
	statusSensor.addConfigVar("payload_off", "offline");
	statusSensor.addConfigVar("stat_t", topic_availability.c_str());
	statusSensor.addConfigVar("device_class", "connectivity");
	statusSensor.addConfigVar("entity_category", "diagnostic");

	// Setup ADS1015 status sensor - depends on ESP32 being online
	adsStatusSensor.addConfigVar("device", "{\"identifiers\":[\"schakelaars_living\"],\"name\":\"Schakelaars Living\",\"model\":\"ESP32 ADS1015\",\"manufacturer\":\"Custom\"}");
	adsStatusSensor.addConfigVar("payload_on", "online");
	adsStatusSensor.addConfigVar("payload_off", "offline");
	adsStatusSensor.addConfigVar("stat_t", topic_ads_availability.c_str());
	adsStatusSensor.addConfigVar("avty_t", topic_availability.c_str());  // ADS depends on ESP32
	adsStatusSensor.addConfigVar("device_class", "connectivity");
	adsStatusSensor.addConfigVar("entity_category", "diagnostic");
}

void loop() {
#if USE_WIFI
	// TODO
#else
	// Check if Ethernet connection was lost
	if (!eth_connected) {
		Serial.print("Ethernet connection lost, reconnecting");
		while (!eth_connected) {
			delay(100);
			Serial.print(".");
		}
		Serial.println("\nEthernet reconnected!");
		delay(2000);  // Give Ethernet time to fully initialize before using WiFiClient
	}
#endif

	if (!mqtt.connected()) {
		Serial.print("Attempting MQTT connection... ");
		// Connect with Last Will and Testament (LWT) for availability
		if (!mqtt.connect(mqtt_client_id.c_str(), mqtt_user.c_str(), mqtt_password.c_str(),
		                 topic_availability.c_str(), 0, true, "offline")) {
			mqtt_print_error();
			return;
		}

		Serial.println("connected");

		// Publish mqtt autodiscovery & availability as online
		mqtt.publish(topic_ads_availability.c_str(), "offline", false);
		ain0Input.publishDiscovery();
		dimmerInput.publishDiscovery();
		ain2Input.publishDiscovery();
		ain3Input.publishDiscovery();
		button1Input.publishDiscovery();
		button2Input.publishDiscovery();
		mqtt.publish(statusSensor.getConfigTopic().c_str(), statusSensor.getConfigPayload().c_str(), true);
		mqtt.publish(adsStatusSensor.getConfigTopic().c_str(), adsStatusSensor.getConfigPayload().c_str(), true);
		mqtt.publish(topic_availability.c_str(), "online", true);
		Serial.println("Published MQTT discovery configs");
	}
  	if (!mqtt.loop()) {
		mqtt_print_error();
		return;
	}


	static bool adsAvailable = false;
	if (!adsAvailable) {
		adsAvailable = ads.begin(0x48, &Wire);
		if (adsAvailable) {
			mqtt.publish(topic_ads_availability.c_str(), "online", true);
			Serial.println("ADS available!");
		}
	}

	if (adsAvailable) {
		ain0Input.update();
		dimmerInput.update();
		ain2Input.update();
		ain3Input.update();
	}
	button1Input.update();
	button2Input.update();

	delay(10);
}