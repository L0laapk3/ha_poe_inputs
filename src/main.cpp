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



struct DigitalInput {
	static constexpr float THRESHOLD_LOW = 3.3f * 0.25f;
	static constexpr float THRESHOLD_HIGH = 3.3f * 0.75f;

	HAMqttDevice device;
	std::string topic;
	uint8_t channel;
	std::optional<bool> boolState, lastBoolState;


	DigitalInput(std::string name, uint8_t _channel, HAMqttDevice _device, std::string state_topic_suffix) :
		device(std::move(_device)), channel(_channel) {
		// Use device-specific state topic (not discovery namespace)
		topic = mqtt_client_id + "/" + state_topic_suffix;

		// Global device config - applied to all sensors
		device.addConfigVar("device", "{\"identifiers\":[\"schakelaars_living\"],\"name\":\"Schakelaars Living\",\"model\":\"ESP32 ADS1015\",\"manufacturer\":\"Custom\"}");

		// Set explicit state topic
		device.addConfigVar("stat_t", topic.c_str());

		// Sensors depend on ADS availability
		device.addConfigVar("avty_t", topic_ads_availability.c_str());
		device.addConfigVar("payload_avail", "online");
		device.addConfigVar("payload_not_avail", "offline");
	}

	DigitalInput(std::string name, uint8_t channel, std::string state_topic_suffix)
		: DigitalInput(name, channel, HAMqttDevice(name.c_str(), HAMqttDevice::BINARY_SENSOR, MQTT_HA_DISCOVERY_PREFIX.c_str()), state_topic_suffix) {
		// Digital input specific config is already handled by delegating constructor
		// Just add the binary sensor specific payloads
		device.addConfigVar("payload_on", "1");
		device.addConfigVar("payload_off", "0");
	}

	void publishDiscovery() {
		mqtt.publish(device.getConfigTopic().c_str(), device.getConfigPayload().c_str(), true);
	}

	virtual void read() {
		float voltage = ads.computeVolts(ads.readADC_SingleEnded(channel));
		boolState = voltage > (lastBoolState.value_or(false) ? THRESHOLD_LOW : THRESHOLD_HIGH);
	}

	virtual void publish() {
		read();
		if (boolState == lastBoolState)
			return;
		lastBoolState = boolState;
		mqtt.publish(topic.c_str(), boolState.value() ? "1" : "0", true);
	}

	void update() {
		read();
		publish();
	}
};

struct AnalogInput : DigitalInput {
	uint8_t analogChannel;
	std::optional<float> floatState, lastFloatState;

	AnalogInput(const char* name, uint8_t _gateChannel, uint8_t _analogChannel, std::string state_topic_suffix)
		: DigitalInput(name, _gateChannel, HAMqttDevice(name, HAMqttDevice::SENSOR, MQTT_HA_DISCOVERY_PREFIX.c_str()), state_topic_suffix), analogChannel(_analogChannel) {
		device.addConfigVar("unit_of_measurement", "%");
		device.addConfigVar("state_class", "measurement");
		device.addConfigVar("icon", "mdi:brightness-percent");
	}

	void read() override {
		DigitalInput::read();
		float analogVoltage = ads.computeVolts(ads.readADC_SingleEnded(analogChannel));
		floatState = boolState.value_or(false) ? (analogVoltage / 3.3f) * 100.0f : 0.0f;
	}

	void publish() override {
		if (!lastFloatState.has_value() || abs(floatState.value() - lastFloatState.value()) > 0.1f) {
			lastFloatState = floatState;
			std::ostringstream oss;
			oss << std::fixed << std::setprecision(1) << floatState.value();
			mqtt.publish(topic.c_str(), oss.str().c_str(), true);
		}
	}
};

AnalogInput dimmerInput{"Living Dimmer", 0, 1, "dimmer"};
DigitalInput ain2Input{"Living Switch 1", 2, "switch1"};
DigitalInput ain3Input{"Living Switch 2", 3, "switch2"};

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

	Wire.begin();
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
	adsStatusSensor.addConfigVar("payload_avail", "online");
	adsStatusSensor.addConfigVar("payload_not_avail", "offline");
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
		dimmerInput.publishDiscovery();
		ain2Input.publishDiscovery();
		ain3Input.publishDiscovery();
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
		// Update all inputs - they will read from ADS and publish to MQTT if changed
		dimmerInput.update();
		ain2Input.update();
		ain3Input.update();
	}

	delay(10);
}