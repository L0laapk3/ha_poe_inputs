// we have ESP32-POE-WROVER
#include <cstdint>
#define BOARD_HAS_PSRAM

#include "HardwareSerial.h"
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

std::string MQTT_HA_DISCOVERY_PREFIX = "homeassistant";

// Matrix scanning configuration
// 2 rows (driver pins) x 3 columns (input pins) = 6 switches
constexpr std::array<uint8_t, 2> MATRIX_DRIVER_PINS = {13, 15};  // Driver pins - outputs
constexpr std::array<uint8_t, 3> MATRIX_INPUT_PINS = {14, 4, 32};  // Input pins - inputs with pulldown
constexpr uint8_t ANALOG_PIN = 36;  // Analog input pin
constexpr uint8_t ANALOG_BUTTON_PIN = 33; // pulldown

#if USE_WIFI
WiFiClient wifiClient;
PubSubClient mqtt(wifiClient);
#else
WiFiClient ethClient;
PubSubClient mqtt(ethClient);
#endif


template<typename T>
struct Input {
	Input(std::string _topic, std::string name, uint8_t pin, HAMqttDevice::DeviceType type) :
		device(name.c_str(), type, MQTT_HA_DISCOVERY_PREFIX.c_str()), pin(pin) {
		topic = mqtt_client_id + "/" + _topic;

		device.addConfigVar("device", "{\"identifiers\":[\"schakelaars_living\"],\"name\":\"Schakelaars Living\",\"model\":\"ESP32\",\"manufacturer\":\"Custom\"}");
		device.addConfigVar("payload_avail", "online");
		device.addConfigVar("payload_not_avail", "offline");
		device.addConfigVar("avty_t", topic_availability.c_str());
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

	virtual void publish() = 0;

	void update() {
		read();
		publish();
	}
};

struct DigitalInput : public Input<bool> {
	DigitalInput(std::string topic, std::string name, uint8_t pin)
		: Input<bool>(topic, name, pin, HAMqttDevice::BINARY_SENSOR) {
		this->device.addConfigVar("payload_on", "1");
		this->device.addConfigVar("payload_off", "0");
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

	void publish() override {
		if (!lastBoolState.has_value() || boolState != lastBoolState) {
			lastBoolState = boolState;
			mqtt.publish(topic.c_str(), boolState.value() ? "1" : "0", true);
		}
	}
};


struct AnalogInput : Input<float> {
	std::optional<float> floatState, lastFloatState;

	AnalogInput(std::string topic, std::string name, uint8_t pin)
		: Input(topic, name, pin, HAMqttDevice::SENSOR) {
		device.addConfigVar("unit_of_measurement", "%");
		device.addConfigVar("state_class", "measurement");
		device.addConfigVar("icon", "mdi:brightness-percent");
		pinMode(pin, INPUT);
	}

	static constexpr int ADC_MAX = 4095; // 12-bit ADC on ESP32
	static constexpr float HYSTERESIS_PERCENT = 0.25;

	void read() override {
		int analogValue = analogRead(pin);
		floatState = (analogValue / float(ADC_MAX)) * 100.0f;
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

DigitalInput switch0Input("switch0", "Living Switch 0", MATRIX_DRIVER_PINS[0]);
DigitalInput switch1Input("switch1", "Living Switch 1", MATRIX_DRIVER_PINS[1]);
DigitalInput switch2Input("switch2", "Living Switch 2", MATRIX_DRIVER_PINS[2]);
DigitalInput switch3Input("switch3", "Living Switch 3", MATRIX_DRIVER_PINS[0]);
DigitalInput switch4Input("switch4", "Living Switch 4", MATRIX_DRIVER_PINS[1]);
DigitalInput switch5Input("switch5", "Living Switch 5", MATRIX_DRIVER_PINS[2]);

// Analog input
AnalogInput dimmerInput("dimmer", "Living Dimmer", ANALOG_PIN);
DigitalInput dimmerSwitchInput("dimmer_switch", "Living Dimmer Switch", MATRIX_DRIVER_PINS[0]);

// Create status sensor to show device availability in HA
HAMqttDevice statusSensor("Status", HAMqttDevice::BINARY_SENSOR, MQTT_HA_DISCOVERY_PREFIX.c_str());

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

	for (auto pin : MATRIX_DRIVER_PINS) {
		pinMode(pin, OUTPUT);
		digitalWrite(pin, LOW);
	}


	mqtt.setServer(mqtt_server.c_str(), mqtt_port);
	mqtt.setBufferSize(2048);
	mqtt.setCallback([](char* topic, byte* payload, unsigned int length) {});

	// Setup main status sensor (ESP32 connectivity)
	statusSensor.addConfigVar("device", "{\"identifiers\":[\"schakelaars_living\"],\"name\":\"Schakelaars Living\",\"model\":\"ESP32\",\"manufacturer\":\"Custom\"}");
	statusSensor.addConfigVar("payload_on", "online");
	statusSensor.addConfigVar("payload_off", "offline");
	statusSensor.addConfigVar("stat_t", topic_availability.c_str());
	statusSensor.addConfigVar("device_class", "connectivity");
	statusSensor.addConfigVar("entity_category", "diagnostic");
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
		switch0Input.publishDiscovery();
		switch1Input.publishDiscovery();
		switch2Input.publishDiscovery();
		switch3Input.publishDiscovery();
		switch4Input.publishDiscovery();
		switch5Input.publishDiscovery();
		dimmerInput.publishDiscovery();
		dimmerSwitchInput.publishDiscovery();

		mqtt.publish(statusSensor.getConfigTopic().c_str(), statusSensor.getConfigPayload().c_str(), true);
		mqtt.publish(topic_availability.c_str(), "online", true);
		Serial.println("Published MQTT discovery configs");
	}
  	if (!mqtt.loop()) {
		mqtt_print_error();
		return;
	}


	constexpr unsigned long SETTLING_TIME_MS = 10;

	digitalWrite(MATRIX_DRIVER_PINS[0], HIGH);
	delay(SETTLING_TIME_MS);
	switch0Input.update();
	switch1Input.update();
	switch2Input.update();
	digitalWrite(MATRIX_DRIVER_PINS[0], LOW);

	digitalWrite(MATRIX_DRIVER_PINS[1], HIGH);
	delay(SETTLING_TIME_MS);
	switch3Input.update();
	switch4Input.update();
	switch5Input.update();
	digitalWrite(MATRIX_DRIVER_PINS[0], LOW);

	dimmerInput.update();
	dimmerSwitchInput.update();
}