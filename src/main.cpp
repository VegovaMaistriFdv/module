/*
	© 2023 Nejc Krmelj <nejc.krmelj.ID@gmail.com>
*/

#include <Arduino.h>
#include <Wire.h>
#include <EEPROM.h>
#include <ArduinoJson.h>
#include <FS.h>
// Communication
#include <WiFi.h>
#include <httpUpdate.h>
#include <WiFiClient.h>
#include <WiFiClientSecure.h>
// #include <ESPAsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <PubSubClient.h>
// Sensors
// #include "air_sensor.h"
// NTP Timestamp
#include <NTPClient.h>
#include <WiFiUdp.h>

#define FIRMWARE_VERSION "0.1"
#define SERVER_NAME "192.168.2.183"
#define SERVER_PORT 5000
#define SERVER_UPDATE_PATH "/firmware/update/host-module/firmware.bin"

#define EEPROM_SIZE 256
#define MASTER_MODULE_INDEX 0

StaticJsonDocument<100> doc;
String jsonData;
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP);


// MQTT Broker
WiFiClient wlanClient;
PubSubClient mqttClient(wlanClient);

long time_unix;


typedef struct
{
	String id;
	String time;
	float temperature;
	float humidity;
	float vocGases;
	bool _update;
} air_data;

typedef struct {
  String id;
  String name;
  String place;
  String masterIP;

} module_props;

module_props module {
  id: "unknown",
  name: "AS-host",
  place: "unkown",
  masterIP: "",
};


// Present airData from modules for analysis and for posting/publishing
air_data modulesAirData[0] = {};

// StaticJsonDocument<1024> payload;
DynamicJsonDocument payload(1024);
JsonArray airData = payload.createNestedArray("fire");


// Intervals
unsigned long sensorReadingInterval = 5000;

unsigned long sensorReadingPreviousTime = 0;

unsigned long setup_buttonActiveTime = 0;			///
volatile bool setup_interrupt_active = false; ///
bool setup_active = false;

unsigned long setupModeLedInterval = 700;
unsigned long setupModeLedPreviousTime = 0;
bool setupLedIsOn = false;


bool exec_wifi_reconnect_flag = false;


// Functions
bool connectToWiFi();
void connectToBroker();
void updateFirmware();
void callback(char *topic, byte *payload, unsigned int length);

void setup()
{
	// PINS SETUP
	pinMode(LED_PIN, OUTPUT);
	digitalWrite(LED_PIN, 0);

	// Init Serial
	Serial.begin(115200);
	Serial.println();
	Serial.print("Running software from github.com/AirSense-DD3 \nVersion: ");
	Serial.println(FIRMWARE_VERSION);
	Serial.println();

	// Init EEPROM
	EEPROM.begin(EEPROM_SIZE);

	// modulesAirData[MASTER_MODULE_INDEX].id = module.name;
	payload["source"] = "first-module";
	airData[MASTER_MODULE_INDEX]["latitude"] = "100";
	airData[MASTER_MODULE_INDEX]["longitude"] = "100";

	// Connect to WiFi WPA2
	if (connectToWiFi()) {
		updateFirmware();
		connectToBroker();
	}

	if (!setup_active)
	{
		digitalWrite(LED_PIN, 1);
	}
}

void loop()
{
	// Local Reading
	if ((millis() - sensorReadingPreviousTime >= sensorReadingInterval) && !setup_active)
	{
    // Read sensors
		Serial.println("Updating air data");

		if(false) return;

		// Get current time
		timeClient.update();
		time_unix = timeClient.getEpochTime();
		airData[MASTER_MODULE_INDEX]["time"] = time_unix;
		// modules_update[MASTER_MODULE_INDEX] = true;

		sensorReadingPreviousTime = millis();

    
		if (!setup_active)
		{
			if (!mqttClient.connected())
			{
				connectToBroker();
				return;
			}

      // Send data to MQTT Broker
			// modules_update[i] = false;
			char jsonBuffer[1024];
			serializeJson(payload, jsonBuffer);
			mqttClient.publish("fire/new", jsonBuffer);

			Serial.print("Updated module on cloud: ");
		}

	}

	


	if (exec_wifi_reconnect_flag)
	{
		connectToWiFi();
		updateFirmware();
		connectToBroker();
		exec_wifi_reconnect_flag = false;
	}

	mqttClient.loop(); // Check mqtt topics

	// Delay
	delay(1);
}


bool connectToWiFi()
{
	if (WiFi.status() == WL_CONNECTED)
	{
		return true;
	}

	// Read saved WiFi ssid and password
	// String ssid = module.getString(eeprom_wifi_ssid_addr);
	// String pass = module.getString(eeprom_wifi_password_addr);
  String ssid = WIFI_SSID;
  String pass = WIFI_PASSWORD;
	Serial.print(ssid);
	Serial.println("-");
	Serial.print(pass);
	Serial.println("-");

	// Connect to WiFi
	WiFi.mode(WIFI_STA);
	WiFi.begin(ssid.c_str(), pass.c_str());
	String hostname = "Maister-module-" + module.id;
	WiFi.hostname(hostname.c_str());

	if (WiFi.waitForConnectResult() != WL_CONNECTED)
	{
		Serial.println("[ERROR] WiFi unconnected");
		return false;
	}

	timeClient.begin(); // For timestamp

	digitalWrite(LED_PIN, 1);

	Serial.print("[INFO] Connected to WiFi ");
	Serial.println(WiFi.SSID());
	Serial.print("IP: ");
	Serial.println(WiFi.localIP());
	Serial.print("RSSI: ");
	Serial.println(WiFi.RSSI());
	return true;
}

// Connect to MQTT broker
void connectToBroker()
{
	Serial.println("Connecting to broker");
	mqttClient.setServer(MQTT_BROKER, MQTT_PORT);
	mqttClient.setCallback(callback);
	if (mqttClient.connect(module.name.c_str(), MQTT_USERNAME, MQTT_PASSWORD))
	{
		// mqttClient.setKeepAlive(sensorReadingInterval / 1000);
		mqttClient.setKeepAlive(900);
		Serial.println("[INFO] Connected to MQTT Broker");
	}
	else
	{
		Serial.print("[ERROR] MQTT Broker connection failed with state: ");
		Serial.println(mqttClient.state());
		delay(200);
	}
	mqttClient.subscribe("firmware/update/host-module");
}

// On MQTT message
void callback(char *topic, byte *payload, unsigned int length)
{
	Serial.print("Message arrived in topic: ");
	Serial.println(topic);
	Serial.print("Message:");
	for (int i = 0; i < length; i++)
	{
		Serial.print((char)payload[i]);
	}
	Serial.println();
	Serial.println("-----------------------");

  // -> Update software
	if (strcmp(topic, "firmware/update/host-module") == 0 && true)
	{
		Serial.println("[INFO] Firmware update available");
		updateFirmware();
	}
}

void update_started()
{
	Serial.println("[UPDATE] HTTP update process started");
}

void update_finished()
{
	Serial.println("\n[UPDATE] HTTP update process finished");
}

void update_progress(int cur, int total)
{
	Serial.print(".");
}

void update_error(int err)
{
	Serial.printf("[UPDATE ERROR] HTTP update fatal error code %d\n", err);
}

void updateFirmware()
{
  Serial.println("[ERROR] OTA firmware update is not supported");
	// WiFiClient client;
	// ESPhttpUpdate.setLedPin(LED_PIN, LOW);
	// ESPhttpUpdate.onStart(update_started);
	// ESPhttpUpdate.onEnd(update_finished);
	// ESPhttpUpdate.onProgress(update_progress);
	// ESPhttpUpdate.onError(update_error);

	// t_httpUpdate_return ret = ESPhttpUpdate.update(client, SERVER_NAME, SERVER_PORT, SERVER_UPDATE_PATH, FIRMWARE_VERSION);

	// switch (ret)
	// {
	// case HTTP_UPDATE_FAILED:
	// 	Serial.printf("[UPDATE] Firmware update failed (%d): %s\n", ESPhttpUpdate.getLastError(), ESPhttpUpdate.getLastErrorString().c_str());
	// 	break;
	// case HTTP_UPDATE_NO_UPDATES:
	// 	Serial.println("[UPDATE] No firmware updates available");
	// 	break;
	// case HTTP_UPDATE_OK:
	// 	Serial.println("[UPDATE] Firmware update successful");
	// 	break;
	// }
}