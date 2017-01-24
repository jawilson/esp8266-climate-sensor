#include <Arduino.h>

#include <ESP8266WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <DHT.h>

extern "C" {
#include "user_interface.h"
}

#include "RtcUserMemoryManager.h"
#include "config.h"

using namespace esp8266climatesensor;
using namespace esp8266climatesensor::rtcstorage;

const char* wifi_ssid = WIFI_SSID;
const char* wifi_password = WIFI_PASSWORD;

const char* mqtt_server_host = MQTT_SERVER_HOST;
const int mqtt_server_port = MQTT_SERVER_PORT;
const char* mqtt_user = MQTT_USER;
const char* mqtt_password = MQTT_PASSWORD;

#ifndef MQTT_CERT_FINGERPRINT
WiFiClient wifiClient;
#else
const char* mqtt_cert_fingerprint = MQTT_CERT_FINGERPRINT;
WiFiClientSecure wifiClient;
#endif
PubSubClient mqtt_client(wifiClient);

#define DHTPIN D2       // Temp sensor data pin
#define DHTTYPE DHT22   // Temp sensor type
// Initialize DHT sensor.
DHT dht(DHTPIN, DHTTYPE);

const int sample_period = (REPORT_PERIOD / SAMPLES_TO_AVG) * 1000000;

inline int max(int a,int b) {return ((a)>(b)?(a):(b)); }

bool setupWiFi() {
  Serial.print("Connecting to ");
  Serial.println(wifi_ssid);

  WiFi.begin(wifi_ssid, wifi_password);

  unsigned long start = millis();
  while (!WiFi.isConnected() && (millis() - start) < WIFI_CONNECT_TIMEOUT_MS) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();

  if (WiFi.isConnected()) {
    Serial.println("WiFi connected");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
    return true;
  } else {
    Serial.println("WiFi failed to connect");
    return false;
  }
}

bool setupMqtt() {
  const char* mqtt_client_id =
    RtcUserMemoryManager::getInstance().getMqttClientId();

  Serial.print("MQTT server: ");
  Serial.print(mqtt_server_host);
  Serial.print(":");
  Serial.println(mqtt_server_port);
  Serial.print("MQTT ClientID: ");
  Serial.println(mqtt_client_id);

  mqtt_client.setServer(mqtt_server_host, mqtt_server_port);

  Serial.print("Attempting MQTT connection...");
  if (mqtt_client.connect(mqtt_client_id, mqtt_user, mqtt_password,
                          RtcUserMemoryManager::getInstance().getMqttWillTopic(),
                          1, 0, "goodbye")) {
    Serial.println("connected");

#ifdef MQTT_CERT_FINGERPRINT
    if (!wifiClient.verify(mqtt_cert_fingerprint, mqtt_server_host)) {
      Serial.println("Certificate doesn't match");
      mqtt_client.disconnect();
      return false;
    }
#endif
  } else {
    Serial.print("failed, rc=");
    Serial.println(mqtt_client.state());
  }

  return mqtt_client.connected();
}

SampleData averageSampleData(const SampleData &last_sample) {
  SampleData sample_data;
  float temperature = last_sample.temperature;
  float humidity = last_sample.humidity;

  size_t samples = RtcUserMemoryManager::getInstance().getSampleCount();

  for (size_t i = 0; i < samples; i++) {
    sample_data = RtcUserMemoryManager::getInstance().getSampleData(i);

    humidity += sample_data.humidity;
    temperature += sample_data.temperature;
  }

  sample_data.temperature = temperature / (samples + 1);
  sample_data.humidity = humidity / (samples + 1);

  return sample_data;
}

SampleData getSampleData() {
  SampleData sample_data;

  // Reading temperature or humidity takes about 250 milliseconds!
  // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
  sample_data.humidity = dht.readHumidity();

  // Read temperature as Celsius (the default)
  sample_data.temperature = dht.readTemperature();

  if (isnan(sample_data.humidity) || isnan(sample_data.temperature)) {
    Serial.println("Failed to read temperature and humidity from sensor");
    return sample_data;
  }

  Serial.printf("Read #%d humidity=",
                RtcUserMemoryManager::getInstance().getSampleCount() + 1);
  Serial.print(sample_data.humidity);
  Serial.print("% temperature=");
  Serial.print(sample_data.temperature);
  Serial.println("˚C");

  return sample_data;
}

void publish(const SampleData &sample_data) {
  if (!setupWiFi()) {
    Serial.println("Failed to setup WiFi");
    return;
  }

  if (!setupMqtt()) {
    Serial.println("Failed to setup MQTT");
    return;
  }

  char reading[6];
  const char* mqtt_temperature_topic =
    RtcUserMemoryManager::getInstance().getMqttTemperatureTopic();
  const char* mqtt_humidity_topic =
    RtcUserMemoryManager::getInstance().getMqttHumidityTopic();

  dtostrf(sample_data.humidity, 4, 2, reading);
  Serial.printf("publishing: %s %s\n", mqtt_humidity_topic, reading);
  mqtt_client.publish(mqtt_humidity_topic, reading, true);

  dtostrf(sample_data.temperature, 4, 2, reading);
  Serial.printf("publishing: %s %s\n", mqtt_temperature_topic, reading);
  mqtt_client.publish(mqtt_temperature_topic, reading, true);

  mqtt_client.disconnect();
}

void setup() {
  unsigned long startTime = micros();

  // Connect D0 to RST to wake up
  pinMode(D0, WAKEUP_PULLUP);

  dht.begin();

  Serial.begin(9600);
  Serial.println("\n");

  SampleData sample_data = getSampleData();
  RtcUserMemoryManager::Action action =
    RtcUserMemoryManager::getInstance().getAction();
  if (action == RtcUserMemoryManager::Action::STORE) {
    RtcUserMemoryManager::getInstance().saveSampleData(sample_data);
  } else if (action == RtcUserMemoryManager::Action::PUBLISH) {
    sample_data = averageSampleData(sample_data);
    publish(sample_data);
    RtcUserMemoryManager::getInstance().resetSampleCount();
  }

  RFMode rfMode = RtcUserMemoryManager::getInstance().getSleepMode();

  Serial.println();
  unsigned long sleepLength = max(1, sample_period - (micros() - startTime));
  ESP.deepSleep(sleepLength, rfMode);
}

void loop() {
}
