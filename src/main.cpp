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
const char* mqtt_client_id;
const char* mqtt_temp_topic;
const char* mqtt_humidity_topic;

#define DHTPIN D2       // Temp sensor data pin
#define DHTTYPE DHT22   // Temp sensor type
// Initialize DHT sensor.
DHT dht(DHTPIN, DHTTYPE);

SampleData sample_data;

const int sample_period = (REPORT_PERIOD / SAMPLES_TO_AVG) * 1000000;

inline int max(int a,int b) {return ((a)>(b)?(a):(b)); }

bool setupWiFi()
{
  Serial.print("Connecting to ");
  Serial.println(wifi_ssid);

  WiFi.begin(wifi_ssid, wifi_password);

  unsigned long start = millis();
  while (!WiFi.isConnected() && (millis() - start) < WIFI_CONNECT_TIMEOUT_MS)
  {
    delay(500);
    Serial.print(".");
  }
  Serial.println();

  if (WiFi.isConnected())
  {
    Serial.println("WiFi connected");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
    return true;
  }
  else
  {
    Serial.println("WiFi failed to connect");
    return false;
  }
}

bool setupMqtt()
{
  mqtt_client_id = RtcUserMemoryManager::getInstance().getMqttClientId();
  mqtt_temp_topic = RtcUserMemoryManager::getInstance().getMqttTemperatureTopic();
  mqtt_humidity_topic = RtcUserMemoryManager::getInstance().getMqttHumidityTopic();

  Serial.print("MQTT server: ");
  Serial.print(mqtt_server_host);
  Serial.print(":");
  Serial.println(mqtt_server_port);
  Serial.print("MQTT temp topic: ");
  Serial.println(mqtt_temp_topic);
  Serial.print("MQTT humidity topic: ");
  Serial.println(mqtt_humidity_topic);
  Serial.print("MQTT ClientID: ");
  Serial.println(mqtt_client_id);

  mqtt_client.setServer(mqtt_server_host, mqtt_server_port);

  Serial.print("Attempting MQTT connection...");
  if (mqtt_client.connect(mqtt_client_id, mqtt_user, mqtt_password)) {
    Serial.println("connected");

#ifdef MQTT_CERT_FINGERPRINT
    if (!wifiClient.verify(mqtt_cert_fingerprint, mqtt_server_host)) {
      Serial.println("Certificate doesn't match");
      mqtt_client.disconnect();
    }
#endif
  } else {
    Serial.print("failed, rc=");
    Serial.println(mqtt_client.state());
  }

  return mqtt_client.connected();
}

void averageSampleData() {
  float temperature = sample_data.temperature;
  float humidity = sample_data.humidity;

  size_t samples = RtcUserMemoryManager::getInstance().getSampleCount();

  for (size_t i = 0; i < samples; i++)
  {
    sample_data = RtcUserMemoryManager::getInstance().getSampleData(i);

    humidity += sample_data.humidity;
    temperature += sample_data.temperature;
  }

  sample_data.temperature = temperature / (samples + 1);
  sample_data.humidity = humidity / (samples + 1);
}

void collect(bool store)
{
  // Reading temperature or humidity takes about 250 milliseconds!
  // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
  float humidity = dht.readHumidity();

  // Read temperature as Celsius (the default)
  float temperature = dht.readTemperature();


  if (isnan(humidity) || isnan(temperature))
  {
    Serial.println("Failed to read temperature and humidity from sensor");
    return;
  }

  Serial.printf("Read #%d humidity=", RtcUserMemoryManager::getInstance().getSampleCount() + 1);
  Serial.print(humidity);
  Serial.print("% temperature=");
  Serial.print(temperature);
  Serial.println("˚C");

  sample_data.humidity = humidity;
  sample_data.temperature = temperature;

  if (store)
  {
    RtcUserMemoryManager::getInstance().saveSampleData(sample_data);
  }
}

void publish()
{
  if (!setupWiFi())
  {
    Serial.println("Failed to setup WiFi");
    return;
  }

  if (!setupMqtt())
  {
    Serial.println("Failed to setup MQTT");
    return;
  }

  averageSampleData();

  char reading[6];

  dtostrf(sample_data.humidity, 4, 2, reading);
  Serial.printf("publishing: %s %s\n", mqtt_humidity_topic, reading);
  mqtt_client.publish(mqtt_humidity_topic, reading, true);

  dtostrf(sample_data.temperature, 4, 2, reading);
  Serial.printf("publishing: %s %s\n", mqtt_temp_topic, reading);
  mqtt_client.publish(mqtt_temp_topic, reading, true);

  mqtt_client.disconnect();
}

void setup()
{
  unsigned long startTime = micros();

  dht.begin();

  Serial.begin(9600);
  Serial.println("\n");

  // Connect D0 to RST to wake up
  pinMode(D0, WAKEUP_PULLUP);

  RtcUserMemoryManager::Action action = RtcUserMemoryManager::getInstance().getAction();
  if (action == RtcUserMemoryManager::Action::COLLECT)
  {
    collect(true);
  }
  else if (action == RtcUserMemoryManager::Action::PUBLISH)
  {
    collect(false);
    publish();
    RtcUserMemoryManager::getInstance().resetSampleCount();
  }

  RFMode rfMode = RtcUserMemoryManager::getInstance().getSleepMode();

  Serial.println();

  unsigned long sleepLength = sample_period - (micros() - startTime);
  ESP.deepSleep(sleepLength, rfMode);
}

void loop() {
}
