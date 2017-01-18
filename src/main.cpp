#include <Arduino.h>

#include <ESP8266WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <DHT.h>

extern "C" {
#include "user_interface.h"
}

#include "config.h"
#include "defaults.h"

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

PubSubClient mqttClient(wifiClient);
char* mqtt_client_id;
char* mqtt_temp_topic;
char* mqtt_humidity_topic;

#define DHTPIN D2       // Temp sensor data pin
#define DHTTYPE DHT22   // Temp sensor type
// Initialize DHT sensor.
DHT dht(DHTPIN, DHTTYPE);

typedef struct {
  size_t device_name_length;
  char device_name[sizeof(MQTT_USER) + 1 + 6 + 1];
  size_t mqtt_namespace_length;
  char mqtt_namespace[128];
} RtcDataStruct;
RtcDataStruct rtcData;

char reading[6];

const int sample_period = SAMPLE_PERIOD * 1000000;

bool setup_wifi()
{
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(wifi_ssid);

  WiFi.begin(wifi_ssid, wifi_password);

  unsigned long timeout = 0;
  while (WiFi.status() != WL_CONNECTED && timeout < WIFI_CONNECT_TIMEOUT_MS)
  {
    delay(500);
    timeout += 500;
    Serial.print(".");
  }
  Serial.println();

  if (WiFi.status() == WL_CONNECTED)
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

bool setup_mqtt()
{
  const unsigned long id = micros() & 0xff;
  mqtt_client_id = new char[rtcData.device_name_length + 3 + 1];
  sprintf(mqtt_client_id, "%s-%02X", rtcData.device_name, id);

  mqtt_temp_topic = new char[rtcData.mqtt_namespace_length + 1 + sizeof(MQTT_TEMP_TOPIC) + 1];
  sprintf(mqtt_temp_topic, "%s/%s", rtcData.mqtt_namespace, MQTT_TEMP_TOPIC);

  mqtt_humidity_topic = new char[rtcData.mqtt_namespace_length + 1 + sizeof(MQTT_HUMIDITY_TOPIC) + 1];
  sprintf(mqtt_humidity_topic, "%s/%s", rtcData.mqtt_namespace, MQTT_HUMIDITY_TOPIC);

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

  mqttClient.setServer(mqtt_server_host, mqtt_server_port);

  Serial.print("Attempting MQTT connection...");
  if (mqttClient.connect(mqtt_client_id, mqtt_user, mqtt_password))
  {
    Serial.println("connected");

#ifdef MQTT_CERT_FINGERPRINT
    if (wifiClient.verify(mqtt_cert_fingerprint, mqtt_server_host))
    {
      Serial.println("Certificate matches");
    }
    else
    {
      Serial.println("Certificate doesn't match");
      mqttClient.disconnect();
    }
#endif
  }
  else
  {
    Serial.print("failed, rc=");
    Serial.println(mqttClient.state());
  }

  return mqttClient.connected();
}

void publish_sensor_info()
{
  // Reading temperature or humidity takes about 250 milliseconds!
  // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
  float h = dht.readHumidity();

  // Read temperature as Celsius (the default)
  float t = dht.readTemperature();


  if (isnan(h) || isnan(t))
  {
    Serial.println("Failed to read temperature and humidity from sensor");
    return;
  }

  dtostrf(h, 4, 2, reading);
  Serial.printf("Publishing: %s %s\n", mqtt_humidity_topic, reading);
  mqttClient.publish(mqtt_humidity_topic, reading, true);

  dtostrf(t, 4, 2, reading);
  Serial.printf("Publishing: %s %s\n", mqtt_temp_topic, reading);
  mqttClient.publish(mqtt_temp_topic, reading, true);
}

void connect_and_publish()
{
  if (!setup_wifi())
  {
    Serial.println("Failed to setup WiFi");
    return;
  }

  if (!setup_mqtt())
  {
    Serial.println("Failed to setup MQTT");
    return;
  }

  publish_sensor_info();

  mqttClient.disconnect();
}

void init_rtc_mem()
{
  uint8_t mac[6];
  WiFi.macAddress(mac);

  rtcData.device_name_length = sizeof(MQTT_USER) + 6;
  sprintf(rtcData.device_name, "%s-%02X%02X%02X",
          MQTT_USER, mac[3], mac[4], mac[5]);

  rtcData.mqtt_namespace_length = rtcData.device_name_length;
  strcpy(rtcData.mqtt_namespace, rtcData.device_name);

  ESP.rtcUserMemoryWrite(0, (uint32_t *) &rtcData, sizeof(rtcData));
}

void setup()
{
  unsigned long startTime = micros();

  dht.begin();

  Serial.begin(9600);
  Serial.println("\n\nWake up");

  // Connect D0 to RST to wake up
  pinMode(D0, WAKEUP_PULLUP);

  rst_info *rsti;
  rsti = ESP.getResetInfoPtr();
  if (rsti->reason == REASON_DEEP_SLEEP_AWAKE)
  {
    Serial.println("Woke up from deep sleep");
    ESP.rtcUserMemoryRead(0, (uint32_t *) &rtcData, sizeof(rtcData));
  }
  else
  {
    Serial.println("Normal boot");
    init_rtc_mem();
  }

  connect_and_publish();

  unsigned long sleepLength = sample_period - (micros() - startTime);
  Serial.printf("Sleep for %d microseconds\n\n", sleepLength / 1000);
  ESP.deepSleep(sleepLength);
}

void loop()
{
}
