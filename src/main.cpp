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
  size_t sample_data_count;
} RtcManagementStruct;
RtcManagementStruct rtc_management_data;

typedef struct {
  float temperature;
  float humidity;
} RtcSampleDataStruct;
volatile RtcSampleDataStruct sample_data;

char reading[6];

const int sample_period = (REPORT_PERIOD / SAMPLES_TO_AVG) * 1000000;

bool setup_wifi()
{
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
  mqtt_client_id = new char[rtc_management_data.device_name_length + 3 + 1];
  sprintf(mqtt_client_id, "%s-%02X", rtc_management_data.device_name, id);

  mqtt_temp_topic = new char[rtc_management_data.mqtt_namespace_length + 1 + sizeof(MQTT_TEMP_TOPIC) + 1];
  sprintf(mqtt_temp_topic, "%s/%s", rtc_management_data.mqtt_namespace, MQTT_TEMP_TOPIC);

  mqtt_humidity_topic = new char[rtc_management_data.mqtt_namespace_length + 1 + sizeof(MQTT_HUMIDITY_TOPIC) + 1];
  sprintf(mqtt_humidity_topic, "%s/%s", rtc_management_data.mqtt_namespace, MQTT_HUMIDITY_TOPIC);

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

uint32_t get_sample_offset(size_t index)
{
  return (sizeof(RtcManagementStruct) + sizeof(RtcSampleDataStruct) * index) / 4;
}

void store_sample_data()
{
  uint32_t offset = get_sample_offset(rtc_management_data.sample_data_count);
  ESP.rtcUserMemoryWrite(offset, (uint32_t *) &sample_data, sizeof(sample_data));

  rtc_management_data.sample_data_count++;
  ESP.rtcUserMemoryWrite(0, (uint32_t *) &rtc_management_data, sizeof(rtc_management_data));
}

void average_sample_data()
{
  float temperature = sample_data.temperature;
  float humidity = sample_data.humidity;

  size_t samples = rtc_management_data.sample_data_count + 1;

  for (size_t i = 0; i < rtc_management_data.sample_data_count; i++)
  {
    uint32_t offset = get_sample_offset(i);
    ESP.rtcUserMemoryRead(offset, (uint32_t *) &sample_data, sizeof(sample_data));

    humidity += sample_data.humidity;
    temperature += sample_data.temperature;
  }

  sample_data.temperature = temperature / samples;
  sample_data.humidity = humidity / samples;
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

  Serial.printf("Read #%d humidity=", rtc_management_data.sample_data_count + 1);
  Serial.print(humidity);
  Serial.print("% temperature=");
  Serial.print(temperature);
  Serial.println("˚C");

  sample_data.humidity = humidity;
  sample_data.temperature = temperature;

  if (store)
  {
    store_sample_data();
  }
}

void publish()
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

  average_sample_data();

  if (rtc_management_data.sample_data_count != 0)
  {
    rtc_management_data.sample_data_count = 0;

    if (!ESP.rtcUserMemoryWrite(0, (uint32_t *) &rtc_management_data, sizeof(rtc_management_data)))
    {
      Serial.println("Failed to update sample data counter");
    }
  }

  dtostrf(sample_data.humidity, 4, 2, reading);
  Serial.printf("Publishing: %s %s\n", mqtt_humidity_topic, reading);
  mqttClient.publish(mqtt_humidity_topic, reading, true);

  dtostrf(sample_data.temperature, 4, 2, reading);
  Serial.printf("Publishing: %s %s\n", mqtt_temp_topic, reading);
  mqttClient.publish(mqtt_temp_topic, reading, true);

  mqttClient.disconnect();
}

void init_rtc_mem()
{
  uint8_t mac[6];
  WiFi.macAddress(mac);

  rtc_management_data.device_name_length = sizeof(MQTT_USER) + 6;
  sprintf(rtc_management_data.device_name, "%s-%02X%02X%02X",
          MQTT_USER, mac[3], mac[4], mac[5]);

  rtc_management_data.mqtt_namespace_length = rtc_management_data.device_name_length;
  strcpy(rtc_management_data.mqtt_namespace, rtc_management_data.device_name);

  rtc_management_data.sample_data_count = 0;

  if(!ESP.rtcUserMemoryWrite(0, (uint32_t *) &rtc_management_data, sizeof(rtc_management_data)))
  {
    Serial.println("Failed to write RTC memory management");
  }

  ESP.deepSleep(10, WAKE_RF_DISABLED);
}

void setup()
{
  unsigned long startTime = micros();

  dht.begin();

  Serial.begin(9600);
  Serial.println("\n");

  // Connect D0 to RST to wake up
  pinMode(D0, WAKEUP_PULLUP);

  rst_info *rsti;
  rsti = ESP.getResetInfoPtr();
  if (rsti->reason == REASON_DEEP_SLEEP_AWAKE)
  {
    Serial.println("Woke up from deep sleep");
    if (!ESP.rtcUserMemoryRead(0, (uint32_t *) &rtc_management_data, sizeof(rtc_management_data)))
    {
      Serial.println("Failed to read RTC memory management");
    }
  }
  else
  {
    Serial.println("Normal boot");
    init_rtc_mem();
  }

  RFMode rfMode;
  if (rtc_management_data.sample_data_count < SAMPLES_TO_AVG - 1)
  {
    collect(true);
    rfMode = WAKE_NO_RFCAL;
  }
  else
  {
    collect(false);
    publish();
    rfMode = WAKE_RF_DEFAULT;
  }

  unsigned long sleepLength = sample_period - (micros() - startTime);
  Serial.printf("Sleep for %d microseconds\n\n", sleepLength / 1000);
  ESP.deepSleep(sleepLength, rfMode);
}

void loop()
{
}
