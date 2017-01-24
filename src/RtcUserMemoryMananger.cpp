#include "RtcUserMemoryManager.h"
extern "C" {
#include "user_interface.h"
}

using namespace esp8266climatesensor::rtcstorage;

RtcUserMemoryManager& RtcUserMemoryManager::getInstance() {
  static RtcUserMemoryManager instance;

  return instance;
}

RtcUserMemoryManager::RtcUserMemoryManager() {
  rst_info *rsti;
  rsti = ESP.getResetInfoPtr();
  if (rsti->reason == REASON_DEEP_SLEEP_AWAKE) {
    ESP.rtcUserMemoryRead(0, (uint32_t *) &management_data_, sizeof(management_data_));

    bool save = false;
    if (management_data_.sample_data_count >= SAMPLES_TO_AVG) {
      management_data_.sample_data_count = 0;
      save = true;
    }
    if (management_data_.wifi_boots >= WIFI_RF_CAL_INTERVAL) {
      Serial.println("Resetting wifi boots");
      management_data_.wifi_boots = 0;
      save = true;
    }

    if (save)
      writeRtcMem();
  } else {
    initRtcMem();
  }
}

void RtcUserMemoryManager::initRtcMem() {
  Serial.print("Initializing RTC Memory...");

  uint8_t mac[6];
  WiFi.macAddress(mac);

  management_data_.wifi_boots = 0;
  sprintf(management_data_.device_name, "%s-%02X%02X%02X",
          MQTT_USER, mac[3], mac[4], mac[5]);
  management_data_.mqtt_namespace_length = strlen(management_data_.device_name);
  strcpy(management_data_.mqtt_namespace, management_data_.device_name);
  management_data_.sample_data_count = 0;

  writeRtcMem();
  Serial.println("done.");
}

void RtcUserMemoryManager::printRtcMem() {
  Serial.println("---RTC Memory Info---");
  Serial.printf("Size: %d\n", sizeof(management_data_));
  Serial.printf("              device_name: %s\n", management_data_.device_name);
  Serial.printf("    mqtt_namespace_length: %d\n", management_data_.mqtt_namespace_length);
  Serial.printf("           mqtt_namespace: %s\n", management_data_.mqtt_namespace);
  Serial.printf("        sample_data_count: %d\n", management_data_.sample_data_count);
  Serial.printf("               wifi_boots: %d\n", management_data_.wifi_boots);
}

void RtcUserMemoryManager::writeRtcMem() {
  ESP.rtcUserMemoryWrite(0, (uint32_t *) &management_data_, sizeof(management_data_));
}

RtcUserMemoryManager::Action RtcUserMemoryManager::getAction() {
  if (management_data_.sample_data_count == SAMPLES_TO_AVG - 1) {
    return PUBLISH;
  } else {
    return STORE;
  }
}

RFMode RtcUserMemoryManager::getSleepMode() {
  RFMode mode;
  if (management_data_.sample_data_count == SAMPLES_TO_AVG - 1) {
    management_data_.wifi_boots++;
    Serial.printf("wifi_boots=%d ", management_data_.wifi_boots);
    if (management_data_.wifi_boots < WIFI_RF_CAL_INTERVAL) {
      Serial.println("Wake w/o RF Cal");
      mode = WAKE_NO_RFCAL;
    } else {
      Serial.println("Wake w/ RF Cal");
      management_data_.wifi_boots = 0;
      mode = WAKE_RFCAL;
    }

    writeRtcMem();
  } else {
    mode = WAKE_RF_DISABLED;
  }
  return mode;
}

const char* RtcUserMemoryManager::getMqttClientId() {
  return management_data_.device_name;
}

const char* RtcUserMemoryManager::getMqttWillTopic() {
  if (mqtt_will_topic_ == NULL) {
    mqtt_will_topic_ = new char[management_data_.mqtt_namespace_length + 1 + 3 + 1];
    sprintf(mqtt_will_topic_, "%s/%s", management_data_.mqtt_namespace, "lwt");
  }

  return mqtt_will_topic_;
}

const char* RtcUserMemoryManager::getMqttTemperatureTopic() {
  if (mqtt_temperature_topic_ == NULL) {
    mqtt_temperature_topic_ = new char[management_data_.mqtt_namespace_length + 1 + sizeof(MQTT_TEMP_TOPIC) + 1];
    sprintf(mqtt_temperature_topic_, "%s/%s", management_data_.mqtt_namespace, MQTT_TEMP_TOPIC);
  }

  return mqtt_temperature_topic_;
}

const char* RtcUserMemoryManager::getMqttHumidityTopic() {
  if (mqtt_humidity_topic_ == NULL) {
    mqtt_humidity_topic_ = new char[management_data_.mqtt_namespace_length + 1 + sizeof(MQTT_HUMIDITY_TOPIC) + 1];
    sprintf(mqtt_humidity_topic_, "%s/%s", management_data_.mqtt_namespace, MQTT_HUMIDITY_TOPIC);
  }

  return mqtt_humidity_topic_;
}

const char* RtcUserMemoryManager::getMqttVccTopic() {
  if (mqtt_vcc_topic_ == NULL) {
    mqtt_vcc_topic_ = new char[management_data_.mqtt_namespace_length + 1 + sizeof(MQTT_VCC_TOPIC) + 1];
    sprintf(mqtt_vcc_topic_, "%s/%s", management_data_.mqtt_namespace, MQTT_VCC_TOPIC);
  }

  return mqtt_vcc_topic_;
}

size_t RtcUserMemoryManager::getSampleCount() {
  return management_data_.sample_data_count;
}

void RtcUserMemoryManager::resetSampleCount() {
  management_data_.sample_data_count = 0;
  writeRtcMem();
}

uint32_t RtcUserMemoryManager::getSampleOffset(size_t index) {
  return (sizeof(RtcManagementData) + sizeof(SampleData) * index) / 4;
}

void RtcUserMemoryManager::saveSampleData(SampleData &sample_data) {
  uint32_t offset = getSampleOffset(getSampleCount());
  ESP.rtcUserMemoryWrite(offset, (uint32_t *) &sample_data, sizeof(sample_data));

  management_data_.sample_data_count++;
  writeRtcMem();
}

SampleData RtcUserMemoryManager::getSampleData(size_t index) {
  SampleData sample_data;

  uint32_t offset = getSampleOffset(index);
  ESP.rtcUserMemoryRead(offset, (uint32_t *) &sample_data, sizeof(sample_data));

  return sample_data;
}
