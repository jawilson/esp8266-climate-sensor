#ifndef ESP8266_CLIMATE_SENSOR_RTC_USER_MEMORY_MANAGER_H_
#define ESP8266_CLIMATE_SENSOR_RTC_USER_MEMORY_MANAGER_H_

#include <ESP8266WiFi.h>
#include <stddef.h>
#include "config.h"

namespace esp8266climatesensor {
  namespace rtcstorage {

    struct SampleData {
      float temperature;
      float humidity;
    };

    class RtcUserMemoryManager {
     public:
      enum Action {
        COLLECT = 0,
        PUBLISH
      };

      static RtcUserMemoryManager& getInstance();
      Action getAction();
      RFMode getSleepMode();
      const char* getMqttClientId();
      const char* getMqttTemperatureTopic();
      const char* getMqttHumidityTopic();
      size_t getSampleCount();
      void resetSampleCount();
      void saveSampleData(SampleData &sample_data);
      SampleData getSampleData(size_t index);

     private:
      struct RtcManagementData {
        size_t device_name_length;
        char device_name[sizeof(MQTT_USER) + 1 + 6 + 1];
        size_t mqtt_namespace_length;
        char mqtt_namespace[128];
        size_t sample_data_count;
        size_t wifi_boots;
      };

      RtcUserMemoryManager();
      void initRtcMem();
      void printRtcMem();
      void writeRtcMem();
      uint32_t getSampleOffset(size_t index);

      RtcManagementData management_data_;
      char* mqtt_client_id_;
      char* mqtt_temperature_topic_;
      char* mqtt_humidity_topic_;
    };
  } // namespace storage
} // esp8266climatesensor

#endif //ESP8266_CLIMATE_SENSOR_RTC_USER_MEMORY_MANAGER_H_
