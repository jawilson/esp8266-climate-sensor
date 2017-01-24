#ifndef __esp8266_dht22_mqtt_defaults_h
#define __esp8266_dht22_mqtt_defaults_h

#ifndef REPORT_PERIOD
#define REPORT_PERIOD (5 * 60)
#endif

#ifndef SAMPLES_TO_AVG
#define SAMPLES_TO_AVG 10
#endif

#ifndef WIFI_CONNECT_TIMEOUT_MS
#define WIFI_CONNECT_TIMEOUT_MS 5000
#endif

#ifndef WIFI_RF_CAL_INTERVAL
#define WIFI_RF_CAL_INTERVAL 8
#endif

#ifndef MQTT_SERVER_PORT
#define MQTT_SERVER_PORT 1883
#endif

#ifndef MQTT_TEMP_TOPIC
#define MQTT_TEMP_TOPIC "temperature"
#endif

#ifndef MQTT_HUMIDITY_TOPIC
#define MQTT_HUMIDITY_TOPIC "humidity"
#endif

#ifndef MQTT_VCC_TOPIC
#define MQTT_VCC_TOPIC "vcc"
#endif

#endif //__esp8266_dht22_mqtt_defaults_h
