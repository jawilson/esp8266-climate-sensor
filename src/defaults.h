#ifndef __esp8266_dht22_mqtt_defaults_h
#define __esp8266_dht22_mqtt_defaults_h

#ifndef SAMPLE_PERIOD
#define SAMPLE_PERIOD 5 * 60;
#endif

#ifndef WIFI_CONNECT_TIMEOUT_MS
#define WIFI_CONNECT_TIMEOUT_MS 5000
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

#endif //__esp8266_dht22_mqtt_defaults_h
