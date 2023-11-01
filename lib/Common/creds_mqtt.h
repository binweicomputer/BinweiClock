#ifndef CREDS_MQTT_H
#define CREDS_MQTT_H

//#include <ESP8266WiFi.h>
#ifdef MQTT_USE_SSL
#include "server_mqtt.crt.h"
#include "client.crt.h"
#include "client.key.h"
#endif

//  Home
#define WIFI_SSID "BinweiComputer"
#define WIFI_PASSWORD "5Astudy.com"

#define MQTT_SERVER "home.binweicomputer.top"
#define MQTT_PORT 1883
#define MQTT_USERNAME "rains"
#define MQTT_PASSWORD "fuliai"

#define MQTT_HOSTNAME "esp-hostname"
#define MQTT_CLIENT_ID MQTT_HOSTNAME

#define MQTT_TEMPERATURE_SENSOR_TOPIC MQTT_CLIENT_ID "/sensor/temperature"
#define MQTT_HUMIDITY_SENSOR_TOPIC    MQTT_CLIENT_ID "/sensor/humidity"
#define MQTT_STATUS_TOPIC             MQTT_CLIENT_ID "/state"
#define MQTT_UPDATE_CMD_TOPIC         MQTT_CLIENT_ID "/update/req"

#define MQTT_REPORT_INTERVAL_MILLIS 30000

#define NTP_SERVER "cn.ntp.org.cn"

#define OTA_URL "http://10.10.10.10:8080/" MQTT_HOSTNAME ".bin"

#endif
