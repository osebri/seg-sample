#pragma once

#if __has_include("blynk_config_local.h")
#include "blynk_config_local.h"
#endif

#ifndef BLYNK_ENABLE
#define BLYNK_ENABLE 1
#endif

#ifndef BLYNK_TEMPLATE_ID
#define BLYNK_TEMPLATE_ID ""
#endif

#ifndef BLYNK_TEMPLATE_NAME
#define BLYNK_TEMPLATE_NAME "BabyMonitor"
#endif

#ifndef BLYNK_AUTH_TOKEN
#define BLYNK_AUTH_TOKEN ""
#endif

#ifndef BLYNK_WIFI_SSID
#define BLYNK_WIFI_SSID ""
#endif

#ifndef BLYNK_WIFI_PASSWORD
#define BLYNK_WIFI_PASSWORD ""
#endif

#ifndef BLYNK_SERVER_HOST
#define BLYNK_SERVER_HOST "blynk.cloud"
#endif

#ifndef BLYNK_SERVER_PORT
#define BLYNK_SERVER_PORT 80
#endif

#ifndef BLYNK_LOOP_PERIOD_MS
#define BLYNK_LOOP_PERIOD_MS 100
#endif

#ifndef BLYNK_WIFI_CONNECT_TIMEOUT_MS
#define BLYNK_WIFI_CONNECT_TIMEOUT_MS 15000
#endif

#define BLYNK_VPIN_ROOM_TEMP 0
#define BLYNK_VPIN_BODY_TEMP 1
#define BLYNK_VPIN_HEART_RATE 2
#define BLYNK_VPIN_SMOKE 3
#define BLYNK_VPIN_CRY 6
#define BLYNK_VPIN_HUMIDITY 7
#define BLYNK_VPIN_SPO2 9
#define BLYNK_VPIN_ACCEL_X 10
#define BLYNK_VPIN_ACCEL_Y 11
#define BLYNK_VPIN_ACCEL_Z 12
