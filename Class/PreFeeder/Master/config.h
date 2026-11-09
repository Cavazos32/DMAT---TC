#pragma once

#include <Arduino.h>
#include <WiFi.h>

// Índice con enlaces a L (.101) y R (.102).
static const char* const PF_MASTER_WIFI_SSID = "R&D_TCM";
static const char* const PF_MASTER_WIFI_PASS = "TCM2026!r&d";
static const IPAddress PF_MASTER_STA_IP(10, 10, 32, 100);
static const IPAddress PF_MASTER_STA_GW(10, 10, 32, 72);
static const IPAddress PF_MASTER_STA_MASK(255, 255, 255, 0);

#define OTA_HOSTNAME "prefeeder-master"
