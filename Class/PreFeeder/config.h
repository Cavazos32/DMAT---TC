#pragma once

#include <Arduino.h>

// Lado: comentar la línea siguiente para L, descomentar para R (misma base de firmware).
#define PF_SIDE_RIGHT

#ifndef PF_CONFIG_SIDE
#if defined(PF_SIDE_RIGHT)
#define PF_CONFIG_SIDE 2
#else
#define PF_CONFIG_SIDE 1
#endif
#endif

#if PF_CONFIG_SIDE == 2
#define PF_SIDE_TAG "R"
#define PF_STA_IP_0 10
#define PF_STA_IP_1 10
#define PF_STA_IP_2 32
#define PF_STA_IP_3 102
constexpr uint16_t PF_SERVO_ACTIVE_US = 2000;
#define PF_HOSTNAME "prefeeder-R"
#else
#define PF_SIDE_TAG "L"
#define PF_STA_IP_0 10
#define PF_STA_IP_1 10
#define PF_STA_IP_2 32
#define PF_STA_IP_3 101
constexpr uint16_t PF_SERVO_ACTIVE_US = 800;
#define PF_HOSTNAME "prefeeder-L"
#endif

// HTTP hacia el maestro (.100).
static const char* const WIFI_SSID = "R&D_TCM";
static const char* const WIFI_PASS = "TCM2026!r&d";
#define PF_MASTER_HOST "10.10.32.100"
#define STA_GW_0 10
#define STA_GW_1 10
#define STA_GW_2 32
#define STA_GW_3 72
#define STA_MASK_0 255
#define STA_MASK_1 255
#define STA_MASK_2 255
#define STA_MASK_3 0

// GPIO. Buffer y DeReeler quedan fuera de este firmware.
constexpr uint8_t PIN_CYLINDER = 25;
constexpr uint8_t PIN_HOSE = 27;          // HIGH = manguera/cinta ausente
constexpr int PIN_FEEDER_STEP = 18;
constexpr uint8_t PIN_HOLGURA = 22;       // HIGH = holgura OK (ON)
constexpr uint8_t HOLGURA_ON_LEVEL = HIGH;
constexpr uint8_t PIN_SERVO = 26;

// Botón físico JOG - L / JOG - R (según PF_SIDE_TAG). Activo en LOW.
// Solo cambia este número de GPIO para cablear otro pin.
constexpr uint8_t PIN_JOG = 33;
constexpr uint8_t JOG_ACTIVE_LEVEL = LOW;
constexpr uint32_t JOG_STABLE_MS = 20;

constexpr uint16_t SERVO_MIN_US = 500;
constexpr uint16_t SERVO_MAX_US = 2500;
constexpr uint16_t SERVO_NEUTRAL_US = 1500;
constexpr uint8_t SERVO_LEDC_CH = 7;
constexpr uint8_t SERVO_LEDC_BITS = 14;

// FeedMotor. El maestro puede reemplazar FEED_RPM en marcha.
constexpr uint16_t MOTOR_MICROSTEP = 16;
constexpr float MOTOR_ACCEL = 3000.0f;
constexpr float MOTOR_DECEL = 3000.0f;
constexpr float RPM_MIN = 1.0f;
constexpr float RPM_MAX = 600.0f;
constexpr float FEED_RPM = 60.0f;

// SensorMonitor
constexpr uint32_t HOLGURA_ON_STABLE_MS = 50;
constexpr uint32_t HOLGURA_OFF_STABLE_MS = 100;

// FeedingMode. Valores hasta que el maestro manda los suyos.
constexpr uint16_t HOLGURA_SETTLE_MS = 150;
constexpr uint16_t HOLGURA_SETTLE_MS_MIN = 10;
constexpr uint16_t HOLGURA_SETTLE_MS_MAX = 500;
constexpr float HOLGURA_ON_FEED_SEC = 2.0f;
constexpr float HOLGURA_OFF_FAULT_SEC = 5.0f;
constexpr float HOLGURA_STALL_FAULT_SEC = 2.0f;
