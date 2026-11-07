#pragma once

/*
 * Parameters.h — fuente única de parámetros MCU Motion
 *
 * [A] Fijos en firmware (rules.md / banco mecánico) — no Motion.json
 * [B] Motion.json + set_tune (RAM en ESP32)
 * [C] Datos de ciclo + set_data / modelo (NVS en ESP32)
 *
 * Claves JSON (data/Motion.json) ↔ tune:
 *   feeder.R/L.profile_velocity_pps, target_position_counts, compensation_mm
 *   lmguide.pr_speed_rpm, pos_puu
 *   set_data: target_length_mm, tolerance_mm, modelo_id, measure_comp_mm
 *
 * FeedStage: timeout láser fijo PARAM_FEED_STAGE_LASER_TIMEOUT_MS;
 * velocidad de alimentación = feeder.R/L.profile_velocity_pps (mismo bloque CAN).
 */

#include <stddef.h>
#include <stdint.h>
#include <string.h>

// ── [A] Ciclo automático (rules.md §6.7) ────────────────────────────────────

static const uint32_t PARAM_MS_HOLDER_ENC_STABLE   = 100;
static const uint32_t PARAM_MS_LM_AT_CUT          = 100;
static const uint32_t PARAM_MS_LM_AT_DISCHARGE    = 100;
static const uint32_t PARAM_MS_CUTTER_ON          = 50;
static const uint32_t PARAM_MS_AFTER_CUTTER       = 100;
static const uint32_t PARAM_MS_AFTER_GRIP_RELEASE = 100;
static const uint32_t PARAM_MS_BLOWER_DELAY       = 200;
static const uint32_t PARAM_MS_AT_HOME_END        = 50;

static const float PARAM_DATA_TOLERANCE_MM_DEFAULT = 2.0f;

// ── [A] Encoder OM + láser ──────────────────────────────────────────────────

static const uint16_t PARAM_ENC_PPR               = 2000;
static const uint8_t  PARAM_ENC_QUAD              = 1;
static const float    PARAM_ENC_PULLEY_DIAM_MM    = 50.0f;
static const int32_t  PARAM_ENC_COUNTS_PER_100MM  = 1273;
static const float    PARAM_ENC_MOVE_MMS          = 20.0f;
static const uint32_t PARAM_ENC_SETTLE_MS         = 250;
static const uint32_t PARAM_ENC_POLL_MS           = 80;
static const int16_t  PARAM_ENC_PCNT_HIGH_LIM     = 30000;
static const int16_t  PARAM_ENC_PCNT_LOW_LIM      = -30000;
static const uint16_t PARAM_ENC_FILTER_APB_TICKS  = 1023;

static const int  PARAM_PIN_ENC_A_R = 18;
static const int  PARAM_PIN_ENC_B_R = 19;
static const int  PARAM_PIN_ENC_Z_R = 21;
static const int  PARAM_PIN_ENC_A_L = 22;
static const int  PARAM_PIN_ENC_B_L = 23;
static const int  PARAM_PIN_ENC_Z_L = 25;

static const int   PARAM_PIN_LASER_R        = 32;
static const int   PARAM_PIN_LASER_L        = 34;
static const bool  PARAM_LASER_ACTIVE_HIGH  = false;
static const int   PARAM_PIN_SAFETY         = 26;
static const float PARAM_ENC_OFFSET_MM        = 0.0f;

#define PARAM_ENC_COUNTS_TO_MM(counts) \
  ((float)(counts) * (100.0f / (float)PARAM_ENC_COUNTS_PER_100MM))

// ── [A] FeedStage (solo firmware) ─────────────────────────────────────────

static const uint32_t PARAM_FEED_STAGE_LASER_TIMEOUT_MS = 4000u;
static const bool     PARAM_FEED_VALIDATE_ENCODER       = true;

// ── [A] Feeder CAN (CiA402) ────────────────────────────────────────────────

static const int      PARAM_CAN_TX_GPIO           = 4;
static const int      PARAM_CAN_RX_GPIO           = 5;
static const uint32_t PARAM_CAN_BITRATE_KBPS      = 500;
static const uint8_t  PARAM_CAN_NODE_R            = 1;
static const uint8_t  PARAM_CAN_NODE_L            = 2;
static const uint8_t  PARAM_CAN_RX_QUEUE_LEN      = 32;
static const uint8_t  PARAM_CAN_TX_QUEUE_LEN      = 16;
static const uint8_t  PARAM_CAN_INIT_MAX_RETRIES  = 3;
static const uint32_t PARAM_CAN_INIT_RETRY_MS     = 400;
static const uint32_t PARAM_CAN_SERVO_POWERUP_MS  = 1000;
static const uint32_t PARAM_CAN_FAULT_RESET_MS    = 300;
static const uint32_t PARAM_CAN_NMT_PREOP_MS      = 500;
static const uint32_t PARAM_CAN_NMT_OPERATIONAL_MS = 500;
static const uint32_t PARAM_CAN_CIA402_STEP_MS    = 200;
static const uint32_t PARAM_CAN_CIA402_SHORT_MS   = 50;

static const uint32_t PARAM_FEED_ENC_INC_PER_REV  = 131072u;
static const int8_t   PARAM_FEED_CMD_SIGN_R       = 1;
static const int8_t   PARAM_FEED_CMD_SIGN_L       = -1;

static const uint32_t PARAM_FEED_TARGET_COUNTS_DEFAULT = 42500u;
static const uint32_t PARAM_FEED_PROFILE_VEL_DEFAULT   = 42500u;
static const uint32_t PARAM_FEED_PROFILE_ACCEL_PPS2      = 800000u;
static const uint32_t PARAM_FEED_PROFILE_DECEL_PPS2      = 800000u;
static const int32_t  PARAM_FEED_JOG_STEP_COUNTS         = 5000;
static const uint32_t PARAM_FEED_MAX_RPM                 = 3000u;
static const uint8_t  PARAM_FEED_HALT_OPTION_CODE        = 2;
static const uint32_t PARAM_FEED_HALT_DECEL_PP           = 8000000u;

static const uint32_t PARAM_FEED_MAX_PPS =
    (uint32_t)((uint64_t)PARAM_FEED_MAX_RPM * PARAM_FEED_ENC_INC_PER_REV / 60u);

// ── [A] LM Guide Modbus (ASDA-B3) ─────────────────────────────────────────

static const uint8_t  PARAM_MODBUS_SLAVE_ID       = 0x01;
static const uint32_t PARAM_MODBUS_BAUD           = 9600;
static const int      PARAM_PIN_MODBUS_RX         = 16;
static const int      PARAM_PIN_MODBUS_TX         = 17;

static const float    PARAM_LM_PR_SPEED_RPM       = 1200.0f;
static const float    PARAM_LM_PR_SPEED_RPM_MIN   = 400.0f;
static const float    PARAM_LM_PR_SPEED_RPM_MAX   = 3000.0f;
static const int32_t  PARAM_LM_JOG_DELTA_PUU      = 2000;
static const uint32_t PARAM_LM_SERVO_ON_SETTLE_MS = 2000;

// Validación LM Guide (ASDA PUU vs objetivo; fábrica ServoLMGuide.h [7])
static const float    PARAM_LM_MOVE_TOLERANCE_MM  = 0.5f;
static const float    PARAM_LM_HOME_TOLERANCE_MM  = 1.0f;
static const float    PARAM_LM_HOME_MAX_TRAVEL_MM = 120.0f;
static const uint32_t PARAM_LM_MOVE_DONE_TIMEOUT_MS = 45000;
static const uint8_t  PARAM_LM_MOVE_MAX_ATTEMPTS  = 3;

static const uint16_t PARAM_LM_HOME_TORQUE_PCT    = 10;
static const uint16_t PARAM_LM_HOME_TIME_MS       = 150;
static const float    PARAM_LM_HOME_SPEED_RPM     = 10.0f;
static const uint16_t PARAM_LM_HOME_METHOD_FWD    = 0x0029;
static const uint16_t PARAM_LM_HOME_METHOD_REV    = 0x002A;
static const uint32_t PARAM_LM_HOME_DONE_TIMEOUT_MS = 30000;

static const uint32_t PARAM_SERIAL_BAUD         = 115200;

// ── [B] Motion.json / set_tune (RAM) ───────────────────────────────────────

enum MotionSide : uint8_t { MOTION_SIDE_R = 0, MOTION_SIDE_L = 1, MOTION_SIDE_COUNT = 2 };

struct MotionFeederSideTune {
  uint32_t profileVelocityPps;
  int32_t targetPositionCounts;
  float compensationMm;
};

struct MotionLMGuideTune {
  float prSpeedRpm;
  int32_t posPuu;
};

struct MotionTune {
  MotionFeederSideTune feeder[MOTION_SIDE_COUNT];
  MotionLMGuideTune lmguide;
  bool validateEncoder;
};

// ── [C] set_data / modelo (NVS) ────────────────────────────────────────────

struct MotionData {
  float targetLengthMm;
  float toleranceMm;
  float measureCompMm;
  char modeloId[24];
};

static inline void motionTuneSetDefaults(MotionTune* tune) {
  if (!tune) return;
  memset(tune, 0, sizeof(MotionTune));
  for (int i = 0; i < MOTION_SIDE_COUNT; i++) {
    tune->feeder[i].profileVelocityPps = PARAM_FEED_PROFILE_VEL_DEFAULT;
    tune->feeder[i].targetPositionCounts = (int32_t)PARAM_FEED_TARGET_COUNTS_DEFAULT;
  }
  tune->lmguide.prSpeedRpm = PARAM_LM_PR_SPEED_RPM;
  tune->lmguide.posPuu = 0;
  tune->validateEncoder = PARAM_FEED_VALIDATE_ENCODER;
}

static inline void motionDataSetDefaults(MotionData* data) {
  if (!data) return;
  memset(data, 0, sizeof(MotionData));
  data->toleranceMm = PARAM_DATA_TOLERANCE_MM_DEFAULT;
}

// ── Alias compatibilidad (Encoder.h / ServoFeeder.h / ServoLMGuide.h) ───────

#define ENC_OM_PPR              PARAM_ENC_PPR
#define ENC_OM_QUAD             PARAM_ENC_QUAD
#define ENC_OM_CPR              (PARAM_ENC_PPR * PARAM_ENC_QUAD)
#define ENC_OM_PULLEY_DIAM_MM   PARAM_ENC_PULLEY_DIAM_MM
#define ENC_OM_COUNTS_PER_100MM PARAM_ENC_COUNTS_PER_100MM
#define ENC_OM_MOVE_MMS         PARAM_ENC_MOVE_MMS
#define ENC_OM_SETTLE_MS        PARAM_ENC_SETTLE_MS
#define ENC_OM_PCNT_HIGH_LIM    PARAM_ENC_PCNT_HIGH_LIM
#define ENC_OM_PCNT_LOW_LIM     PARAM_ENC_PCNT_LOW_LIM
#define ENC_OM_FILTER_APB_TICKS PARAM_ENC_FILTER_APB_TICKS
#define ENC_OM_PIN_A_R          PARAM_PIN_ENC_A_R
#define ENC_OM_PIN_B_R          PARAM_PIN_ENC_B_R
#define ENC_OM_PIN_Z_R          PARAM_PIN_ENC_Z_R
#define ENC_OM_PIN_A_L          PARAM_PIN_ENC_A_L
#define ENC_OM_PIN_B_L          PARAM_PIN_ENC_B_L
#define ENC_OM_PIN_Z_L          PARAM_PIN_ENC_Z_L
#define LASER_PIN_R             PARAM_PIN_LASER_R
#define LASER_PIN_L             PARAM_PIN_LASER_L
#define LASER_ACTIVE_HIGH       PARAM_LASER_ACTIVE_HIGH
#define SAFETY_PIN              PARAM_PIN_SAFETY
#define ENC_OM_COUNTS_TO_MM     PARAM_ENC_COUNTS_TO_MM

#define SERVO_CAN_TX_GPIO           PARAM_CAN_TX_GPIO
#define SERVO_CAN_RX_GPIO           PARAM_CAN_RX_GPIO
#define SERVO_CAN_BITRATE_KBPS      PARAM_CAN_BITRATE_KBPS
#define SERVO_CAN_NODE_R            PARAM_CAN_NODE_R
#define SERVO_CAN_NODE_L            PARAM_CAN_NODE_L
#define SERVO_CAN_RX_QUEUE_LEN      PARAM_CAN_RX_QUEUE_LEN
#define SERVO_CAN_TX_QUEUE_LEN      PARAM_CAN_TX_QUEUE_LEN
#define SERVO_CAN_INIT_MAX_RETRIES  PARAM_CAN_INIT_MAX_RETRIES
#define SERVO_CAN_INIT_RETRY_MS     PARAM_CAN_INIT_RETRY_MS
#define SERVO_CAN_POWERUP_MS        PARAM_CAN_SERVO_POWERUP_MS
#define SERVO_CAN_FAULT_RESET_MS    PARAM_CAN_FAULT_RESET_MS
#define SERVO_CAN_NMT_PREOP_MS      PARAM_CAN_NMT_PREOP_MS
#define SERVO_CAN_NMT_OPERATIONAL_MS PARAM_CAN_NMT_OPERATIONAL_MS
#define SERVO_CAN_CIA402_STEP_MS    PARAM_CAN_CIA402_STEP_MS
#define SERVO_CAN_CIA402_SHORT_MS   PARAM_CAN_CIA402_SHORT_MS
#define SERVO_CAN_ENC_INC_PER_REV   PARAM_FEED_ENC_INC_PER_REV
#define SERVO_CAN_CMD_SIGN_R        PARAM_FEED_CMD_SIGN_R
#define SERVO_CAN_CMD_SIGN_L        PARAM_FEED_CMD_SIGN_L
#define SERVO_CAN_HOME_PP           PARAM_FEED_TARGET_COUNTS_DEFAULT
#define SERVO_CAN_PROFILE_ACCEL_PPS2 PARAM_FEED_PROFILE_ACCEL_PPS2
#define SERVO_CAN_PROFILE_DECEL_PPS2 PARAM_FEED_PROFILE_DECEL_PPS2
#define SERVO_CAN_JOG_STEP_COUNTS   PARAM_FEED_JOG_STEP_COUNTS
#define SERVO_CAN_MAX_RPM           PARAM_FEED_MAX_RPM
#define SERVO_CAN_MAX_PPS           PARAM_FEED_MAX_PPS
#define SERVO_CAN_HALT_OPTION_CODE  PARAM_FEED_HALT_OPTION_CODE
#define SERVO_CAN_HALT_DECEL_PP     PARAM_FEED_HALT_DECEL_PP

#define MODBUS_ASDA_SLAVE_ID      PARAM_MODBUS_SLAVE_ID
#define MODBUS_ASDA_BAUD          PARAM_MODBUS_BAUD
#define ASDA_PR_SPEED_RPM         PARAM_LM_PR_SPEED_RPM
#define ASDA_JOG_DELTA_PUU        PARAM_LM_JOG_DELTA_PUU
#define ASDA_SERVO_ON_SETTLE_MS   PARAM_LM_SERVO_ON_SETTLE_MS
#define ASDA_HOME_TORQUE_PCT_DEFAULT PARAM_LM_HOME_TORQUE_PCT
#define ASDA_HOME_TIME_MS_DEFAULT    PARAM_LM_HOME_TIME_MS
#define ASDA_HOME_SPEED_RPM_DEFAULT  PARAM_LM_HOME_SPEED_RPM
#define ASDA_HOME_METHOD_FORWARD     PARAM_LM_HOME_METHOD_FWD
#define ASDA_HOME_METHOD_REVERSE     PARAM_LM_HOME_METHOD_REV
