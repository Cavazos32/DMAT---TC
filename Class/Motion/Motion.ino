/*
 * Motion.ino — MCU Motion (ESP32)
 * Encoders OM (PCNT), feeders CAN (CiA402), LM Guide Modbus (ASDA-B3).
 * Referencia de bus: Encoder.h, ServoFeeder.h, ServoLMGuide.h
 *
 * Datos de ciclo → Preferences (NVS) en el ESP32.
 * Ajustes editables → data/Motion.json vía set_tune (RAM, defaults en Parameters.h).
 *
 * Órdenes de la HMI por Serial (JSON, una línea).
 */

#include <Arduino.h>
#include <Preferences.h>
#include <driver/pcnt.h>
#include <driver/twai.h>
#include <math.h>
#include <string.h>

#include "Parameters.h"
#include "Encoder.h"
#include "ServoFeeder.h"
#include "ServoLMGuide.h"

struct EncoderSideRuntime {
  int32_t zeroRef;
  int32_t pcntOverflow;
  int32_t lastCount;
  int32_t settledCount;
  volatile uint32_t zCount;
  uint32_t lastPollMs;
  uint32_t settleStartMs;
  bool settled;
  bool wasMoving;
  float mmPerSec;
  float mmLive;
  float mmOfficial;
};

struct MotionRuntime {
  EncoderSideRuntime encoder[MOTION_SIDE_COUNT];
  bool feederReady[MOTION_SIDE_COUNT];
  bool lmguideServoOn;
};

static MotionTune gTune;
static MotionData gData;
static MotionRuntime gRt;
static Preferences gPrefs;

static uint32_t gLastEncoderPollMs = 0;
static bool gFeedStageBusy = false;

enum FeedStageError {
  FEED_STAGE_OK = 0,
  FEED_STAGE_ERR_BUSY,
  FEED_STAGE_ERR_NO_TARGET,
  FEED_STAGE_ERR_FEEDER,
  FEED_STAGE_ERR_LASER_R,
  FEED_STAGE_ERR_LASER_L,
  FEED_STAGE_ERR_ENC_R,
  FEED_STAGE_ERR_ENC_L,
  FEED_STAGE_ERR_COMP,
  FEED_STAGE_ERR_SAFETY,
};

struct FeedStageSideResult {
  bool laserOn;
  bool encOk;
  float mmMeasured;
};

struct FeedStageResult {
  FeedStageError error;
  FeedStageSideResult side[MOTION_SIDE_COUNT];
};

// Sin "side" y sin "mode" el ciclo usa ambos lados, valida encoder y el timeout de Parameters.h.
// "mode":"clean" alimenta el tiempo indicado y no se detiene en el láser.
struct FeedStageRequest {
  bool useR;
  bool useL;
  bool checkEncoder;
  bool ignoreLaser;
  uint32_t laserTimeoutMs;
};

static FeedStageResult gLastFeedStage = {};

// ── Utilidades JSON plano (Serial) ──────────────────────────────────────────

static bool jsonFindKey(const char* json, const char* key, const char** valOut) {
  char needle[48];
  snprintf(needle, sizeof(needle), "\"%s\"", key);
  const char* p = strstr(json, needle);
  if (!p) return false;
  p = strchr(p + strlen(needle), ':');
  if (!p) return false;
  p++;
  while (*p == ' ' || *p == '\t') p++;
  *valOut = p;
  return true;
}

static float jsonReadFloat(const char* json, const char* key, float defVal) {
  const char* p = nullptr;
  if (!jsonFindKey(json, key, &p)) return defVal;
  return strtof(p, nullptr);
}

static long jsonReadLong(const char* json, const char* key, long defVal) {
  const char* p = nullptr;
  if (!jsonFindKey(json, key, &p)) return defVal;
  return strtol(p, nullptr, 10);
}

static void runtimeInit(MotionRuntime* rt) {
  memset(rt, 0, sizeof(MotionRuntime));
}

static void dataLoadNvs() {
  motionDataSetDefaults(&gData);
  if (!gPrefs.begin("motion", true)) return;
  gData.targetLengthMm = gPrefs.getFloat("tgt_mm", gData.targetLengthMm);
  gData.toleranceMm = gPrefs.getFloat("tol_mm", gData.toleranceMm);
  gData.measureCompMm = gPrefs.getFloat("comp_mm", gData.measureCompMm);
  String model = gPrefs.getString("model", "");
  model.toCharArray(gData.modeloId, sizeof(gData.modeloId));
  gPrefs.end();
}

static void dataSaveNvs() {
  if (!gPrefs.begin("motion", false)) return;
  gPrefs.putFloat("tgt_mm", gData.targetLengthMm);
  gPrefs.putFloat("tol_mm", gData.toleranceMm);
  gPrefs.putFloat("comp_mm", gData.measureCompMm);
  gPrefs.putString("model", gData.modeloId);
  gPrefs.end();
}

static MotionSide sideFromToken(const char* tok) {
  if (!tok) return MOTION_SIDE_R;
  if (tok[0] == 'L' || tok[0] == 'l') return MOTION_SIDE_L;
  return MOTION_SIDE_R;
}

static uint8_t feederNode(MotionSide side) {
  return (side == MOTION_SIDE_R) ? SERVO_CAN_NODE_R : SERVO_CAN_NODE_L;
}

static int8_t feederCmdSign(MotionSide side) {
  return (side == MOTION_SIDE_R) ? SERVO_CAN_CMD_SIGN_R : SERVO_CAN_CMD_SIGN_L;
}

// ── Encoder (PCNT) ──────────────────────────────────────────────────────────

struct SideHw {
  pcnt_unit_t unit;
  int pinA;
  int pinB;
  int pinZ;
};

static SideHw kSideHw[MOTION_SIDE_COUNT] = {
    {PCNT_UNIT_0, ENC_OM_PIN_A_R, ENC_OM_PIN_B_R, ENC_OM_PIN_Z_R},
    {PCNT_UNIT_1, ENC_OM_PIN_A_L, ENC_OM_PIN_B_L, ENC_OM_PIN_Z_L},
};

static float omRoundMm(float mmAbs) {
  float whole = floorf(mmAbs);
  float frac = mmAbs - whole;
  int tenths = (int)floorf(frac * 10.0f + 0.5f);
  if (tenths <= 2) return whole;
  if (tenths <= 7) return whole + 0.5f;
  return whole + 1.0f;
}

static void IRAM_ATTR onZR() {
  gRt.encoder[MOTION_SIDE_R].zCount++;
}

static void IRAM_ATTR onZL() {
  gRt.encoder[MOTION_SIDE_L].zCount++;
}

static void pcntSetupSide(MotionSide side) {
  const SideHw& hw = kSideHw[side];
  pinMode(hw.pinA, INPUT_PULLUP);
  pinMode(hw.pinB, INPUT_PULLUP);
  pinMode(hw.pinZ, INPUT_PULLUP);

  pcnt_config_t cfg = {};
  cfg.pulse_gpio_num = hw.pinA;
  cfg.ctrl_gpio_num = hw.pinB;
  cfg.lctrl_mode = PCNT_MODE_KEEP;
  cfg.hctrl_mode = PCNT_MODE_REVERSE;
  cfg.pos_mode = PCNT_COUNT_INC;
  cfg.neg_mode = PCNT_COUNT_DEC;
  cfg.counter_h_lim = ENC_OM_PCNT_HIGH_LIM;
  cfg.counter_l_lim = ENC_OM_PCNT_LOW_LIM;
  cfg.unit = hw.unit;
  cfg.channel = PCNT_CHANNEL_0;
  pcnt_unit_config(&cfg);
  pcnt_counter_pause(hw.unit);
  pcnt_counter_clear(hw.unit);
  pcnt_counter_resume(hw.unit);
}

static int16_t pcntReadRaw(MotionSide side) {
  int16_t v = 0;
  pcnt_get_counter_value(kSideHw[side].unit, &v);
  return v;
}

static void pcntClearSide(MotionSide side) {
  pcnt_counter_pause(kSideHw[side].unit);
  pcnt_counter_clear(kSideHw[side].unit);
  pcnt_counter_resume(kSideHw[side].unit);
}

static int32_t encoderReadCountAbs(MotionSide side) {
  int16_t hw = pcntReadRaw(side);
  EncoderSideRuntime* st = &gRt.encoder[side];

  if (hw >= ENC_OM_PCNT_HIGH_LIM - 1) {
    pcntClearSide(side);
    st->pcntOverflow += ENC_OM_PCNT_HIGH_LIM;
    hw = pcntReadRaw(side);
  } else if (hw <= ENC_OM_PCNT_LOW_LIM + 1) {
    pcntClearSide(side);
    st->pcntOverflow += ENC_OM_PCNT_LOW_LIM;
    hw = pcntReadRaw(side);
  }
  return (int32_t)st->pcntOverflow + (int32_t)hw;
}

static void initEncodersHw() {
  runtimeInit(&gRt);
  pcntSetupSide(MOTION_SIDE_R);
  pcntSetupSide(MOTION_SIDE_L);
  attachInterrupt(digitalPinToInterrupt(ENC_OM_PIN_Z_R), onZR, FALLING);
  attachInterrupt(digitalPinToInterrupt(ENC_OM_PIN_Z_L), onZL, FALLING);
}

static void encoderSetZero(MotionSide side) {
  EncoderSideRuntime* st = &gRt.encoder[side];
  int32_t countAbs = encoderReadCountAbs(side);
  st->zeroRef = countAbs;
  st->settledCount = countAbs;
  st->settled = true;
  st->wasMoving = false;
  st->settleStartMs = 0;
}

static void encoderResetHw(MotionSide side) {
  EncoderSideRuntime* st = &gRt.encoder[side];
  pcntClearSide(side);
  st->pcntOverflow = 0;
  st->zeroRef = 0;
  st->lastCount = 0;
  st->settled = false;
  st->wasMoving = false;
  st->settleStartMs = 0;
  st->settledCount = 0;
  st->zCount = 0;
  st->mmLive = 0;
  st->mmOfficial = 0;
  st->mmPerSec = 0;
  st->lastPollMs = 0;
}

static void updateEncoderSide(MotionSide side, uint32_t nowMs) {
  EncoderSideRuntime* st = &gRt.encoder[side];
  int32_t countAbs = encoderReadCountAbs(side);
  int32_t countRel = countAbs - st->zeroRef;
  st->mmLive = ENC_OM_COUNTS_TO_MM(countRel);

  float mmAbs = fabsf(st->mmLive);
  st->mmOfficial = omRoundMm(mmAbs) + PARAM_ENC_OFFSET_MM;

  if (st->lastPollMs != 0) {
    uint32_t dtMs = nowMs - st->lastPollMs;
    if (dtMs > 0) {
      int32_t delta = countAbs - st->lastCount;
      st->mmPerSec = (ENC_OM_COUNTS_TO_MM(delta) * 1000.0f) / (float)dtMs;
    }
  }
  st->lastCount = countAbs;
  st->lastPollMs = nowMs;

  if (fabsf(st->mmPerSec) >= ENC_OM_MOVE_MMS) {
    st->wasMoving = true;
    st->settled = false;
    st->settleStartMs = 0;
  } else if (st->wasMoving) {
    if (st->settleStartMs == 0) {
      st->settleStartMs = nowMs;
    } else if (nowMs - st->settleStartMs >= ENC_OM_SETTLE_MS) {
      st->settled = true;
      st->settledCount = countAbs;
      st->wasMoving = false;
    }
  }
}

static float encoderMmOfficial(MotionSide side) {
  EncoderSideRuntime* st = &gRt.encoder[side];
  if (st->settled) {
    int32_t countRel = st->settledCount - st->zeroRef;
    float mmAbs = fabsf(ENC_OM_COUNTS_TO_MM(countRel));
    return omRoundMm(mmAbs) + PARAM_ENC_OFFSET_MM;
  }
  return st->mmOfficial;
}

static bool encoderWithinTolerance(MotionSide side) {
  if (gData.targetLengthMm <= 0.0f) return false;
  float mm = encoderMmOfficial(side);
  return fabsf(mm - gData.targetLengthMm) <= gData.toleranceMm;
}

// ── CAN feeder ──────────────────────────────────────────────────────────────

static bool canBusInit() {
  twai_general_config_t g_config =
      TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)SERVO_CAN_TX_GPIO, (gpio_num_t)SERVO_CAN_RX_GPIO, TWAI_MODE_NORMAL);
  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
  if (twai_driver_install(&g_config, &t_config, &f_config) != ESP_OK) return false;
  return twai_start() == ESP_OK;
}

static bool canSendRaw(uint32_t cobId, const uint8_t* data, uint8_t len) {
  twai_message_t msg = {};
  msg.identifier = cobId;
  msg.data_length_code = len;
  msg.extd = 0;
  memcpy(msg.data, data, len);
  return twai_transmit(&msg, pdMS_TO_TICKS(80)) == ESP_OK;
}

static bool canSdoDownloadU32(uint8_t node, uint16_t index, uint8_t sub, uint32_t value) {
  uint8_t d[8] = {0x23,
                  (uint8_t)(index & 0xFF),
                  (uint8_t)(index >> 8),
                  sub,
                  (uint8_t)(value & 0xFF),
                  (uint8_t)((value >> 8) & 0xFF),
                  (uint8_t)((value >> 16) & 0xFF),
                  (uint8_t)((value >> 24) & 0xFF)};
  return canSendRaw(SERVO_CAN_COB_TX(node), d, 8);
}

static bool canSdoDownloadU16(uint8_t node, uint16_t index, uint8_t sub, uint16_t value) {
  uint8_t d[8] = {0x2B,
                  (uint8_t)(index & 0xFF),
                  (uint8_t)(index >> 8),
                  sub,
                  (uint8_t)(value & 0xFF),
                  (uint8_t)((value >> 8) & 0xFF),
                  0,
                  0};
  return canSendRaw(SERVO_CAN_COB_TX(node), d, 8);
}

static bool canSdoDownloadU8(uint8_t node, uint16_t index, uint8_t sub, uint8_t value) {
  uint8_t d[8] = {0x2F, (uint8_t)(index & 0xFF), (uint8_t)(index >> 8), sub, value, 0, 0, 0};
  return canSendRaw(SERVO_CAN_COB_TX(node), d, 8);
}

static bool canNmt(uint8_t cmd) {
  uint8_t d[2] = {cmd, 0};
  return canSendRaw(SERVO_CAN_NMT_ID, d, 2);
}

static bool canReadSdoI32(uint8_t node, uint16_t index, uint8_t sub, int32_t* out, uint32_t timeoutMs) {
  uint8_t req[8] = {0x40, (uint8_t)(index & 0xFF), (uint8_t)(index >> 8), sub, 0, 0, 0, 0};
  if (!canSendRaw(SERVO_CAN_COB_TX(node), req, 8)) return false;

  uint32_t t0 = millis();
  while (millis() - t0 < timeoutMs) {
    twai_message_t rx;
    if (twai_receive(&rx, pdMS_TO_TICKS(20)) != ESP_OK) continue;
    if (rx.identifier != SERVO_CAN_COB_RX(node) || rx.data_length_code < 8) continue;
    if (rx.data[0] != 0x43) continue;
    if (rx.data[1] != (index & 0xFF) || rx.data[2] != (index >> 8) || rx.data[3] != sub) continue;
    *out = (int32_t)((uint32_t)rx.data[4] | ((uint32_t)rx.data[5] << 8) | ((uint32_t)rx.data[6] << 16) |
                     ((uint32_t)rx.data[7] << 24));
    return true;
  }
  return false;
}

static void feederWriteProfile(MotionSide side) {
  uint8_t node = feederNode(side);
  canSdoDownloadU32(node, SERVO_OD_PROFILE_VELOCITY, 0, gTune.feeder[side].profileVelocityPps);
  canSdoDownloadU32(node, SERVO_OD_PROFILE_ACCEL, 0, SERVO_CAN_PROFILE_ACCEL_PPS2);
  canSdoDownloadU32(node, SERVO_OD_PROFILE_DECEL, 0, SERVO_CAN_PROFILE_DECEL_PPS2);
}

static bool feederInitNode(MotionSide side) {
  uint8_t node = feederNode(side);
  canSdoDownloadU16(node, SERVO_OD_CONTROL_WORD, 0, 0x0080);
  delay(200);
  canSdoDownloadU16(node, SERVO_OD_CONTROL_WORD, 0, 0x0006);
  delay(200);
  canSdoDownloadU16(node, SERVO_OD_CONTROL_WORD, 0, 0x0007);
  delay(200);
  canSdoDownloadU16(node, SERVO_OD_CONTROL_WORD, 0, 0x000F);
  delay(200);
  canSdoDownloadU8(node, SERVO_OD_MODE_OF_OPERATION, 0, (uint8_t)SERVO_MODE_PP);
  delay(100);
  canSdoDownloadU32(node, SERVO_OD_MAX_PROFILE_VEL, 0, SERVO_CAN_MAX_PPS);
  canSdoDownloadU16(node, SERVO_OD_HALT_OPTION, 0, SERVO_CAN_HALT_OPTION_CODE);
  canSdoDownloadU32(node, SERVO_OD_QUICK_STOP_DECEL, 0, SERVO_CAN_HALT_DECEL_PP);
  feederWriteProfile(side);
  canSdoDownloadU32(node, SERVO_OD_TARGET_POSITION, 0, (uint32_t)gTune.feeder[side].targetPositionCounts);
  canSdoDownloadU16(node, SERVO_OD_CONTROL_WORD, 0, 0x002F);
  delay(100);
  gRt.feederReady[side] = true;
  return true;
}

static void feederInitBus() {
  canNmt(0x80);
  delay(50);
  canNmt(0x01);
  delay(50);
  feederInitNode(MOTION_SIDE_R);
  feederInitNode(MOTION_SIDE_L);
}

static void feederStop(MotionSide side) {
  canSdoDownloadU16(feederNode(side), SERVO_OD_CONTROL_WORD, 0, 0x010F);
}

static bool feederApplySideComp(MotionSide side) {
  float mm = gTune.feeder[side].compensationMm;
  int32_t counts = (int32_t)lroundf(mm * SERVO_FEED_STEPS_PER_MM_S);
  if (counts == 0) return true;
  if (!gRt.feederReady[side]) return false;

  counts *= (int32_t)feederCmdSign(side);
  uint8_t node = feederNode(side);
  canSdoDownloadU8(node, SERVO_OD_MODE_OF_OPERATION, 0, (uint8_t)SERVO_MODE_PP);
  int32_t pos = 0;
  if (!canReadSdoI32(node, SERVO_OD_POSITION_ACTUAL, 0, &pos, 400)) return false;

  int32_t target = pos + counts;
  feederWriteProfile(side);
  canSdoDownloadU16(node, SERVO_OD_CONTROL_WORD, 0, 0x010F);
  canSdoDownloadU32(node, SERVO_OD_TARGET_POSITION, 0, (uint32_t)target);
  canSdoDownloadU16(node, SERVO_OD_CONTROL_WORD, 0, 0x012F);
  canSdoDownloadU16(node, SERVO_OD_CONTROL_WORD, 0, 0x013F);
  canSdoDownloadU16(node, SERVO_OD_CONTROL_WORD, 0, 0x003F);
  return true;
}

static void feederJogStep(MotionSide side, int dirSign) {
  if (!gRt.feederReady[side]) return;
  uint8_t node = feederNode(side);
  int32_t pos = 0;
  if (!canReadSdoI32(node, SERVO_OD_POSITION_ACTUAL, 0, &pos, 400)) return;

  int32_t delta = SERVO_CAN_JOG_STEP_COUNTS * (int32_t)dirSign * (int32_t)feederCmdSign(side);
  int32_t target = pos + delta;

  feederWriteProfile(side);
  canSdoDownloadU16(node, SERVO_OD_CONTROL_WORD, 0, 0x010F);
  canSdoDownloadU32(node, SERVO_OD_TARGET_POSITION, 0, (uint32_t)target);
  canSdoDownloadU16(node, SERVO_OD_CONTROL_WORD, 0, 0x012F);
  canSdoDownloadU16(node, SERVO_OD_CONTROL_WORD, 0, 0x013F);
  canSdoDownloadU16(node, SERVO_OD_CONTROL_WORD, 0, 0x003F);
}

static void feederRunPv(MotionSide side, int32_t velocitySigned) {
  uint8_t node = feederNode(side);
  canSdoDownloadU8(node, SERVO_OD_MODE_OF_OPERATION, 0, (uint8_t)SERVO_MODE_PV);
  feederWriteProfile(side);
  canSdoDownloadU32(node, SERVO_OD_TARGET_VELOCITY, 0, (uint32_t)velocitySigned);
  canSdoDownloadU16(node, SERVO_OD_CONTROL_WORD, 0, 0x000F);
}

static void feederStopAll() {
  feederStop(MOTION_SIDE_R);
  feederStop(MOTION_SIDE_L);
  canSdoDownloadU32(feederNode(MOTION_SIDE_R), SERVO_OD_TARGET_VELOCITY, 0, 0);
  canSdoDownloadU32(feederNode(MOTION_SIDE_L), SERVO_OD_TARGET_VELOCITY, 0, 0);
}

static int32_t feederFeedVelocity(MotionSide side) {
  int32_t v = (int32_t)gTune.feeder[side].profileVelocityPps;
  if (v < 0) v = -v;
  return v * (int32_t)feederCmdSign(side);
}

// ── FeedStage (Bloque 1 — alimentación y medición) ──────────────────────────

static void initLasersHw() {
  pinMode(LASER_PIN_R, INPUT_PULLUP);
  pinMode(LASER_PIN_L, INPUT);
  pinMode(SAFETY_PIN, INPUT_PULLUP);
}

static bool safetyIsActive() {
  return digitalRead(SAFETY_PIN) == HIGH;
}

static bool laserIsOn(MotionSide side) {
  int pin = (side == MOTION_SIDE_R) ? LASER_PIN_R : LASER_PIN_L;
  int level = digitalRead(pin);
  if (LASER_ACTIVE_HIGH) return level == HIGH;
  return level == LOW;
}

static void motionPollEncoders(uint32_t nowMs) {
  updateEncoderSide(MOTION_SIDE_R, nowMs);
  updateEncoderSide(MOTION_SIDE_L, nowMs);
}

static void encoderArmSettleAfterStop(MotionSide side) {
  EncoderSideRuntime* st = &gRt.encoder[side];
  st->settled = false;
  st->wasMoving = true;
  st->settleStartMs = 0;
}

static const char* feedStageErrorName(FeedStageError err) {
  switch (err) {
    case FEED_STAGE_OK:
      return "ok";
    case FEED_STAGE_ERR_BUSY:
      return "busy";
    case FEED_STAGE_ERR_NO_TARGET:
      return "no_target";
    case FEED_STAGE_ERR_FEEDER:
      return "feeder_not_ready";
    case FEED_STAGE_ERR_LASER_R:
      return "laser_timeout_r";
    case FEED_STAGE_ERR_LASER_L:
      return "laser_timeout_l";
    case FEED_STAGE_ERR_ENC_R:
      return "encoder_tolerance_r";
    case FEED_STAGE_ERR_ENC_L:
      return "encoder_tolerance_l";
    case FEED_STAGE_ERR_COMP:
      return "comp_move";
    case FEED_STAGE_ERR_SAFETY:
      return "safety";
    default:
      return "unknown";
  }
}

static FeedStageRequest feedStageDefaultRequest() {
  FeedStageRequest req;
  req.useR = true;
  req.useL = true;
  req.checkEncoder = true;
  req.ignoreLaser = false;
  req.laserTimeoutMs = PARAM_FEED_STAGE_LASER_TIMEOUT_MS;
  return req;
}

static FeedStageResult FeedStage(FeedStageRequest req) {
  FeedStageResult out = {};
  if (gFeedStageBusy) {
    out.error = FEED_STAGE_ERR_BUSY;
    return out;
  }
  if (req.checkEncoder && gData.targetLengthMm <= 0.0f) {
    out.error = FEED_STAGE_ERR_NO_TARGET;
    return out;
  }
  if ((req.useR && !gRt.feederReady[MOTION_SIDE_R]) ||
      (req.useL && !gRt.feederReady[MOTION_SIDE_L])) {
    out.error = FEED_STAGE_ERR_FEEDER;
    return out;
  }
  if (safetyIsActive()) {
    out.error = FEED_STAGE_ERR_SAFETY;
    return out;
  }

  gFeedStageBusy = true;

  bool sideFeedStopped[MOTION_SIDE_COUNT] = {!req.useR, !req.useL};
  bool sideValidated[MOTION_SIDE_COUNT] = {!req.useR, !req.useL};
  uint32_t tStart = millis();

  if (req.useR) feederRunPv(MOTION_SIDE_R, feederFeedVelocity(MOTION_SIDE_R));
  if (req.useL) feederRunPv(MOTION_SIDE_L, feederFeedVelocity(MOTION_SIDE_L));

  FeedStageError fail = FEED_STAGE_OK;

  while (true) {
    pollSerial();
    uint32_t nowMs = millis();
    motionPollEncoders(nowMs);

    if (safetyIsActive()) {
      fail = FEED_STAGE_ERR_SAFETY;
      break;
    }

    if (req.ignoreLaser) {
      if (nowMs - tStart >= req.laserTimeoutMs) {
        out.error = FEED_STAGE_OK;
        break;
      }
    } else {
    for (int i = 0; i < MOTION_SIDE_COUNT; i++) {
      if (sideValidated[i]) continue;
      MotionSide side = (MotionSide)i;

      if (!sideFeedStopped[i] && laserIsOn(side)) {
        feederStop(side);
        canSdoDownloadU32(feederNode(side), SERVO_OD_TARGET_VELOCITY, 0, 0);
        sideFeedStopped[i] = true;
        out.side[i].laserOn = true;
        if (req.checkEncoder) encoderArmSettleAfterStop(side);
      }

      if (sideFeedStopped[i] && !sideValidated[i]) {
        if (!req.checkEncoder) {
          out.side[i].mmMeasured = encoderMmOfficial(side);
          out.side[i].encOk = true;
          sideValidated[i] = true;
        } else if (gRt.encoder[side].settled) {
          out.side[i].mmMeasured = encoderMmOfficial(side);
          out.side[i].encOk = encoderWithinTolerance(side);
          sideValidated[i] = true;
          if (!out.side[i].encOk) {
            fail = (side == MOTION_SIDE_R) ? FEED_STAGE_ERR_ENC_R : FEED_STAGE_ERR_ENC_L;
          }
        }
      }

      if (!sideFeedStopped[i] && (nowMs - tStart >= req.laserTimeoutMs)) {
        fail = (side == MOTION_SIDE_R) ? FEED_STAGE_ERR_LASER_R : FEED_STAGE_ERR_LASER_L;
        break;
      }
    }
    }

    if (fail != FEED_STAGE_OK) break;

    if (sideValidated[MOTION_SIDE_R] && sideValidated[MOTION_SIDE_L]) {
      out.error = FEED_STAGE_OK;
      break;
    }
  }

  feederStopAll();
  if (fail != FEED_STAGE_OK) {
    out.error = fail;
  } else if (req.checkEncoder) {
    // mmMeasured ya quedó congelado con el encoder quieto. La compensación
    // mueve el material después; no se vuelve a leer ni a validar.
    bool compOk = true;
    if (req.useR) compOk = feederApplySideComp(MOTION_SIDE_R);
    if (compOk && req.useL) compOk = feederApplySideComp(MOTION_SIDE_L);
    if (!compOk) out.error = FEED_STAGE_ERR_COMP;
  }

  gLastFeedStage = out;
  gFeedStageBusy = false;
  return out;
}

static void printFeedStageResultJson(const FeedStageResult& r) {
  Serial.print(F("{\"op\":\"feed_stage\",\"ok\":"));
  Serial.print(r.error == FEED_STAGE_OK ? "true" : "false");
  Serial.print(F(",\"error\":\""));
  Serial.print(feedStageErrorName(r.error));
  Serial.print(F("\",\"R\":{\"laser\":"));
  Serial.print(r.side[MOTION_SIDE_R].laserOn ? "true" : "false");
  Serial.print(F(",\"enc_ok\":"));
  Serial.print(r.side[MOTION_SIDE_R].encOk ? "true" : "false");
  Serial.print(F(",\"mm\":"));
  Serial.print(r.side[MOTION_SIDE_R].mmMeasured, 2);
  Serial.print(F(",\"comp_mm\":"));
  Serial.print(gTune.feeder[MOTION_SIDE_R].compensationMm, 2);
  Serial.print(F("},\"L\":{\"laser\":"));
  Serial.print(r.side[MOTION_SIDE_L].laserOn ? "true" : "false");
  Serial.print(F(",\"enc_ok\":"));
  Serial.print(r.side[MOTION_SIDE_L].encOk ? "true" : "false");
  Serial.print(F(",\"mm\":"));
  Serial.print(r.side[MOTION_SIDE_L].mmMeasured, 2);
  Serial.print(F(",\"comp_mm\":"));
  Serial.print(gTune.feeder[MOTION_SIDE_L].compensationMm, 2);
  Serial.println(F("}}"));
}

// ── Modbus LM Guide ─────────────────────────────────────────────────────────

static HardwareSerial ModbusSerial(2);

static uint16_t modbusCrc16(const uint8_t* data, size_t len) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (int b = 0; b < 8; b++) {
      if (crc & 1)
        crc = (crc >> 1) ^ 0xA001;
      else
        crc >>= 1;
    }
  }
  return crc;
}

static bool modbusParseRxOk(const uint8_t* tx, size_t txLen, const uint8_t* rx, size_t rxLen) {
  if (rxLen < 4) return false;
  uint16_t crc = modbusCrc16(rx, rxLen - 2);
  if (rx[rxLen - 2] != (uint8_t)(crc & 0xFF) || rx[rxLen - 1] != (uint8_t)(crc >> 8)) return false;
  if (rx[0] != MODBUS_ASDA_SLAVE_ID) return false;
  if (tx[1] == 0x06 && rxLen >= 8) return memcmp(tx, rx, 6) == 0;
  if (tx[1] == 0x10 && rxLen >= 8) return rx[1] == 0x10;
  if (tx[1] == 0x03 && rxLen >= 5) return rx[1] == 0x03;
  return false;
}

static bool modbusTxRx(const uint8_t* tx, size_t txLen, uint8_t* rx, size_t* rxLen, size_t rxCap) {
  while (ModbusSerial.available()) ModbusSerial.read();
  ModbusSerial.write(tx, txLen);
  ModbusSerial.flush();

  uint32_t t0 = millis();
  size_t n = 0;
  while (millis() - t0 < 350) {
    while (ModbusSerial.available() && n < rxCap) {
      rx[n++] = (uint8_t)ModbusSerial.read();
    }
    if (n >= 5) {
      if (tx[1] == 0x03 && n >= (size_t)(5 + rx[2])) break;
      if ((tx[1] == 0x06 || tx[1] == 0x10) && n >= 8) break;
    }
    delay(2);
  }
  *rxLen = n;
  return modbusParseRxOk(tx, txLen, rx, n);
}

static bool modbusWriteReg16(uint16_t addr, uint16_t value) {
  uint8_t tx[8];
  tx[0] = MODBUS_ASDA_SLAVE_ID;
  tx[1] = 0x06;
  tx[2] = (uint8_t)(addr >> 8);
  tx[3] = (uint8_t)(addr & 0xFF);
  tx[4] = (uint8_t)(value >> 8);
  tx[5] = (uint8_t)(value & 0xFF);
  uint16_t crc = modbusCrc16(tx, 6);
  tx[6] = (uint8_t)(crc & 0xFF);
  tx[7] = (uint8_t)(crc >> 8);
  uint8_t rx[16];
  size_t rxLen = 0;
  return modbusTxRx(tx, 8, rx, &rxLen, sizeof(rx));
}

static bool modbusWriteReg32(uint16_t addr, int32_t value) {
  uint8_t tx[13];
  tx[0] = MODBUS_ASDA_SLAVE_ID;
  tx[1] = 0x10;
  tx[2] = (uint8_t)(addr >> 8);
  tx[3] = (uint8_t)(addr & 0xFF);
  tx[4] = 0x00;
  tx[5] = 0x02;
  tx[6] = 0x04;
  tx[7] = (uint8_t)((value >> 8) & 0xFF);
  tx[8] = (uint8_t)(value & 0xFF);
  tx[9] = (uint8_t)((value >> 24) & 0xFF);
  tx[10] = (uint8_t)((value >> 16) & 0xFF);
  uint16_t crc = modbusCrc16(tx, 11);
  tx[11] = (uint8_t)(crc & 0xFF);
  tx[12] = (uint8_t)(crc >> 8);
  uint8_t rx[16];
  size_t rxLen = 0;
  return modbusTxRx(tx, 13, rx, &rxLen, sizeof(rx));
}

static bool modbusReadReg32(uint16_t addr, int32_t* out) {
  uint8_t tx[8];
  tx[0] = MODBUS_ASDA_SLAVE_ID;
  tx[1] = 0x03;
  tx[2] = (uint8_t)(addr >> 8);
  tx[3] = (uint8_t)(addr & 0xFF);
  tx[4] = 0x00;
  tx[5] = 0x02;
  uint16_t crc = modbusCrc16(tx, 6);
  tx[6] = (uint8_t)(crc & 0xFF);
  tx[7] = (uint8_t)(crc >> 8);
  uint8_t rx[16];
  size_t rxLen = 0;
  if (!modbusTxRx(tx, 8, rx, &rxLen, sizeof(rx)) || rxLen < 9) return false;
  uint16_t lo = ((uint16_t)rx[3] << 8) | rx[4];
  uint16_t hi = ((uint16_t)rx[5] << 8) | rx[6];
  *out = (int32_t)((uint32_t)lo | ((uint32_t)hi << 16));
  return true;
}

static void modbusBeginHw() {
  ModbusSerial.begin(MODBUS_ASDA_BAUD, SERIAL_8N1, 16, 17);
}

static bool modbusReadReg16(uint16_t addr, uint16_t* out) {
  uint8_t tx[8];
  tx[0] = MODBUS_ASDA_SLAVE_ID;
  tx[1] = 0x03;
  tx[2] = (uint8_t)(addr >> 8);
  tx[3] = (uint8_t)(addr & 0xFF);
  tx[4] = 0x00;
  tx[5] = 0x01;
  uint16_t crc = modbusCrc16(tx, 6);
  tx[6] = (uint8_t)(crc & 0xFF);
  tx[7] = (uint8_t)(crc >> 8);
  uint8_t rx[16];
  size_t rxLen = 0;
  if (!modbusTxRx(tx, 8, rx, &rxLen, sizeof(rx)) || rxLen < 7) return false;
  *out = ((uint16_t)rx[3] << 8) | rx[4];
  return true;
}

static bool lmguideServoOn() {
  if (!modbusWriteReg16(ASDA_REG_P2_030, 1)) return false;
  delay(ASDA_SERVO_ON_SETTLE_MS);
  gRt.lmguideServoOn = true;
  return true;
}

static bool lmguideServoOff() {
  if (!modbusWriteReg16(ASDA_REG_P2_030, 0xFFFF)) return false;
  if (!modbusWriteReg16(ASDA_REG_P2_030, 0)) return false;
  gRt.lmguideServoOn = false;
  return true;
}

static bool lmguideMovePr1(int32_t targetPuu) {
  int32_t speed01 = (int32_t)(gTune.lmguide.prSpeedRpm * 10.0f);
  if (!modbusWriteReg32(ASDA_REG_P5_060, speed01)) return false;
  if (!modbusWriteReg32(ASDA_REG_P6_002, 0x00000002)) return false;
  if (!modbusWriteReg32(ASDA_REG_P6_003, targetPuu)) return false;
  return modbusWriteReg16(ASDA_REG_P5_007, 1);
}

static bool lmguideStop() {
  return modbusWriteReg16(ASDA_REG_P5_007, 1000);
}

static bool lmguideJog(int dir) {
  int32_t pos = 0;
  if (!modbusReadReg32(ASDA_REG_P5_016, &pos)) return false;
  int32_t target = pos + dir * ASDA_JOG_DELTA_PUU;
  return lmguideMovePr1(target);
}

static bool lmguideJogDelta(int32_t deltaPuu) {
  int32_t pos = 0;
  if (!modbusReadReg32(ASDA_REG_P5_016, &pos)) return false;
  return lmguideMovePr1(pos + deltaPuu);
}

// Homing mecánico ASDA — secuencia ServoLMGuide.h [5], método forward.
static bool lmguideHome() {
  if (!lmguideServoOn()) return false;
  if (!modbusWriteReg16(ASDA_REG_P1_087, ASDA_HOME_TORQUE_PCT_DEFAULT)) return false;
  if (!modbusWriteReg16(ASDA_REG_P1_088, ASDA_HOME_TIME_MS_DEFAULT)) return false;
  if (!modbusWriteReg16(ASDA_REG_P5_004, ASDA_HOME_METHOD_FORWARD)) return false;
  int32_t speed01 = (int32_t)(ASDA_HOME_SPEED_RPM_DEFAULT * 10.0f);
  if (!modbusWriteReg32(ASDA_REG_P5_005, speed01)) return false;
  if (!modbusWriteReg32(ASDA_REG_P5_006, speed01)) return false;
  if (!modbusWriteReg32(ASDA_REG_P6_000, 0)) return false;
  if (!modbusWriteReg32(ASDA_REG_P6_001, 0)) return false;
  if (!modbusWriteReg16(ASDA_REG_P5_007, 0)) return false;

  uint32_t t0 = millis();
  while (millis() - t0 < PARAM_LM_HOME_DONE_TIMEOUT_MS) {
    uint16_t dwo = 0;
    if (modbusReadReg16(ASDA_REG_P0_046, &dwo)) {
      if ((dwo & ASDA_DO_BIT_HOME) && (dwo & ASDA_DO_BIT_ZSPD)) return true;
    }
    delay(50);
  }
  return false;
}

static bool lmguideMoveModel() {
  if (gData.targetLengthMm <= 0.0f) return false;
  float mm = gData.targetLengthMm + gData.measureCompMm;
  int32_t puu = (int32_t)lroundf(-mm * ASDA_FACTORY_STEPS_PER_MM);
  return lmguideMovePr1(puu);
}

static bool lmguideMoveNamed(const char* name) {
  if (!name) return false;
  if (strncmp(name, "model", 5) == 0) return lmguideMoveModel();
  if (strncmp(name, "home", 4) == 0) return lmguideMovePr1(0);
  return lmguideMovePr1(gTune.lmguide.posPuu);
}

// ── Serial / telemetría ─────────────────────────────────────────────────────

static void printStatusJson() {
  Serial.print(F("{\"op\":\"status\""));
  Serial.print(F(",\"data\":{\"target_mm\":"));
  Serial.print(gData.targetLengthMm, 2);
  Serial.print(F(",\"tol_mm\":"));
  Serial.print(gData.toleranceMm, 2);
  Serial.print(F(",\"comp_mm\":"));
  Serial.print(gData.measureCompMm, 2);
  Serial.print(F(",\"modelo\":\""));
  Serial.print(gData.modeloId);
  Serial.print(F("\"}"));

  const char* sideTag[] = {"R", "L"};
  for (int i = 0; i < MOTION_SIDE_COUNT; i++) {
    Serial.print(F(",\"enc_"));
    Serial.print(sideTag[i]);
    Serial.print(F("\":{\"mm_live\":"));
    Serial.print(gRt.encoder[i].mmLive, 2);
    Serial.print(F(",\"mm_off\":"));
    Serial.print(encoderMmOfficial((MotionSide)i), 2);
    Serial.print(F(",\"settled\":"));
    Serial.print(gRt.encoder[i].settled ? "true" : "false");
    Serial.print(F(",\"ok_tol\":"));
    Serial.print(encoderWithinTolerance((MotionSide)i) ? "true" : "false");
    Serial.print(F(",\"laser\":"));
    Serial.print(laserIsOn((MotionSide)i) ? "true" : "false");
    Serial.print(F("}"));
  }

  Serial.print(F(",\"feeder_r_ready\":"));
  Serial.print(gRt.feederReady[MOTION_SIDE_R] ? "true" : "false");
  Serial.print(F(",\"feeder_l_ready\":"));
  Serial.print(gRt.feederReady[MOTION_SIDE_L] ? "true" : "false");
  Serial.print(F(",\"lm_servo_on\":"));
  Serial.print(gRt.lmguideServoOn ? "true" : "false");
  Serial.print(F(",\"safety\":"));
  Serial.print(safetyIsActive() ? "true" : "false");
  Serial.println(F("}"));
}

static void applyTuneFromJson(const char* json) {
  gTune.feeder[MOTION_SIDE_R].profileVelocityPps =
      (uint32_t)jsonReadLong(json, "feed_r_vel_pps", (long)gTune.feeder[MOTION_SIDE_R].profileVelocityPps);
  gTune.feeder[MOTION_SIDE_R].targetPositionCounts =
      (int32_t)jsonReadLong(json, "feed_r_target_counts", gTune.feeder[MOTION_SIDE_R].targetPositionCounts);

  gTune.feeder[MOTION_SIDE_L].profileVelocityPps =
      (uint32_t)jsonReadLong(json, "feed_l_vel_pps", (long)gTune.feeder[MOTION_SIDE_L].profileVelocityPps);
  gTune.feeder[MOTION_SIDE_L].targetPositionCounts =
      (int32_t)jsonReadLong(json, "feed_l_target_counts", gTune.feeder[MOTION_SIDE_L].targetPositionCounts);

  gTune.feeder[MOTION_SIDE_R].compensationMm =
      jsonReadFloat(json, "feed_r_comp_mm", gTune.feeder[MOTION_SIDE_R].compensationMm);
  gTune.feeder[MOTION_SIDE_L].compensationMm =
      jsonReadFloat(json, "feed_l_comp_mm", gTune.feeder[MOTION_SIDE_L].compensationMm);

  gTune.lmguide.prSpeedRpm = jsonReadFloat(json, "lm_pr_speed_rpm", gTune.lmguide.prSpeedRpm);
  gTune.lmguide.posPuu = (int32_t)jsonReadLong(json, "lm_pos_puu", gTune.lmguide.posPuu);
}

static void applyDataFromJson(const char* json) {
  gData.targetLengthMm = jsonReadFloat(json, "target_length_mm", gData.targetLengthMm);
  gData.toleranceMm = jsonReadFloat(json, "tolerance_mm", gData.toleranceMm);
  gData.measureCompMm = jsonReadFloat(json, "measure_comp_mm", gData.measureCompMm);

  const char* p = nullptr;
  if (jsonFindKey(json, "modelo_id", &p)) {
    if (*p == '"') {
      p++;
      size_t i = 0;
      while (*p && *p != '"' && i + 1 < sizeof(gData.modeloId)) {
        gData.modeloId[i++] = *p++;
      }
      gData.modeloId[i] = '\0';
    }
  }
  dataSaveNvs();
}

static void handleSerialLine(char* line) {
  while (*line == ' ' || *line == '\t') line++;
  if (*line == '\0') return;

  if (strstr(line, "\"op\":\"ping\"") || strstr(line, "\"op\": \"ping\"")) {
    Serial.println(F("{\"op\":\"pong\"}"));
    return;
  }
  if (strstr(line, "\"op\":\"status\"")) {
    printStatusJson();
    return;
  }
  if (strstr(line, "\"op\":\"set_tune\"")) {
    applyTuneFromJson(line);
    Serial.println(F("{\"op\":\"set_tune\",\"ok\":true}"));
    return;
  }
  if (strstr(line, "\"op\":\"set_data\"")) {
    applyDataFromJson(line);
    Serial.println(F("{\"op\":\"set_data\",\"ok\":true}"));
    return;
  }
  if (strstr(line, "\"op\":\"encoder_set0\"")) {
    const char* sideTok = strstr(line, "\"side\"");
    MotionSide side = MOTION_SIDE_R;
    if (sideTok) {
      const char* q = strchr(sideTok, ':');
      if (q) {
        q++;
        while (*q == ' ' || *q == '"') q++;
        side = sideFromToken(q);
      }
    }
    if (strstr(line, "\"side\":\"both\"")) {
      encoderSetZero(MOTION_SIDE_R);
      encoderSetZero(MOTION_SIDE_L);
    } else {
      encoderSetZero(side);
    }
    Serial.println(F("{\"op\":\"encoder_set0\",\"ok\":true}"));
    return;
  }
  if (strstr(line, "\"op\":\"encoder_reset\"")) {
    if (strstr(line, "\"side\":\"L\"")) {
      encoderResetHw(MOTION_SIDE_L);
    } else if (strstr(line, "\"side\":\"R\"")) {
      encoderResetHw(MOTION_SIDE_R);
    } else {
      encoderResetHw(MOTION_SIDE_R);
      encoderResetHw(MOTION_SIDE_L);
    }
    Serial.println(F("{\"op\":\"encoder_reset\",\"ok\":true}"));
    return;
  }
  if (strstr(line, "\"op\":\"feeder_init\"")) {
    if (strstr(line, "\"side\":\"both\"") || !strstr(line, "\"side\"")) {
      feederInitBus();
    } else if (strstr(line, "\"side\":\"L\"")) {
      feederInitNode(MOTION_SIDE_L);
    } else {
      feederInitNode(MOTION_SIDE_R);
    }
    Serial.println(F("{\"op\":\"feeder_init\",\"ok\":true}"));
    return;
  }
  if (strstr(line, "\"op\":\"feeder_jog\"")) {
    if (safetyIsActive()) {
      Serial.println(F("{\"op\":\"feeder_jog\",\"ok\":false,\"error\":\"safety\"}"));
      return;
    }
    MotionSide side = MOTION_SIDE_R;
    if (strstr(line, "\"side\":\"L\"")) side = MOTION_SIDE_L;
    long dir = jsonReadLong(line, "dir", 1);
    feederJogStep(side, dir >= 0 ? 1 : -1);
    Serial.println(F("{\"op\":\"feeder_jog\",\"ok\":true}"));
    return;
  }
  if (strstr(line, "\"op\":\"feeder_stop\"")) {
    feederStopAll();
    Serial.println(F("{\"op\":\"feeder_stop\",\"ok\":true}"));
    return;
  }
  if (strstr(line, "\"op\":\"feed_stage\"")) {
    FeedStageRequest req = feedStageDefaultRequest();
    if (strstr(line, "\"side\":\"L\"")) req.useR = false;
    else if (strstr(line, "\"side\":\"R\"")) req.useL = false;
    if (strstr(line, "\"mode\":\"clean\"")) {
      req.checkEncoder = false;
      req.ignoreLaser = true;
      req.laserTimeoutMs = 3000u;
    } else if (strstr(line, "\"mode\":\"laser\"")) {
      req.checkEncoder = false;
      req.laserTimeoutMs = 10000u;
    }
    long timeoutMs = jsonReadLong(line, "timeout_ms", (long)req.laserTimeoutMs);
    if (timeoutMs >= 500 && timeoutMs <= 60000) req.laserTimeoutMs = (uint32_t)timeoutMs;
    FeedStageResult r = FeedStage(req);
    printFeedStageResultJson(r);
    return;
  }
  if (strstr(line, "\"op\":\"lmguide_servo_on\"")) {
    bool ok = lmguideServoOn();
    Serial.print(F("{\"op\":\"lmguide_servo_on\",\"ok\":"));
    Serial.print(ok ? "true" : "false");
    Serial.println(F("}"));
    return;
  }
  if (strstr(line, "\"op\":\"lmguide_servo_off\"")) {
    bool ok = lmguideServoOff();
    Serial.print(F("{\"op\":\"lmguide_servo_off\",\"ok\":"));
    Serial.print(ok ? "true" : "false");
    Serial.println(F("}"));
    return;
  }
  if (strstr(line, "\"op\":\"lmguide_home\"")) {
    if (safetyIsActive()) {
      Serial.println(F("{\"op\":\"lmguide_home\",\"ok\":false,\"error\":\"safety\"}"));
      return;
    }
    bool ok = lmguideHome();
    Serial.print(F("{\"op\":\"lmguide_home\",\"ok\":"));
    Serial.print(ok ? "true" : "false");
    Serial.println(F("}"));
    return;
  }
  if (strstr(line, "\"op\":\"lmguide_move\"")) {
    if (safetyIsActive()) {
      Serial.println(F("{\"op\":\"lmguide_move\",\"ok\":false,\"error\":\"safety\"}"));
      return;
    }
    const char* target = "home";
    const char* key = strstr(line, "\"target\"");
    if (key) {
      const char* q = strchr(key, ':');
      if (q) {
        q++;
        while (*q == ' ' || *q == '"') q++;
        target = q;
      }
    }
    long customPuu = jsonReadLong(line, "puu", LONG_MIN);
    bool ok = false;
    if (customPuu != LONG_MIN)
      ok = lmguideMovePr1((int32_t)customPuu);
    else
      ok = lmguideMoveNamed(target);
    Serial.print(F("{\"op\":\"lmguide_move\",\"ok\":"));
    Serial.print(ok ? "true" : "false");
    Serial.println(F("}"));
    return;
  }
  if (strstr(line, "\"op\":\"lmguide_jog\"")) {
    if (safetyIsActive()) {
      Serial.println(F("{\"op\":\"lmguide_jog\",\"ok\":false,\"error\":\"safety\"}"));
      return;
    }
    bool ok = false;
    if (strstr(line, "\"delta_puu\"")) {
      ok = lmguideJogDelta((int32_t)jsonReadLong(line, "delta_puu", 0));
    } else {
      long dir = jsonReadLong(line, "dir", 1);
      ok = lmguideJog(dir >= 0 ? 1 : -1);
    }
    Serial.print(F("{\"op\":\"lmguide_jog\",\"ok\":"));
    Serial.print(ok ? "true" : "false");
    Serial.println(F("}"));
    return;
  }
  if (strstr(line, "\"op\":\"lmguide_stop\"")) {
    bool ok = lmguideStop();
    Serial.print(F("{\"op\":\"lmguide_stop\",\"ok\":"));
    Serial.print(ok ? "true" : "false");
    Serial.println(F("}"));
    return;
  }

  Serial.println(F("{\"op\":\"error\",\"msg\":\"unknown_command\"}"));
}

static char gSerialLine[512];
static size_t gSerialLineLen = 0;

static void pollSerial() {
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\r') continue;
    if (c == '\n') {
      gSerialLine[gSerialLineLen] = '\0';
      handleSerialLine(gSerialLine);
      gSerialLineLen = 0;
      continue;
    }
    if (gSerialLineLen + 1 < sizeof(gSerialLine)) {
      gSerialLine[gSerialLineLen++] = c;
    }
  }
}

// ── Arduino ─────────────────────────────────────────────────────────────────

void setup() {
  Serial.begin(PARAM_SERIAL_BAUD);
  delay(200);

  motionTuneSetDefaults(&gTune);
  dataLoadNvs();
  initEncodersHw();
  initLasersHw();
  modbusBeginHw();

  if (canBusInit()) {
    feederInitBus();
  }

  Serial.println(F("{\"op\":\"boot\",\"mcu\":\"motion\"}"));
  printStatusJson();
}

void loop() {
  pollSerial();

  uint32_t now = millis();
  if (now - gLastEncoderPollMs >= PARAM_ENC_POLL_MS) {
    gLastEncoderPollMs = now;
    updateEncoderSide(MOTION_SIDE_R, now);
    updateEncoderSide(MOTION_SIDE_L, now);
  }
}
