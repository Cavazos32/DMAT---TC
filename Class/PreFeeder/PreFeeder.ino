// Lado L o R según config.h. El ciclo es local.
// Al maestro: POST del estado cuando cambia, GET de los parámetros.

#include "config.h"
#include "ErrorCode.h"
#include <HTTPClient.h>
#include <StepperRMT.h>
#include <WiFi.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

struct SensorSnapshot {
  bool holguraOn = false;
  bool cylinderOpen = false;
  bool limitActive = false;
};

struct FeedParams {
  float rpm = FEED_RPM;
  float pullerRpm = PULLER_RPM;
  uint16_t settleMs = HOLGURA_SETTLE_MS;
  float onSec = HOLGURA_ON_FEED_SEC;
  float offFaultSec = HOLGURA_OFF_FAULT_SEC;
  float stallFaultSec = HOLGURA_STALL_FAULT_SEC;
};

struct StatusPod {
  uint8_t state = PF_RUN_IDLE;
  uint8_t fault = FAULT_NONE;
  uint8_t cylinder = 0;
  uint8_t holgura = 0;
  uint8_t limit = 0;
};

static FeedParams gFeed;
static SemaphoreHandle_t paramMux = nullptr;
static bool gFeedLock = true;
static bool gOmitHolgura = false;
static volatile bool gCfgPollUrgent = false;
static bool gForceFeed = false;  // HMI / maestro (set_force_feed)
static bool gJogBtn = false;     // Botón físico JOG - L / JOG - R

static FeedParams paramsNow() {
  FeedParams p = gFeed;
  if (paramMux && xSemaphoreTake(paramMux, portMAX_DELAY)) {
    p = gFeed;
    xSemaphoreGive(paramMux);
  }
  return p;
}

static void clampFeed(FeedParams& p) {
  if (p.rpm < RPM_MIN) p.rpm = RPM_MIN;
  if (p.rpm > RPM_MAX) p.rpm = RPM_MAX;
  if (p.pullerRpm < RPM_MIN) p.pullerRpm = RPM_MIN;
  if (p.pullerRpm > RPM_MAX) p.pullerRpm = RPM_MAX;
  if (p.settleMs < HOLGURA_SETTLE_MS_MIN) p.settleMs = HOLGURA_SETTLE_MS_MIN;
  if (p.settleMs > HOLGURA_SETTLE_MS_MAX) p.settleMs = HOLGURA_SETTLE_MS_MAX;
  if (p.onSec < 0.1f) p.onSec = 0.1f;
  if (p.onSec > 120.0f) p.onSec = 120.0f;
  if (p.offFaultSec < 0.5f) p.offFaultSec = 0.5f;
  if (p.offFaultSec > 300.0f) p.offFaultSec = 300.0f;
  if (p.stallFaultSec < 0.5f) p.stallFaultSec = 0.5f;
  if (p.stallFaultSec > 60.0f) p.stallFaultSec = 60.0f;
}

// --- Steppers: mismo patrón que PreFeeder TCM (feeder + motor2), sin limitar RPM_MAX
//     y con parada suave setSpeed(0) en ciclo (no Brake).

static StepperRMT* feeder = nullptr;
static StepperRMT* puller = nullptr;
static float commandedFeederRpm = 0.0f;
static float commandedPullerRpm = 0.0f;
static uint8_t gPullerDir = 1;  // 1 = DIR MOSFET activo
static bool motorsReady = false;

static void PullerDir(uint8_t on) {
  static bool ready = false;
  if (!ready) {
    pinMode(PIN_PULLER_DIR, OUTPUT);
    ready = true;
  }
  gPullerDir = on ? 1 : 0;
  digitalWrite(PIN_PULLER_DIR, gPullerDir ? HIGH : LOW);
}

static bool FeedMotorMoving() {
  if (commandedFeederRpm > 0.5f) return true;
  return feeder && fabsf(feeder->currentRPM()) > 0.5f;
}

static bool MotorPullerMoving() {
  if (commandedPullerRpm > 0.5f) return true;
  return puller && fabsf(puller->currentRPM()) > 0.5f;
}

// Parada suave en ciclo (requisito Class): no Brake — evita reset RMT.
static void motorSoftStopIfRunning(StepperRMT* m, float& commanded) {
  if (!m) return;
  if (commanded > 0.01f || m->targetRPM() > 0.01f || fabsf(m->currentRPM()) > 0.01f) {
    m->setSpeed(0.0f);
    commanded = 0.0f;
  }
}

// Igual que TCM: solo cambia el target cuando la consigna realmente cambió.
static void motorApplyRpm(StepperRMT* m, float& commanded, float targetRpm) {
  if (!m) return;
  if (targetRpm < RPM_MIN) {
    motorSoftStopIfRunning(m, commanded);
    return;
  }
  targetRpm = constrain(targetRpm, RPM_MIN, RPM_MAX);
  // La caché sola no basta: si la librería perdió/reseteó el target, reenviarlo.
  if (fabsf(commanded - targetRpm) <= 0.5f
      && fabsf(m->targetRPM() - targetRpm) <= 0.5f)
    return;
  if (m->setSpeed(targetRpm, MOTOR_ACCEL)) commanded = targetRpm;
  // Si falla, motorsCommand se ejecuta otra vez y reintenta sin reiniciar perfil.
}

static void motorsCommand(float feedRpm, float pullRpm) {
  // Nunca permitir JOG parcial: ambos canales deben estar inicializados.
  if (!motorsReady || !feeder || !puller) return;
  motorApplyRpm(feeder, commandedFeederRpm, feedRpm > 0.5f ? feedRpm : 0.0f);
  motorApplyRpm(puller, commandedPullerRpm, pullRpm > 0.5f ? pullRpm : 0.0f);
}

static bool motorsInit() {
  motorsReady = false;
  PullerDir(1);

  // Orden TCM: motor2 (GPIO 32) primero, luego feeder (GPIO 18).
  StepperRMT::Pins pPull = { PIN_PULLER_STEP, -1, -1, -1, -1, -1, -1 };
  puller = new StepperRMT(DRIVER_DM556T, pPull);
  if (!puller->begin()) {
    Serial.printf("[PF] puller begin FAIL GPIO %d\n", PIN_PULLER_STEP);
    delete puller;
    puller = nullptr;
    return false;
  }
  puller->setDefault(MOTOR_MICROSTEP, MOTOR_ACCEL, MOTOR_DECEL);
  puller->Brake();
  commandedPullerRpm = 0.0f;

  StepperRMT::Pins pFeed = { PIN_FEEDER_STEP, -1, -1, -1, -1, -1, -1 };
  feeder = new StepperRMT(DRIVER_DM556T, pFeed);
  if (!feeder->begin()) {
    Serial.printf("[PF] feeder begin FAIL GPIO %d\n", PIN_FEEDER_STEP);
    delete feeder;
    feeder = nullptr;
    return false;
  }
  feeder->setDefault(MOTOR_MICROSTEP, MOTOR_ACCEL, MOTOR_DECEL);
  feeder->Brake();
  commandedFeederRpm = 0.0f;

  motorsReady = true;
  Serial.printf("[PF] motors OK step F=%d P=%d\n", PIN_FEEDER_STEP, PIN_PULLER_STEP);
  return true;
}

// --- SensorMonitor: lectura y filtro. No mueve nada. ---

static SensorSnapshot gSnap;
static uint32_t holgHighMs = 0;
static uint32_t holgLowMs = 0;
static bool holguraStableOn = false;
static uint32_t limitHighMs = 0;
static uint32_t limitLowMs = 0;
static bool limitStableOn = false;
static bool sensorsReady = false;

static const SensorSnapshot& SensorMonitor(uint32_t nowMs) {
  if (!sensorsReady) {
    pinMode(PIN_CYLINDER, INPUT_PULLDOWN);
    pinMode(PIN_HOLGURA, INPUT_PULLUP);
    pinMode(PIN_LIMIT, INPUT_PULLUP);
    sensorsReady = true;
  }

  static uint32_t lastMs = 0;
  uint32_t dt = lastMs ? (nowMs - lastMs) : 0;
  lastMs = nowMs;

  gSnap.cylinderOpen = digitalRead(PIN_CYLINDER) == HIGH;

  if (digitalRead(PIN_HOLGURA) == HOLGURA_ON_LEVEL) {
    holgHighMs += dt;
    holgLowMs = 0;
    if (holgHighMs >= HOLGURA_ON_STABLE_MS) holguraStableOn = true;
  } else {
    holgLowMs += dt;
    holgHighMs = 0;
    if (holgLowMs >= HOLGURA_OFF_STABLE_MS) holguraStableOn = false;
  }
  gSnap.holguraOn = holguraStableOn;

  if (digitalRead(PIN_LIMIT) == LIMIT_ACTIVE_LEVEL) {
    limitHighMs += dt;
    limitLowMs = 0;
    if (limitHighMs >= LIMIT_ON_STABLE_MS) limitStableOn = true;
  } else {
    limitLowMs += dt;
    limitHighMs = 0;
    if (limitLowMs >= LIMIT_OFF_STABLE_MS) limitStableOn = false;
  }
  gSnap.limitActive = limitStableOn;
  return gSnap;
}

// --- FeedingMode: pide FeedMotor y MotorPuller según lo que vio SensorMonitor. ---

enum FeederMode : uint8_t { FEED_NONE = 0, FEED_HOLGURA };
enum HolguraPhase : uint8_t { HOLG_IDLE = 0, HOLG_WAIT_ON, HOLG_SETTLE, HOLG_TIMED_FEED };

static SystemFault fault = FAULT_NONE;
static bool feederDemand = false;
static FeederMode feederMode = FEED_NONE;
static HolguraPhase holguraPhase = HOLG_IDLE;
static uint32_t holguraSettleEndMs = 0;
static uint32_t holguraTimedEndMs = 0;
static bool holguraLastOn = false;
static bool holguraSawOn = false;
static bool holguraPostCycleOffWatch = false;
static uint32_t holguraOffSinceMs = 0;
static uint32_t holguraStallSinceMs = 0;

static void resetHolguraFeedState() {
  holguraPhase = HOLG_IDLE;
  holguraSettleEndMs = 0;
  holguraTimedEndMs = 0;
  holguraLastOn = false;
  holguraSawOn = false;
  holguraPostCycleOffWatch = false;
  holguraOffSinceMs = 0;
  holguraStallSinceMs = 0;
}

static void holguraOnRisingEdge(uint32_t nowMs, const FeedParams& p) {
  holguraSawOn = true;
  holguraPhase = HOLG_SETTLE;
  holguraSettleEndMs = nowMs + p.settleMs;
  holguraTimedEndMs = 0;
  holguraOffSinceMs = 0;
}

static void stopFeederDemand() {
  feederDemand = false;
  feederMode = FEED_NONE;
  resetHolguraFeedState();
}

// HMI (gForceFeed) y botón físico (gJogBtn) son OR: soltar uno no corta si el otro sigue activo.
static bool forceFeedActive() {
  return gForceFeed || gJogBtn;
}

static bool feedMayRun() {
  return fault == FAULT_NONE;
}

static void applyFeedOutputs(const FeedParams& p) {
  const bool run = feederDemand && feedMayRun();
  motorsCommand(run ? p.rpm : 0.0f, run ? p.pullerRpm : 0.0f);
}

static void enterFault(SystemFault f) {
  fault = f;
  stopFeederDemand();
  applyFeedOutputs(paramsNow());
}

static void beginHolguraFeed(const SensorSnapshot& s) {
  if (s.holguraOn) return;
  holguraPostCycleOffWatch = false;
  feederMode = FEED_HOLGURA;
  holguraSawOn = false;
  holguraSettleEndMs = 0;
  holguraTimedEndMs = 0;
  holguraLastOn = false;
  holguraPhase = HOLG_WAIT_ON;
  feederDemand = true;
}

static void stopHolguraFeedCycle(uint32_t nowMs, const SensorSnapshot& s) {
  feederDemand = false;
  feederMode = FEED_NONE;
  holguraPhase = HOLG_IDLE;
  holguraTimedEndMs = 0;
  holguraLastOn = s.holguraOn;
  if (holguraSawOn) {
    holguraPostCycleOffWatch = true;
    holguraSawOn = false;
  }
  if (!s.holguraOn)
    holguraOffSinceMs = nowMs;
  else
    holguraOffSinceMs = 0;
}

static bool holguraOffFaultPaused() {
  return feederMode == FEED_HOLGURA &&
         (holguraPhase == HOLG_SETTLE || holguraPhase == HOLG_TIMED_FEED);
}

static void serviceHolguraOffFault(uint32_t nowMs, const SensorSnapshot& s, const FeedParams& p) {
  if (fault != FAULT_NONE) {
    holguraOffSinceMs = 0;
    return;
  }
  if (s.holguraOn || holguraOffFaultPaused()) {
    holguraOffSinceMs = 0;
    return;
  }
  if (!holguraOffSinceMs) holguraOffSinceMs = nowMs;
  if ((nowMs - holguraOffSinceMs) >= (uint32_t)(p.offFaultSec * 1000.0f))
    enterFault(FAULT_HOLGURA);
}

static void serviceHolguraStallFault(uint32_t nowMs, const SensorSnapshot& s, const FeedParams& p) {
  if (fault != FAULT_NONE || s.holguraOn || FeedMotorMoving() || MotorPullerMoving()) {
    holguraStallSinceMs = 0;
    return;
  }
  if (!holguraStallSinceMs) holguraStallSinceMs = nowMs;
  if ((nowMs - holguraStallSinceMs) >= (uint32_t)(p.stallFaultSec * 1000.0f))
    enterFault(FAULT_HOLGURA);
}

static void serviceHolguraFeed(uint32_t nowMs, const SensorSnapshot& s, const FeedParams& p) {
  if (feederMode != FEED_HOLGURA) return;
  const bool on = s.holguraOn;
  const bool rising = on && !holguraLastOn;
  holguraLastOn = on;

  if (holguraPhase == HOLG_WAIT_ON) {
    if (rising || (on && !holguraSawOn)) {
      holguraOnRisingEdge(nowMs, p);
      feederDemand = true;  // sigue a settle sin frenar el stepper
      return;
    }
    feederDemand = !on;
    return;
  }
  if (holguraPhase == HOLG_SETTLE) {
    feederDemand = true;
    if (holguraSettleEndMs && (int32_t)(nowMs - holguraSettleEndMs) >= 0) {
      holguraPhase = HOLG_TIMED_FEED;
      holguraTimedEndMs = nowMs + (uint32_t)(p.onSec * 1000.0f);
    }
    return;
  }
  if (holguraPhase == HOLG_TIMED_FEED) {
    feederDemand = true;
    if (holguraTimedEndMs && (int32_t)(nowMs - holguraTimedEndMs) >= 0)
      stopHolguraFeedCycle(nowMs, s);
    return;
  }
  holguraPhase = HOLG_WAIT_ON;
  feederDemand = !on;
}

static void FeedingMode(uint32_t nowMs) {
  const SensorSnapshot& s = SensorMonitor(nowMs);
  const FeedParams p = paramsNow();

  // JOG físico: misma ruta que force_feed de la HMI (nivel, no flanco).
  {
    static bool jogReady = false;
    static bool jogRawLast = false;
    static uint32_t jogStableAt = 0;
    if (!jogReady) {
      pinMode(PIN_JOG, INPUT_PULLUP);
      jogReady = true;
      jogRawLast = digitalRead(PIN_JOG) == JOG_ACTIVE_LEVEL;
      gJogBtn = jogRawLast;
      jogStableAt = nowMs;
    } else {
      const bool raw = digitalRead(PIN_JOG) == JOG_ACTIVE_LEVEL;
      if (raw != jogRawLast) {
        jogRawLast = raw;
        jogStableAt = nowMs;
      } else if ((uint32_t)(nowMs - jogStableAt) >= JOG_STABLE_MS) {
        gJogBtn = raw;
      }
    }
  }

  // JOG (HMI o boton): fuerza giro ya; sin fallas ni feed_lock.
  if (forceFeedActive()) {
    holguraOffSinceMs = 0;
    holguraStallSinceMs = 0;
    feederDemand = true;
    motorsCommand(p.rpm, p.pullerRpm);
    return;
  }

  if (fault == FAULT_LIMIT && !s.limitActive) fault = FAULT_NONE;
  if (fault == FAULT_CYLINDER && !s.cylinderOpen) fault = FAULT_NONE;
  // Holgura se limpia con sensor ON o si el operador omitio la vigilancia.
  if (fault == FAULT_HOLGURA && (s.holguraOn || gOmitHolgura)) fault = FAULT_NONE;

  // Limite / cilindro solo afectan el ciclo automatico.
  if (s.limitActive && fault != FAULT_LIMIT) enterFault(FAULT_LIMIT);
  else if (fault == FAULT_NONE) {
    if (s.cylinderOpen) enterFault(FAULT_CYLINDER);
  }

  if (gFeedLock) {
    stopFeederDemand();
    applyFeedOutputs(p);
    return;
  }

  if (fault == FAULT_NONE) {
    if (holguraPostCycleOffWatch && s.holguraOn) {
      holguraPostCycleOffWatch = false;
      holguraOffSinceMs = 0;
    }
    if (!feederDemand && !s.holguraOn)
      beginHolguraFeed(s);
    serviceHolguraFeed(nowMs, s, p);
  }

  if (!gOmitHolgura) {
    serviceHolguraOffFault(nowMs, s, p);
    serviceHolguraStallFault(nowMs, s, p);
  } else {
    holguraOffSinceMs = 0;
    holguraStallSinceMs = 0;
  }
  applyFeedOutputs(p);
}

// --- HTTP hacia el maestro. No corre en el lazo de alimentación. ---

static StatusPod gStatus;
static bool statusDirty = true;
static SemaphoreHandle_t statusMux = nullptr;
static bool wifiUp = false;
static uint32_t wifiRetryAt = 0;

static void publishStatus() {
  StatusPod n;
  n.fault = (uint8_t)fault;
  n.state = (uint8_t)pfResolveState(fault, feederDemand);
  n.cylinder = gSnap.cylinderOpen;
  n.holgura = gSnap.holguraOn;
  n.limit = gSnap.limitActive;
  if (!statusMux) return;
  if (xSemaphoreTake(statusMux, portMAX_DELAY)) {
    if (n.state != gStatus.state || n.fault != gStatus.fault || n.cylinder != gStatus.cylinder
        || n.holgura != gStatus.holgura || n.limit != gStatus.limit) {
      gStatus = n;
      statusDirty = true;
    }
    xSemaphoreGive(statusMux);
  }
}

static void wifiStart() {
  const IPAddress ip(PF_STA_IP_0, PF_STA_IP_1, PF_STA_IP_2, PF_STA_IP_3);
  const IPAddress gw(STA_GW_0, STA_GW_1, STA_GW_2, STA_GW_3);
  const IPAddress mask(STA_MASK_0, STA_MASK_1, STA_MASK_2, STA_MASK_3);
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.persistent(false);
  WiFi.setAutoReconnect(true);
  WiFi.setHostname(PF_HOSTNAME);
  if (!WiFi.config(ip, gw, mask, gw))
    Serial.println("[PF] WiFi.config() fallo");
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  wifiRetryAt = millis() + 10000;
}

static void wifiPoll() {
  const wl_status_t st = WiFi.status();
  const bool up = st == WL_CONNECTED;
  if (up != wifiUp) {
    wifiUp = up;
    if (statusMux && xSemaphoreTake(statusMux, portMAX_DELAY)) {
      statusDirty = true;
      xSemaphoreGive(statusMux);
    }
    if (up)
      Serial.printf("[PF] WiFi On %s\n", WiFi.localIP().toString().c_str());
    else
      Serial.println("[PF] WiFi Off");
  }
  if (up) return;
  if ((int32_t)(millis() - wifiRetryAt) < 0) return;
  wifiRetryAt = millis() + 10000;
  if (st == WL_IDLE_STATUS) return;
  WiFi.disconnect();
  WiFi.begin(WIFI_SSID, WIFI_PASS);
}

static bool jsonNum(const String& j, const char* key, float& out) {
  const String needle = String("\"") + key + "\":";
  const int i = j.indexOf(needle);
  if (i < 0) return false;
  out = j.substring(i + needle.length()).toFloat();
  return true;
}

static bool jsonBoolCfg(const String& j, const char* key, bool& out) {
  const String needle = String("\"") + key + "\":";
  const int i = j.indexOf(needle);
  if (i < 0) return false;
  const int at = i + needle.length();
  if (j.startsWith("true", at)) {
    out = true;
    return true;
  }
  if (j.startsWith("false", at)) {
    out = false;
    return true;
  }
  return false;
}

static String cfgJsonSlice(const String& body) {
  const int i = body.indexOf("\"cfg\":");
  if (i < 0) return body;
  const int start = body.indexOf('{', i);
  if (start < 0) return body;
  int depth = 0;
  for (int k = start; k < (int)body.length(); k++) {
    const char c = body.charAt(k);
    if (c == '{') depth++;
    else if (c == '}') {
      depth--;
      if (depth == 0) return body.substring(start, k + 1);
    }
  }
  return body;
}

static void applyConfigJson(const String& body) {
  const String cfg = cfgJsonSlice(body);
  FeedParams next = paramsNow();
  float v = 0;
  if (jsonNum(cfg, "rpm", v)) next.rpm = v;
  if (jsonNum(cfg, "puller_rpm", v)) next.pullerRpm = v;
  if (jsonNum(cfg, "settle_ms", v)) next.settleMs = (uint16_t)v;
  if (jsonNum(cfg, "on_s", v)) next.onSec = v;
  if (jsonNum(cfg, "off_fault_s", v)) next.offFaultSec = v;
  if (jsonNum(cfg, "stall_fault_s", v)) next.stallFaultSec = v;
  bool bit = false;
  if (jsonBoolCfg(body, "feed_lock", bit)) gFeedLock = bit;
  if (jsonBoolCfg(body, "omit_holgura", bit)) gOmitHolgura = bit;
  if (jsonBoolCfg(body, "force_feed", bit)) {
    if (bit != gForceFeed) gCfgPollUrgent = true;
    gForceFeed = bit;
  }
  clampFeed(next);
  if (!paramMux) return;
  if (xSemaphoreTake(paramMux, portMAX_DELAY)) {
    const bool changed = next.rpm != gFeed.rpm || next.pullerRpm != gFeed.pullerRpm
      || next.settleMs != gFeed.settleMs || next.onSec != gFeed.onSec
      || next.offFaultSec != gFeed.offFaultSec || next.stallFaultSec != gFeed.stallFaultSec;
    gFeed = next;
    xSemaphoreGive(paramMux);
    if (changed)
      Serial.printf("[PF] config rpm %.1f puller %.1f settle %u on %.2f\n",
                    next.rpm, next.pullerRpm, next.settleMs, next.onSec);
  }
}

static bool httpPostStatus(const StatusPod& pod) {
  const PfRunState state = (PfRunState)pod.state;
  String body;
  body.reserve(220);
  body += "{\"side\":\"";
  body += PF_SIDE_TAG;
  body += "\",\"state\":\"";
  body += pfStateName(state);
  body += "\",\"fault\":";
  body += String(pod.fault);
  body += ",\"sensors\":{\"cylinder\":";
  body += pod.cylinder ? "true" : "false";
  body += ",\"holgura\":";
  body += pod.holgura ? "true" : "false";
  body += ",\"limit\":";
  body += pod.limit ? "true" : "false";
  body += "}}";

  HTTPClient http;
  http.setConnectTimeout(600);
  http.setTimeout(800);
  const String url = String("http://") + PF_MASTER_HOST + PF_HTTP_STATUS;
  if (!http.begin(url)) return false;
  http.addHeader("Content-Type", "application/json");
  const int code = http.POST(body);
  http.end();
  return code == 200;
}

static bool httpGetConfig() {
  HTTPClient http;
  http.setConnectTimeout(600);
  http.setTimeout(800);
  const String url = String("http://") + PF_MASTER_HOST + PF_HTTP_CONFIG + "?side=" + PF_SIDE_TAG;
  if (!http.begin(url)) return false;
  const int code = http.GET();
  String body;
  if (code == 200) body = http.getString();
  http.end();
  if (code != 200) return false;
  applyConfigJson(body);
  return true;
}

static void netTask(void*) {
  uint32_t nextCfgMs = 0;
  uint32_t nextKeepMs = 0;
  uint32_t nextTryMs = 0;
  for (;;) {
    wifiPoll();
    if (WiFi.status() == WL_CONNECTED) {
      const uint32_t now = millis();
      StatusPod pod;
      bool send = false;
      if (statusMux && xSemaphoreTake(statusMux, portMAX_DELAY)) {
        pod = gStatus;
        const bool due = statusDirty || (int32_t)(now - nextKeepMs) >= 0;
        if (due && (int32_t)(now - nextTryMs) >= 0) {
          statusDirty = false;
          send = true;
        }
        xSemaphoreGive(statusMux);
      }
      if (send) {
        if (httpPostStatus(pod)) {
          nextKeepMs = now + 1500;
          nextTryMs = now;
        } else {
          nextTryMs = now + 500;
          if (statusMux && xSemaphoreTake(statusMux, portMAX_DELAY)) {
            statusDirty = true;
            xSemaphoreGive(statusMux);
          }
        }
      }
      const bool quickCfg = gForceFeed || gCfgPollUrgent;
      if (gCfgPollUrgent) gCfgPollUrgent = false;
      if (quickCfg || (int32_t)(now - nextCfgMs) >= 0) {
        nextCfgMs = now + (quickCfg ? 40u : 80u);
        httpGetConfig();
      }
    }
    vTaskDelay(pdMS_TO_TICKS(40));
  }
}

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.printf("\nPF %s -> %s | JOG - %s GPIO %u\n", PF_SIDE_TAG, PF_MASTER_HOST, PF_SIDE_TAG,
                (unsigned)PIN_JOG);
  clampFeed(gFeed);
  paramMux = xSemaphoreCreateMutex();
  statusMux = xSemaphoreCreateMutex();
  if (!motorsInit()) Serial.println("[PF] motorsInit incomplete");
  wifiStart();
  xTaskCreatePinnedToCore(netTask, "pfnet", 8192, nullptr, 1, nullptr, 0);
}

void loop() {
  FeedingMode(millis());
  publishStatus();
  delay(2);
}
