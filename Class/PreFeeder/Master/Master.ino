// Maestro: guarda y reparte parámetros (NVS).
// Edición solo desde HMI (serial set_config). Persisten hasta el próximo Guardar.
// Cada lado hace POST /api/status y GET /api/config de sus parámetros.

#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>
#include "config.h"
#include "page.h"
#include "../ErrorCode.h"

struct SideLive {
  PfSideParams cfg;
  uint32_t lastMs = 0;
  bool seen = false;
  uint8_t fault = FAULT_NONE;
  char state[8] = "idle";
  bool cylinder = false;
  bool holgura = false;
  bool limit = false;
  bool omitHolgura = false;
  bool forceFeed = false;
};

static WebServer server(80);
static Preferences gPrefs;
static bool wifiUp = false;
static uint32_t wifiRetryAt = 0;
static bool gFeedLock = true;
static SideLive liveL;
static SideLive liveR;

static SideLive* sideLive(char side) {
  if (side == 'L' || side == 'l') return &liveL;
  if (side == 'R' || side == 'r') return &liveR;
  return nullptr;
}

static void wifiStart() {
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.persistent(false);
  WiFi.setAutoReconnect(true);
  WiFi.setHostname(OTA_HOSTNAME);
  if (!WiFi.config(PF_MASTER_STA_IP, PF_MASTER_STA_GW, PF_MASTER_STA_MASK, PF_MASTER_STA_GW))
    Serial.println("[MASTER] WiFi.config() fallo");
  WiFi.begin(PF_MASTER_WIFI_SSID, PF_MASTER_WIFI_PASS);
  wifiRetryAt = millis() + 10000;
}

static void wifiPoll() {
  const wl_status_t st = WiFi.status();
  const bool up = st == WL_CONNECTED;
  if (up != wifiUp) {
    wifiUp = up;
    if (up)
      Serial.printf("[MASTER] WiFi On %s\n", WiFi.localIP().toString().c_str());
    else
      Serial.println("[MASTER] WiFi Off");
  }
  if (up) return;
  if ((int32_t)(millis() - wifiRetryAt) < 0) return;
  wifiRetryAt = millis() + 10000;
  if (st == WL_IDLE_STATUS) return;
  WiFi.disconnect();
  WiFi.begin(PF_MASTER_WIFI_SSID, PF_MASTER_WIFI_PASS);
}

static void clampSide(PfSideParams& p) {
  if (p.rpm < 1.0f) p.rpm = 1.0f;
  if (p.rpm > 600.0f) p.rpm = 600.0f;
  if (p.pullerRpm < 1.0f) p.pullerRpm = 1.0f;
  if (p.pullerRpm > 600.0f) p.pullerRpm = 600.0f;
  if (p.settleMs < 10) p.settleMs = 10;
  if (p.settleMs > 500) p.settleMs = 500;
  if (p.onSec < 0.1f) p.onSec = 0.1f;
  if (p.onSec > 120.0f) p.onSec = 120.0f;
  if (p.offFaultSec < 0.5f) p.offFaultSec = 0.5f;
  if (p.offFaultSec > 300.0f) p.offFaultSec = 300.0f;
  if (p.stallFaultSec < 0.5f) p.stallFaultSec = 0.5f;
  if (p.stallFaultSec > 60.0f) p.stallFaultSec = 60.0f;
}

static bool jsonNum(const String& j, const char* key, float& out) {
  const String needle = String("\"") + key + "\":";
  const int i = j.indexOf(needle);
  if (i < 0) return false;
  out = j.substring(i + needle.length()).toFloat();
  return true;
}

static bool jsonBool(const String& j, const char* key, bool& out) {
  const String needle = String("\"") + key + "\":";
  const int i = j.indexOf(needle);
  if (i < 0) return false;
  const int at = i + needle.length();
  if (j.startsWith("true", at)) { out = true; return true; }
  if (j.startsWith("false", at)) { out = false; return true; }
  return false;
}

static bool jsonSide(const String& j, char& side) {
  const int i = j.indexOf("\"side\":\"");
  if (i < 0) return false;
  const char c = j.charAt(i + 8);
  if (c == 'L' || c == 'l') { side = 'L'; return true; }
  if (c == 'R' || c == 'r') { side = 'R'; return true; }
  return false;
}

static void appendFloat(String& j, float v, uint8_t dec) {
  char b[20];
  snprintf(b, sizeof(b), dec == 2 ? "%.2f" : "%.1f", v);
  j += b;
}

static void appendCfg(String& j, const PfSideParams& c) {
  j += "\"cfg\":{\"rpm\":";
  appendFloat(j, c.rpm, 1);
  j += ",\"puller_rpm\":";
  appendFloat(j, c.pullerRpm, 1);
  j += ",\"settle_ms\":";
  j += String(c.settleMs);
  j += ",\"on_s\":";
  appendFloat(j, c.onSec, 2);
  j += ",\"off_fault_s\":";
  appendFloat(j, c.offFaultSec, 2);
  j += ",\"stall_fault_s\":";
  appendFloat(j, c.stallFaultSec, 2);
  j += "}";
}

static String slaveConfigJson(const SideLive& live) {
  String j;
  j.reserve(240);
  j += "{";
  appendCfg(j, live.cfg);
  j += ",\"feed_lock\":";
  j += gFeedLock ? "true" : "false";
  j += ",\"omit_holgura\":";
  j += live.omitHolgura ? "true" : "false";
  j += ",\"force_feed\":";
  j += live.forceFeed ? "true" : "false";
  j += "}";
  return j;
}

static uint8_t sideReportedFault(const SideLive& s) {
  if (s.omitHolgura && s.fault == FAULT_HOLGURA) return FAULT_NONE;
  return s.fault;
}

static bool sideEffectiveIdle(const SideLive& s) {
  // Holgura omitida: el esclavo puede reportar error/HOLGURA o quedar en
  // error tras limpiar fault; para listo/estabilizado cuenta como idle.
  if (s.omitHolgura && (s.fault == FAULT_HOLGURA || strcmp(s.state, "error") == 0))
    return true;
  return strcmp(s.state, "idle") == 0;
}

static bool masterStable(uint32_t now) {
  if (!sideOnline(liveL, now) || !sideOnline(liveR, now)) return false;
  if (sideReportedFault(liveL) != FAULT_NONE || sideReportedFault(liveR) != FAULT_NONE) return false;
  if (!sideEffectiveIdle(liveL) || !sideEffectiveIdle(liveR)) return false;
  return true;
}

static void emitPfFault(char side, uint8_t fault) {
  Serial.print(F("{\"op\":\"pf_fault\",\"side\":\""));
  Serial.print(side);
  Serial.print(F("\",\"fault\":"));
  Serial.print(fault);
  Serial.println(F("}"));
}

static bool sideOnline(const SideLive& s, uint32_t now) {
  return s.seen && (uint32_t)(now - s.lastMs) < 4000;
}

static void appendSide(String& j, const char* name, const SideLive& s, uint32_t now) {
  const bool on = sideOnline(s, now);
  const uint8_t faultCode = sideReportedFault(s);
  const SystemFault fault = (SystemFault)faultCode;
  const char* stateOut = s.state;
  if (s.omitHolgura && s.fault == FAULT_HOLGURA) stateOut = "idle";
  j += "\"";
  j += name;
  j += "\":{\"online\":";
  j += on ? "true" : "false";
  j += ",\"state\":\"";
  j += stateOut;
  j += "\",\"fault\":";
  j += String(faultCode);
  j += ",\"error\":\"";
  j += pfFaultName(fault);
  j += "\",\"sensors\":{\"cylinder\":";
  j += s.cylinder ? "true" : "false";
  j += ",\"holgura\":";
  j += s.holgura ? "true" : "false";
  j += ",\"limit\":";
  j += s.limit ? "true" : "false";
  j += "},";
  appendCfg(j, s.cfg);
  j += ",\"omit_holgura\":";
  j += s.omitHolgura ? "true" : "false";
  j += ",\"force_feed\":";
  j += s.forceFeed ? "true" : "false";
  j += "}";
}

static void fillStatusJson(String& j, bool serialEnvelope) {
  const uint32_t now = millis();
  const bool stable = masterStable(now);
  const bool ready = stable && gFeedLock;
  j.reserve(serialEnvelope ? 820 : 760);
  if (serialEnvelope) {
    j += "{\"op\":\"status\",\"mcu\":\"prefeeder\",";
    j += "\"feed_lock\":";
    j += gFeedLock ? "true" : "false";
    j += ",\"stable\":";
    j += stable ? "true" : "false";
    j += ",\"ready\":";
    j += ready ? "true" : "false";
    j += ",";
  } else {
    j += "{\"feed_lock\":";
    j += gFeedLock ? "true" : "false";
    j += ",\"stable\":";
    j += stable ? "true" : "false";
    j += ",\"ready\":";
    j += ready ? "true" : "false";
    j += ",";
  }
  appendSide(j, "L", liveL, now);
  j += ",";
  appendSide(j, "R", liveR, now);
  j += "}";
}

static void sendStatus() {
  String j;
  fillStatusJson(j, false);
  server.send(200, "application/json", j);
}

static bool cfgSame(const PfSideParams& a, const PfSideParams& b) {
  return a.rpm == b.rpm && a.pullerRpm == b.pullerRpm && a.settleMs == b.settleMs
    && a.onSec == b.onSec && a.offFaultSec == b.offFaultSec
    && a.stallFaultSec == b.stallFaultSec;
}

static void cfgLoadNvsSide(char side, PfSideParams& dst) {
  char key[12];
  snprintf(key, sizeof(key), "%c_rpm", side);
  dst.rpm = gPrefs.getFloat(key, dst.rpm);
  snprintf(key, sizeof(key), "%c_puller", side);
  dst.pullerRpm = gPrefs.getFloat(key, dst.pullerRpm);
  snprintf(key, sizeof(key), "%c_settle", side);
  dst.settleMs = (uint16_t)gPrefs.getUShort(key, dst.settleMs);
  snprintf(key, sizeof(key), "%c_on", side);
  dst.onSec = gPrefs.getFloat(key, dst.onSec);
  snprintf(key, sizeof(key), "%c_off", side);
  dst.offFaultSec = gPrefs.getFloat(key, dst.offFaultSec);
  snprintf(key, sizeof(key), "%c_stall", side);
  dst.stallFaultSec = gPrefs.getFloat(key, dst.stallFaultSec);
}

static void cfgSaveNvsSide(char side, const PfSideParams& src) {
  char key[12];
  snprintf(key, sizeof(key), "%c_rpm", side);
  gPrefs.putFloat(key, src.rpm);
  snprintf(key, sizeof(key), "%c_puller", side);
  gPrefs.putFloat(key, src.pullerRpm);
  snprintf(key, sizeof(key), "%c_settle", side);
  gPrefs.putUShort(key, src.settleMs);
  snprintf(key, sizeof(key), "%c_on", side);
  gPrefs.putFloat(key, src.onSec);
  snprintf(key, sizeof(key), "%c_off", side);
  gPrefs.putFloat(key, src.offFaultSec);
  snprintf(key, sizeof(key), "%c_stall", side);
  gPrefs.putFloat(key, src.stallFaultSec);
}

static void cfgLoadNvs() {
  liveL.cfg = PF_PARAMS_L;
  liveR.cfg = PF_PARAMS_R;
  if (!gPrefs.begin("pfmaster", true)) return;
  cfgLoadNvsSide('L', liveL.cfg);
  cfgLoadNvsSide('R', liveR.cfg);
  gPrefs.end();
  clampSide(liveL.cfg);
  clampSide(liveR.cfg);
}

static void cfgSaveNvs(char side, const PfSideParams& cfg) {
  if (side != 'L' && side != 'R') return;
  if (!gPrefs.begin("pfmaster", false)) return;
  cfgSaveNvsSide(side, cfg);
  gPrefs.end();
}

static bool applyCfgToSide(SideLive* live, char side, PfSideParams& cfg, bool& changed) {
  if (!live) return false;
  clampSide(cfg);
  changed = !cfgSame(live->cfg, cfg);
  if (changed) {
    live->cfg = cfg;
    cfgSaveNvs(side, live->cfg);
  }
  return true;
}

static void handleStatusPost() {
  if (!server.hasArg("plain")) {
    server.send(400, "text/plain", "json");
    return;
  }
  const String body = server.arg("plain");
  char side = 0;
  if (!jsonSide(body, side)) {
    server.send(400, "text/plain", "side");
    return;
  }
  SideLive* live = sideLive(side);
  if (!live) {
    server.send(404, "text/plain", "side");
    return;
  }
  const uint8_t prevReported = sideReportedFault(*live);
  float faultN = 0;
  if (jsonNum(body, "fault", faultN)) live->fault = (uint8_t)faultN;
  bool bit = false;
  const int si = body.indexOf("\"sensors\":");
  if (si >= 0) {
    const String sen = body.substring(si);
    if (jsonBool(sen, "cylinder", bit)) live->cylinder = bit;
    if (jsonBool(sen, "holgura", bit)) live->holgura = bit;
    if (jsonBool(sen, "limit", bit)) live->limit = bit;
  } else {
    if (jsonBool(body, "cylinder", bit)) live->cylinder = bit;
    if (jsonBool(body, "holgura", bit)) live->holgura = bit;
    if (jsonBool(body, "limit", bit)) live->limit = bit;
  }
  const int st = body.indexOf("\"state\":\"");
  if (st >= 0) {
    const String name = body.substring(st + 9, body.indexOf('"', st + 9));
    if (name == "idle" || name == "busy" || name == "error") {
      strncpy(live->state, name.c_str(), sizeof(live->state) - 1);
      live->state[sizeof(live->state) - 1] = 0;
    }
  }
  // Con omit activo no se sostiene ni se reinyecta FAULT_HOLGURA.
  // También hay que salir de "error": si solo se limpia fault, masterStable
  // sigue viendo state=error y nunca marca stable/ready (E099 en el HMI).
  if (live->omitHolgura && live->fault == FAULT_HOLGURA) {
    live->fault = FAULT_NONE;
    if (strcmp(live->state, "error") == 0) {
      strncpy(live->state, "idle", sizeof(live->state) - 1);
      live->state[sizeof(live->state) - 1] = 0;
    }
  }
  if (live->fault != FAULT_NONE) strncpy(live->state, "error", sizeof(live->state));
  live->lastMs = millis();
  live->seen = true;
  const uint8_t reported = sideReportedFault(*live);
  if (reported != prevReported) emitPfFault(side, reported);
  server.send(200, "text/plain", "ok");
}

static bool readCfg(const String& body, PfSideParams& cfg) {
  float v = 0;
  bool any = false;
  if (jsonNum(body, "rpm", v)) { cfg.rpm = v; any = true; }
  if (jsonNum(body, "puller_rpm", v)) { cfg.pullerRpm = v; any = true; }
  if (jsonNum(body, "settle_ms", v)) { cfg.settleMs = (uint16_t)v; any = true; }
  if (jsonNum(body, "on_s", v)) { cfg.onSec = v; any = true; }
  if (jsonNum(body, "off_fault_s", v)) { cfg.offFaultSec = v; any = true; }
  if (jsonNum(body, "stall_fault_s", v)) { cfg.stallFaultSec = v; any = true; }
  if (any) clampSide(cfg);
  return any;
}

static void handleConfigGet() {
  if (!server.hasArg("side")) {
    server.send(400, "text/plain", "side");
    return;
  }
  SideLive* live = sideLive(server.arg("side").charAt(0));
  if (!live) {
    server.send(404, "text/plain", "side");
    return;
  }
  server.send(200, "application/json", slaveConfigJson(*live));
}

static void replySetFeedLockSerial(const String& body) {
  bool locked = gFeedLock;
  if (jsonBool(body, "locked", locked)) gFeedLock = locked;
  Serial.print(F("{\"op\":\"set_feed_lock\",\"ok\":true,\"locked\":"));
  Serial.print(gFeedLock ? "true" : "false");
  Serial.println(F("}"));
}

static void replySetOmitSerial(const String& body) {
  char side = 0;
  if (!jsonSide(body, side)) {
    Serial.println(F("{\"op\":\"set_omit\",\"ok\":false}"));
    return;
  }
  SideLive* live = sideLive(side);
  if (!live) {
    Serial.println(F("{\"op\":\"set_omit\",\"ok\":false}"));
    return;
  }
  bool on = live->omitHolgura;
  if (jsonBool(body, "on", on)) live->omitHolgura = on;
  // Omitir debe liberar la falla de holgura ya latcheada (si no, Reset queda bloqueado).
  if (live->omitHolgura && live->fault == FAULT_HOLGURA) {
    live->fault = FAULT_NONE;
    if (strcmp(live->state, "error") == 0) {
      strncpy(live->state, "idle", sizeof(live->state));
      live->state[sizeof(live->state) - 1] = 0;
    }
    emitPfFault(side, FAULT_NONE);
  }
  Serial.print(F("{\"op\":\"set_omit\",\"ok\":true,\"side\":\""));
  Serial.print(side);
  Serial.print(F("\",\"on\":"));
  Serial.print(live->omitHolgura ? "true" : "false");
  Serial.println(F("}"));
}

static void replySetForceSerial(const String& body) {
  char side = 0;
  if (!jsonSide(body, side)) {
    Serial.println(F("{\"op\":\"set_force_feed\",\"ok\":false}"));
    return;
  }
  SideLive* live = sideLive(side);
  if (!live) {
    Serial.println(F("{\"op\":\"set_force_feed\",\"ok\":false}"));
    return;
  }
  bool on = false;
  if (!jsonBool(body, "on", on)) {
    Serial.println(F("{\"op\":\"set_force_feed\",\"ok\":false}"));
    return;
  }
  live->forceFeed = on;
  Serial.print(F("{\"op\":\"set_force_feed\",\"ok\":true,\"side\":\""));
  Serial.print(side);
  Serial.print(F("\",\"on\":"));
  Serial.print(on ? "true" : "false");
  Serial.println(F("}"));
}

// --- Serial hacia el orquestador (JSON por línea, como Motion/PLC/Andon) ---

static char gSerialLine[768];
static size_t gSerialLineLen = 0;

static void printSerialStatus() {
  String j;
  fillStatusJson(j, true);
  Serial.println(j);
}

static void handleSetConfigSerial(const String& body) {
  char side = 0;
  if (!jsonSide(body, side) && body.indexOf("\"side\":") < 0) {
    Serial.println(F("{\"op\":\"set_config\",\"ok\":false}"));
    return;
  }
  SideLive* live = sideLive(side);
  if (!live) {
    Serial.println(F("{\"op\":\"set_config\",\"ok\":false}"));
    return;
  }
  PfSideParams next = live->cfg;
  if (!readCfg(body, next)) {
    Serial.print(F("{\"op\":\"set_config\",\"ok\":true,\"side\":\""));
    Serial.print(side);
    Serial.println(F("\",\"changed\":false}"));
    return;
  }
  bool changed = false;
  applyCfgToSide(live, side, next, changed);
  if (changed)
    Serial.printf("[MASTER] config %c rpm %.1f (serial/nvs)\n", side, live->cfg.rpm);
  Serial.print(F("{\"op\":\"set_config\",\"ok\":true,\"side\":\""));
  Serial.print(side);
  Serial.print(F("\",\"changed\":"));
  Serial.print(changed ? "true" : "false");
  Serial.println(F("}"));
}

static void handleSerialLine(char* line) {
  while (*line == ' ' || *line == '\t') line++;
  if (*line == '\0') return;

  if (strstr(line, "\"op\":\"status\"")) {
    printSerialStatus();
    return;
  }
  if (strstr(line, "\"op\":\"set_config\"")) {
    handleSetConfigSerial(String(line));
    return;
  }
  if (strstr(line, "\"op\":\"ping\"")) {
    Serial.println(F("{\"op\":\"pong\",\"mcu\":\"prefeeder\"}"));
    return;
  }
  if (strstr(line, "\"op\":\"set_feed_lock\"")) {
    replySetFeedLockSerial(String(line));
    return;
  }
  if (strstr(line, "\"op\":\"set_omit\"")) {
    replySetOmitSerial(String(line));
    return;
  }
  if (strstr(line, "\"op\":\"set_force_feed\"")) {
    replySetForceSerial(String(line));
    return;
  }

  Serial.println(F("{\"op\":\"error\",\"msg\":\"unknown_command\"}"));
}

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

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println(F("{\"op\":\"boot\",\"mcu\":\"prefeeder\"}"));
  Serial.println("[MASTER] PreFeeder Master boot");
  cfgLoadNvs();
  Serial.printf("[MASTER] cfg L rpm %.1f puller %.1f | R rpm %.1f puller %.1f\n",
                liveL.cfg.rpm, liveL.cfg.pullerRpm, liveR.cfg.rpm, liveR.cfg.pullerRpm);
  wifiStart();
  server.on("/", HTTP_GET, []() {
    server.send(200, "text/plain", "prefeeder-master");
  });
  server.on(PF_HTTP_STATUS, HTTP_GET, sendStatus);
  server.on(PF_HTTP_STATUS, HTTP_POST, handleStatusPost);
  server.on(PF_HTTP_CONFIG, HTTP_GET, handleConfigGet);
  server.on("/ping", HTTP_GET, []() { server.send(200, "text/plain", "pong"); });
  server.begin();
}

void loop() {
  pollSerial();
  server.handleClient();
  wifiPoll();
  delay(2);
}
