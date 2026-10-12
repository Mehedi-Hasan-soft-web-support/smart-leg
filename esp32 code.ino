/*
  SmartLeg ESP32 firmware  v2.0.0  (single file)

  Hardware : ESP32 DevKit V1 + BTS7960 43 A driver + 12 V wiper motor + 0.96" SSD1306 OLED + 3 push buttons
  Control  : 1) physical buttons   2) website / Kotlin app over the internet (Supabase)
             3) Kotlin app over Bluetooth Low Energy when there is no internet
             All three call the same start / stop / reset logic, so they always behave the same.
  Storage  : every session, pause, stop and setting change is saved to Supabase. With no Wi-Fi the
             events are buffered on the ESP32 (session results also in flash) and uploaded later.

  Arduino IDE setup
    Board package : "esp32" by Espressif, version 3.x            Board: "ESP32 Dev Module"
    Partition     : Tools > Partition Scheme > "Huge APP (3MB No OTA/1MB SPIFFS)"   <- needed (Wi-Fi + BLE + TLS)
    Libraries     : Adafruit SSD1306, Adafruit GFX Library, ArduinoJson (7.x), NimBLE-Arduino (2.x)

  Wiring
    BTS7960 RPWM -> GPIO25      BTS7960 LPWM -> GPIO26 (kept LOW, one direction)
    BTS7960 R_EN + L_EN (joined) -> GPIO27
    BTS7960 VCC -> 5V (or 3.3V), GND -> GND  (ESP32, driver and 12 V supply share ground)
    BTS7960 B+ / B- -> 12 V supply through a fuse;  M+ / M- -> wiper motor
    OLED SDA -> GPIO21, SCL -> GPIO22, VCC -> 3.3V, GND -> GND
    START button -> GPIO32 to GND      STOP button -> GPIO33 to GND      RESET button -> GPIO13 to GND
    (internal pull-ups are used, no resistors needed)

  Buttons
    START : press to start a session (motor ramps up gently). During a session a press pauses / resumes.
    STOP  : press to stop the motor at once and save the session as completed.
    RESET : hold 2 s to cancel the session (saved as cancelled), go back to READY and clear a fault.

  Safety
    * Put a latching emergency-stop switch in series with the 12 V motor supply. It works without any
      software. Optionally wire its second contact to GPIO14 and set ESTOP_SENSE_ENABLED 1.
    * Speed and session length are always clamped to the limits below, whatever the app or website asks.
    * A command from the internet older than 20 s is never executed (checked by the database).
    * A session started remotely pauses by itself if that link (internet or BLE) is lost for 30 s.
*/

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <Preferences.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <ArduinoJson.h>
#include <NimBLEDevice.h>
#include <esp_task_wdt.h>
#include <esp_timer.h>
#include <time.h>

// =====================================================================
// 1. YOUR SETTINGS
// =====================================================================
#define WIFI_SSID          "Me"
#define WIFI_PASSWORD      "mehedi113"

#define SUPABASE_URL       "https://aduprvzfdoaiucvxczcj.supabase.co"
// Public anon key (safe to ship; RLS protects the data). NEVER put the service_role key here.
#define SUPABASE_ANON_KEY  "eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJpc3MiOiJzdXBhYmFzZSIsInJlZiI6ImFkdXBydnpmZG9haXVjdnhjemNqIiwicm9sZSI6ImFub24iLCJpYXQiOjE3OTE2NDY4ODAsImV4cCI6MjEwNzIyMjg4MH0.SLhpTItfTyNjaPA5l55bki_es732vActEkU5F6FZds0"

// Must match the device row created by supabase/schema.sql (section 7)
#define DEVICE_CODE        "SL-01"
#define DEVICE_KEY         "f7e5360a357c8eb36095784613048b17"

// Bluetooth: name the app looks for, and the 6-digit PIN the phone asks for the first time it pairs
#define BLE_NAME           "SmartLeg-" DEVICE_CODE
#define BLE_PASSKEY        246810

// =====================================================================
// 2. HARDWARE AND LIMITS
// =====================================================================
#define FW_VERSION              "2.0.0"

#define PIN_RPWM                25
#define PIN_LPWM                26
#define PIN_MOTOR_EN            27
#define PIN_CURRENT_SENSE       34
#define PIN_BTN_START           32
#define PIN_BTN_STOP            33
#define PIN_BTN_RESET           13
#define PIN_ESTOP_SENSE         14

#define PWM_FREQ_HZ             20000   // BTS7960 max 25 kHz; try 1000 if the motor buzzes or lacks torque at low speed
#define PWM_RES_BITS            10      // duty 0..1023
#define DEFAULT_SPEED_PCT       50
#define MIN_SPEED_PCT           25      // below this a geared wiper motor stalls and heats up
#define MAX_SPEED_PCT           80      // hard cap for website, app and BLE
#define RAMP_PCT_PER_S          20      // soft start and gentle speed changes
#define DEFAULT_SESSION_MIN     20
#define MAX_SESSION_ACTIVE_MIN  60
#define MAX_PAUSE_MIN           15      // paused longer than this -> session "interrupted"
#define RECOVERY_WINDOW_MIN     10      // after a power cut the session can be resumed within this time

#define ESTOP_SENSE_ENABLED     0       // 1 = E-stop second contact wired to GPIO14 (open = pressed)
#define CURRENT_SENSE_ENABLED   0       // 1 = trip on BTS7960 R_IS (GPIO34) after calibrating OVERCURRENT_MV
#define OVERCURRENT_MV          900
#define OVERCURRENT_HOLD_MS     300

#define HEARTBEAT_MS            3000    // cloud sync and command pick-up interval
#define LINK_LOSS_PAUSE_MS      30000
#define MAX_EVENTS_PER_POST     10
#define BACKOFF_MIN_MS          2000
#define BACKOFF_MAX_MS          60000
#define NVS_SAVE_INTERVAL_MS    15000
#define BLE_STATUS_MS           1000

// TLS: 0 = encrypted, certificate not checked (works first time). 1 = verify with ROOT_CA.
#define TLS_VERIFY              0
static const char ROOT_CA[] PROGMEM = R"PEM(
-----BEGIN CERTIFICATE-----
paste the root CA of aduprvzfdoaiucvxczcj.supabase.co here
-----END CERTIFICATE-----
)PEM";

// BLE UUIDs (the Kotlin app uses the same values)
#define BLE_SERVICE_UUID   "7a1e0001-5e55-4c7b-9f3a-1c2d3e4f5a01"
#define BLE_STATUS_UUID    "7a1e0002-5e55-4c7b-9f3a-1c2d3e4f5a01"   // read + notify, JSON status every second
#define BLE_COMMAND_UUID   "7a1e0003-5e55-4c7b-9f3a-1c2d3e4f5a01"   // write (paired only): "start", "stop", ...
#define BLE_RESULT_UUID    "7a1e0004-5e55-4c7b-9f3a-1c2d3e4f5a01"   // notify, result of each command

// =====================================================================
// 3. STATE
//    Keep every struct/enum ABOVE the first function: the Arduino IDE inserts
//    function prototypes right before the first function it finds.
// =====================================================================
enum MState : uint8_t { ST_IDLE = 0, ST_RUNNING, ST_PAUSED, ST_FAULT };
enum Source : uint8_t { SRC_BUTTON = 0, SRC_WEB, SRC_BLE };

struct SessionState {               // saved in flash
  uint8_t  active;
  uint8_t  state;
  char     code[28];
  uint32_t seq;
  uint64_t totalMs, activeMs, pausedMs;
  uint32_t pauseStartedTotalS;
};

struct Evt {
  char     uid[44];
  char     type[20];
  char     sess[28];
  uint64_t monoMs;                  // UINT64_MAX = from a previous boot
  uint32_t epoch;                   // 0 = clock not synced yet
  char     details[160];
};

struct EndRec {                     // session results that survive a reboot until uploaded
  uint8_t  used;
  char     uid[44];
  char     sess[28];
  uint32_t epoch;
  char     details[160];
};

#define QUEUE_CAP   48
#define ENDREC_CAP  5
static Evt     q[QUEUE_CAP];
static int     qHead = 0, qCount = 0;
static EndRec  endRecs[ENDREC_CAP];
static SemaphoreHandle_t mtx;

static SessionState S;
static MState   mstate = ST_IDLE;
static char     faultReason[40] = "";
static uint32_t bootCount = 0, sessCounter = 0, sysSeq = 0;
static bool     recovered = false;
static uint64_t recoveryDeadline = 0;
static uint64_t lastTickMs = 0, lastSaveMs = 0;
static uint32_t curDuty = 0;
static Source   startedBy = SRC_BUTTON;

// settings (saved in flash)
static uint8_t  cfgSpeedPct   = DEFAULT_SPEED_PCT;
static uint16_t cfgSessionMin = DEFAULT_SESSION_MIN;
static int32_t  cfgVer        = 0;
struct PendingCfg { bool has; int32_t ver; int speed; int sessionMin; };
static PendingCfg pendingCfg = { false, 0, 0, 0 };

// commands from the cloud (net task) and BLE (BLE task) -> executed by loop()
struct Cmd { char id[40]; char text[24]; Source src; };
struct CmdResult { char id[40]; bool ok; char err[48]; };
#define CMD_CAP 6
static Cmd       inbox[CMD_CAP];      static int inboxCount = 0;
static CmdResult cmdOut[CMD_CAP * 2]; static int cmdOutCount = 0;
static char      recentCmdIds[8][40]; static int recentCmdPos = 0;

// link status
static volatile bool     wifiOk = false, syncOk = false, authError = false;
static volatile uint64_t lastSyncOkMs = 0;
static volatile bool     bleConnected = false, bleSecure = false;
static volatile uint64_t bleLostAt = 0;

Preferences prefs;
Adafruit_SSD1306 oled(128, 64, &Wire, -1);
static bool oledOk = false;

// What the OLED shows. Filled from the live state, then drawn by oledRender().
enum OledScreen : uint8_t { SCR_READY = 0, SCR_RUNNING, SCR_PAUSED, SCR_FAULT, SCR_PAIRING };
struct OledView {
  OledScreen screen;
  char header[22];       // top line, e.g. "WiFi OK" or "No WiFi | BLE"
  char big[10];          // large centred value, e.g. "14:52"
  char footer[22];       // bottom line, e.g. "RUN 50% ACT 05:08"
  uint8_t bigSize;       // text size of the big value (2 or 3)
  uint8_t progress;      // 0..100, thin bar under the header during a session
  bool blink;            // toggles every 500 ms (running dot)
};

static NimBLECharacteristic* bleStatusChr = nullptr;
static NimBLECharacteristic* bleResultChr = nullptr;

static inline uint64_t nowMs() { return (uint64_t)(esp_timer_get_time() / 1000ULL); }
static inline bool clockSynced() { return time(nullptr) > 1700000000; }
static const char* stateName(MState s) {
  switch (s) { case ST_RUNNING: return "running"; case ST_PAUSED: return "paused";
               case ST_FAULT: return "fault"; default: return "stopped"; }
}
static int clampSpeed(int p)      { return p < MIN_SPEED_PCT ? MIN_SPEED_PCT : (p > MAX_SPEED_PCT ? MAX_SPEED_PCT : p); }
static int clampSessionMin(int m) { return m < 1 ? 1 : (m > MAX_SESSION_ACTIVE_MIN ? MAX_SESSION_ACTIVE_MIN : m); }
static inline uint8_t dutyPct()   { return (uint8_t)((curDuty * 100UL + 511UL) / 1023UL); }
static const char* srcName(Source s) { return s == SRC_WEB ? "web" : s == SRC_BLE ? "ble" : "button"; }

// =====================================================================
// 4. PERSISTENCE AND EVENT QUEUE
// =====================================================================
void saveSession()  { prefs.putBytes("sess", &S, sizeof(S)); lastSaveMs = nowMs(); }
void saveEndRecs()  { prefs.putBytes("ends", endRecs, sizeof(endRecs)); }

static void qPushLocked(const Evt& e) {             // caller holds mtx
  if (qCount == QUEUE_CAP) {                         // full: drop the oldest event that is not a session result
    int drop = 0;
    for (int i = 0; i < qCount; i++) if (strcmp(q[(qHead + i) % QUEUE_CAP].type, "session_end") != 0) { drop = i; break; }
    for (int i = drop; i < qCount - 1; i++) q[(qHead + i) % QUEUE_CAP] = q[(qHead + i + 1) % QUEUE_CAP];
    qCount--;
  }
  q[(qHead + qCount) % QUEUE_CAP] = e;
  qCount++;
}

void enqueue(const char* type, const char* details = "{}", bool withSession = true) {
  Evt e = {};
  strlcpy(e.type, type, sizeof(e.type));
  e.monoMs = nowMs();
  e.epoch  = clockSynced() ? (uint32_t)time(nullptr) : 0;
  strlcpy(e.details, details, sizeof(e.details));
  xSemaphoreTake(mtx, portMAX_DELAY);
  if (withSession && S.active) {
    strlcpy(e.sess, S.code, sizeof(e.sess));
    S.seq++;
    snprintf(e.uid, sizeof(e.uid), "%s-e%03lu", S.code, (unsigned long)S.seq);
  } else {
    sysSeq++;
    snprintf(e.uid, sizeof(e.uid), "%s-b%05lu-%04lu", DEVICE_CODE, (unsigned long)bootCount, (unsigned long)sysSeq);
  }
  qPushLocked(e);
  if (strcmp(type, "session_end") == 0) {
    for (int i = 0; i < ENDREC_CAP; i++) if (!endRecs[i].used) {
      endRecs[i].used = 1;
      strlcpy(endRecs[i].uid, e.uid, sizeof(endRecs[i].uid));
      strlcpy(endRecs[i].sess, e.sess, sizeof(endRecs[i].sess));
      endRecs[i].epoch = e.epoch;
      strlcpy(endRecs[i].details, e.details, sizeof(endRecs[i].details));
      break;
    }
    saveEndRecs();
  }
  xSemaphoreGive(mtx);
  Serial.printf("[evt] %s %s %s\n", e.uid, e.type, e.details);
}

// =====================================================================
// 5. MOTOR
// =====================================================================
uint32_t targetDuty() { return (uint32_t)((1023UL * clampSpeed(cfgSpeedPct)) / 100UL); }

void motorOff() {
  curDuty = 0;
  ledcWrite(PIN_RPWM, 0);
  digitalWrite(PIN_MOTOR_EN, LOW);
}

void motorUpdate(uint32_t dtMs) {
  if (mstate != ST_RUNNING) { if (curDuty) motorOff(); return; }
  digitalWrite(PIN_MOTOR_EN, HIGH);
  uint32_t tgt = targetDuty();
  uint32_t step = (1023UL * RAMP_PCT_PER_S * dtMs) / 100000UL; if (step == 0) step = 1;
  if (curDuty < tgt)      curDuty = (curDuty + step > tgt) ? tgt : curDuty + step;
  else if (curDuty > tgt) curDuty = (curDuty < tgt + step) ? tgt : curDuty - step;
  ledcWrite(PIN_RPWM, curDuty);
}

static bool estopPressed() {
#if ESTOP_SENSE_ENABLED
  return digitalRead(PIN_ESTOP_SENSE) == HIGH;
#else
  return false;
#endif
}

// =====================================================================
// 6. SESSION LOGIC (shared by buttons, cloud and BLE)
// =====================================================================
void timingJson(char* buf, size_t n, const char* extra = nullptr) {
  snprintf(buf, n, "{\"total_s\":%lu,\"active_s\":%lu,\"paused_s\":%lu%s%s}",
           (unsigned long)(S.totalMs / 1000), (unsigned long)(S.activeMs / 1000),
           (unsigned long)(S.pausedMs / 1000), extra ? "," : "", extra ? extra : "");
}

void startSession(Source src) {
  memset(&S, 0, sizeof(S));
  sessCounter++;
  prefs.putUInt("scount", sessCounter);
  snprintf(S.code, sizeof(S.code), "%s-%06lu-%04lx", DEVICE_CODE, (unsigned long)sessCounter, (unsigned long)(esp_random() & 0xFFFF));
  S.active = 1; S.state = ST_RUNNING;
  mstate = ST_RUNNING;
  recovered = false;
  startedBy = src;
  char d[48]; snprintf(d, sizeof(d), "{\"source\":\"%s\"}", srcName(src));
  enqueue("session_start", d);
  saveSession();
}

void pauseSession(const char* why) {
  if (mstate != ST_RUNNING) return;
  mstate = ST_PAUSED; S.state = ST_PAUSED;
  S.pauseStartedTotalS = (uint32_t)(S.totalMs / 1000);
  motorOff();
  char extra[48]; snprintf(extra, sizeof(extra), "\"reason\":\"%s\"", why);
  char d[160]; timingJson(d, sizeof(d), extra);
  enqueue("pause", d);
  saveSession();
}

void resumeSession(Source src) {
  if (mstate != ST_PAUSED) return;
  mstate = ST_RUNNING; S.state = ST_RUNNING;
  recovered = false;
  startedBy = src;
  char extra[32]; snprintf(extra, sizeof(extra), "\"source\":\"%s\"", srcName(src));
  char d[160]; timingJson(d, sizeof(d), extra);
  enqueue("resume", d);
  saveSession();
}

void endSession(const char* status, const char* fault = nullptr) {
  if (!S.active) return;
  motorOff();
  if (strcmp(status, "completed") == 0 && S.activeMs < 30000) status = "cancelled";   // never really ran
  char extra[100];
  if (fault) snprintf(extra, sizeof(extra), "\"status\":\"%s\",\"fault\":\"%s\"", status, fault);
  else       snprintf(extra, sizeof(extra), "\"status\":\"%s\"", status);
  char d[160]; timingJson(d, sizeof(d), extra);
  enqueue("session_end", d);
  memset(&S, 0, sizeof(S));
  saveSession();
  recovered = false;
  startedBy = SRC_BUTTON;
  if (mstate != ST_FAULT) mstate = ST_IDLE;
}

void raiseFault(const char* reason) {
  motorOff();
  if (mstate == ST_FAULT) return;
  strlcpy(faultReason, reason, sizeof(faultReason));
  mstate = ST_FAULT;
  char d[80]; snprintf(d, sizeof(d), "{\"reason\":\"%s\"}", reason);
  enqueue("fault", d);
  if (S.active) endSession("faulted", reason);
}

void clearFault() {
  if (mstate != ST_FAULT || estopPressed()) return;
  mstate = ST_IDLE;
  faultReason[0] = 0;
  enqueue("fault_cleared", "{}", false);
}

void resetSession() {
  if (S.active) endSession("cancelled");
  if (mstate == ST_FAULT) clearFault();
}

void setSpeedLocal(int pct, Source src) {
  cfgSpeedPct = (uint8_t)clampSpeed(pct);
  prefs.putUChar("spd", cfgSpeedPct);
  char d[96]; snprintf(d, sizeof(d), "{\"source\":\"%s\",\"speed\":%u,\"session_min\":%u}", srcName(src), cfgSpeedPct, cfgSessionMin);
  enqueue("config_applied", d, false);
}

void setSessionMinLocal(int m, Source src) {
  cfgSessionMin = (uint16_t)clampSessionMin(m);
  prefs.putUShort("smin", cfgSessionMin);
  char d[96]; snprintf(d, sizeof(d), "{\"source\":\"%s\",\"speed\":%u,\"session_min\":%u}", srcName(src), cfgSpeedPct, cfgSessionMin);
  enqueue("config_applied", d, false);
}

/*
  One command handler for every remote source. Returns nullptr on success, otherwise the reason.
    start         start a new session, or resume a paused one
    stop          stop the motor and save the session as completed
    reset         cancel the session, back to READY, clear a fault
    pause         pause a running session (BLE / app)
    speed:<n>     set running speed in % (BLE / app; website uses its own setting)
    min:<n>       set session length in minutes (BLE / app)
    status        just send the status again
*/
const char* executeCommand(const char* cmd, Source src) {
  if (strcmp(cmd, "stop") == 0) { if (S.active) endSession("completed"); else motorOff(); return nullptr; }
  if (strcmp(cmd, "reset") == 0) {
    if (mstate == ST_FAULT && estopPressed()) { if (S.active) endSession("cancelled"); return "release the E-stop first"; }
    resetSession();
    return mstate == ST_FAULT ? "fault still active" : nullptr;
  }
  if (strcmp(cmd, "start") == 0 || strcmp(cmd, "resume") == 0) {
    if (estopPressed())       return "E-stop is pressed";
    if (mstate == ST_FAULT)   return "device in fault, press RESET";
    if (mstate == ST_RUNNING) return nullptr;
    if (mstate == ST_PAUSED)  { resumeSession(src); return nullptr; }
    if (strcmp(cmd, "resume") == 0) return "no paused session";
    startSession(src);
    return nullptr;
  }
  if (strcmp(cmd, "pause") == 0) {
    if (mstate != ST_RUNNING) return "not running";
    pauseSession(srcName(src));
    return nullptr;
  }
  if (strncmp(cmd, "speed:", 6) == 0) {
    int v = atoi(cmd + 6);
    if (v <= 0 || v > 100) return "speed must be 1..100";
    setSpeedLocal(v, src);
    return nullptr;
  }
  if (strncmp(cmd, "min:", 4) == 0) {
    int v = atoi(cmd + 4);
    if (v <= 0 || v > 240) return "minutes must be 1..60";
    setSessionMinLocal(v, src);
    return nullptr;
  }
  if (strcmp(cmd, "status") == 0) return nullptr;
  return "unknown command";
}

// called from the net task and the BLE task; loop() executes
static void pushCommand(const char* id, const char* text, Source src) {
  xSemaphoreTake(mtx, portMAX_DELAY);
  if (inboxCount < CMD_CAP) {
    strlcpy(inbox[inboxCount].id, id ? id : "", sizeof(inbox[0].id));
    strlcpy(inbox[inboxCount].text, text, sizeof(inbox[0].text));
    inbox[inboxCount].src = src;
    inboxCount++;
  }
  xSemaphoreGive(mtx);
}

void bleNotifyResult(const char* cmd, const char* err);
void bleNotifyStatus();

void processCommands() {
  Cmd box[CMD_CAP]; int n;
  xSemaphoreTake(mtx, portMAX_DELAY);
  n = inboxCount; memcpy(box, inbox, sizeof(Cmd) * n); inboxCount = 0;
  xSemaphoreGive(mtx);
  for (int i = 0; i < n; i++) {
    Cmd& c = box[i];
    if (c.id[0]) {                                     // cloud commands carry an id: run each id once
      bool seen = false;
      for (int k = 0; k < 8; k++) if (strcmp(recentCmdIds[k], c.id) == 0) seen = true;
      if (seen) continue;
      strlcpy(recentCmdIds[recentCmdPos], c.id, 40); recentCmdPos = (recentCmdPos + 1) % 8;
    }
    const char* err = executeCommand(c.text, c.src);
    if (strcmp(c.text, "status") != 0) {
      char d[120]; snprintf(d, sizeof(d), "{\"cmd\":\"%s\",\"source\":\"%s\",\"result\":\"%s\"}", c.text, srcName(c.src), err ? err : "ok");
      enqueue("remote_command", d);
    }
    if (c.src == SRC_WEB) {
      xSemaphoreTake(mtx, portMAX_DELAY);
      if (cmdOutCount < CMD_CAP * 2) {
        CmdResult& r = cmdOut[cmdOutCount++];
        strlcpy(r.id, c.id, sizeof(r.id)); r.ok = (err == nullptr); strlcpy(r.err, err ? err : "", sizeof(r.err));
      }
      xSemaphoreGive(mtx);
    } else if (c.src == SRC_BLE) {
      bleNotifyResult(c.text, err);
      bleNotifyStatus();
    }
    Serial.printf("[cmd] %s (%s) -> %s\n", c.text, srcName(c.src), err ? err : "ok");
  }
}

// settings sent by the website
void applyPendingConfig() {
  PendingCfg p;
  xSemaphoreTake(mtx, portMAX_DELAY); p = pendingCfg; pendingCfg.has = false; xSemaphoreGive(mtx);
  if (!p.has || p.ver == cfgVer) return;
  cfgSpeedPct   = (uint8_t)clampSpeed(p.speed);
  cfgSessionMin = (uint16_t)clampSessionMin(p.sessionMin);
  cfgVer        = p.ver;
  prefs.putUChar("spd", cfgSpeedPct);
  prefs.putUShort("smin", cfgSessionMin);
  prefs.putInt("cver", cfgVer);
  char d[140];
  snprintf(d, sizeof(d), "{\"source\":\"web\",\"ver\":%ld,\"requested_speed\":%d,\"speed\":%u,\"session_min\":%u}",
           (long)cfgVer, p.speed, (unsigned)cfgSpeedPct, (unsigned)cfgSessionMin);
  enqueue("config_applied", d, false);
}

// =====================================================================
// 7. BUTTONS
// =====================================================================
struct Button {
  uint8_t pin; bool down = false; bool lastRaw = false; uint64_t changedAt = 0, pressedAt = 0; bool longFired = false;
  void begin(uint8_t p) { pin = p; pinMode(pin, INPUT_PULLUP); }
  // 1 = short press (on release), 2 = held for longMs (once), 3 = pressed (down edge)
  int poll(uint32_t longMs) {
    bool raw = digitalRead(pin) == LOW;
    uint64_t t = nowMs();
    if (raw != lastRaw) { lastRaw = raw; changedAt = t; }
    if (t - changedAt < 30) return 0;
    if (raw && !down) { down = true; pressedAt = t; longFired = false; return 3; }
    if (raw && down && !longFired && longMs && t - pressedAt >= longMs) { longFired = true; return 2; }
    if (!raw && down) { down = false; if (!longFired) return 1; }
    return 0;
  }
};
Button btnStart, btnStop, btnReset;

void handleButtons() {
  int a = btnStart.poll(0);
  int b = btnStop.poll(0);
  int c = btnReset.poll(2000);
  if (a == 3) Serial.println("[btn] START pressed");
  if (b == 3) Serial.println("[btn] STOP pressed");
  if (c == 3) Serial.println("[btn] RESET pressed (hold 2 s)");
  if (b == 3) { executeCommand("stop", SRC_BUTTON); return; }            // STOP acts on the press itself
  if (c == 2) { executeCommand("reset", SRC_BUTTON); return; }
  if (a != 1) return;                                                     // START acts on release of a press
  if (mstate == ST_IDLE) {
    const char* err = executeCommand("start", SRC_BUTTON);
    if (err) Serial.printf("[btn] START refused: %s\n", err);
  }
  else if (mstate == ST_RUNNING) pauseSession("button");
  else if (mstate == ST_PAUSED)  resumeSession(SRC_BUTTON);
  else if (mstate == ST_FAULT)   Serial.println("[btn] START ignored: fault, hold RESET 2 s");
}

// =====================================================================
// 8. SAFETY AND TIMING
// =====================================================================
void checkSafety() {
  if (mstate == ST_RUNNING) {
    uint64_t t = nowMs();
    if (startedBy == SRC_WEB && t - lastSyncOkMs > LINK_LOSS_PAUSE_MS) pauseSession("link_lost");
    else if (startedBy == SRC_BLE && !bleConnected && bleLostAt && t - bleLostAt > LINK_LOSS_PAUSE_MS) pauseSession("ble_lost");
  }
  if (estopPressed()) { raiseFault("estop"); return; }
#if CURRENT_SENSE_ENABLED
  static uint64_t overSince = 0;
  if (mstate == ST_RUNNING) {
    uint32_t mv = analogReadMilliVolts(PIN_CURRENT_SENSE);
    if (mv > OVERCURRENT_MV) { if (!overSince) overSince = nowMs(); else if (nowMs() - overSince > OVERCURRENT_HOLD_MS) raiseFault("overcurrent"); }
    else overSince = 0;
  } else overSince = 0;
#endif
}

void tickTiming() {
  uint64_t t = nowMs();
  uint32_t dt = (uint32_t)(t - lastTickMs);
  lastTickMs = t;
  if (S.active) {
    xSemaphoreTake(mtx, portMAX_DELAY);
    S.totalMs += dt;
    if (mstate == ST_RUNNING) S.activeMs += dt;
    else if (mstate == ST_PAUSED) S.pausedMs += dt;
    xSemaphoreGive(mtx);
    if (mstate == ST_RUNNING && S.activeMs >= (uint64_t)clampSessionMin(cfgSessionMin) * 60000ULL) {
      char d[64]; snprintf(d, sizeof(d), "{\"limit\":\"session_minutes\",\"minutes\":%u}", (unsigned)cfgSessionMin);
      enqueue("limit_reached", d);
      endSession("completed");
    } else if (mstate == ST_PAUSED) {
      if (recovered && t > recoveryDeadline) endSession("interrupted");
      else if (!recovered && S.totalMs / 1000 - S.pauseStartedTotalS > (uint64_t)MAX_PAUSE_MIN * 60ULL) endSession("interrupted");
    }
    if (S.active && t - lastSaveMs > NVS_SAVE_INTERVAL_MS) saveSession();
  }
  motorUpdate(dt);
}

// =====================================================================
// 9. OLED
// =====================================================================
void fmtHMS(char* b, size_t n, uint64_t ms) {
  uint32_t s = ms / 1000;
  snprintf(b, n, "%02lu:%02lu:%02lu", (unsigned long)(s / 3600), (unsigned long)((s / 60) % 60), (unsigned long)(s % 60));
}
void fmtMS(char* b, size_t n, uint64_t ms) {           // "MM:SS" (sessions are at most 60 min)
  uint32_t s = (ms + 999) / 1000;
  snprintf(b, n, "%02lu:%02lu", (unsigned long)(s / 60), (unsigned long)(s % 60));
}

// OLED-RENDER-BEGIN  (pure drawing code, also used by the PC preview tool)
static void oledCenter(Adafruit_GFX& g, const char* txt, int16_t y, uint8_t size) {
  int16_t x1, y1; uint16_t w, h;
  g.setTextSize(size);
  g.getTextBounds(txt, 0, 0, &x1, &y1, &w, &h);
  g.setCursor((128 - (int16_t)w) / 2, y);
  g.print(txt);
}

// Rounded double frame, centred header with divider, big value, footer (like the reference design)
void oledRender(Adafruit_GFX& g, const OledView& v) {
  g.fillScreen(0);
  g.setTextColor(1);
  g.setTextWrap(false);
  g.drawRoundRect(0, 0, 128, 64, 9, 1);              // outer rounded border
  g.drawRoundRect(2, 2, 124, 60, 7, 1);              // inner rounded border
  oledCenter(g, v.header, 6, 1);
  if (v.screen == SCR_RUNNING || v.screen == SCR_PAUSED) {
    int w = (int)(106L * v.progress / 100);
    g.drawRoundRect(10, 15, 108, 5, 2, 1);           // divider becomes a progress bar during a session
    if (w > 0) g.fillRoundRect(11, 16, w, 3, 1, 1);
    if (v.screen == SCR_RUNNING && v.blink) g.fillCircle(116, 9, 2, 1);   // "alive" dot
  } else {
    g.drawFastHLine(9, 16, 110, 1);                  // divider under the header
  }
  int16_t bigY = v.bigSize >= 3 ? 25 : 29;
  oledCenter(g, v.big, bigY, v.bigSize);
  oledCenter(g, v.footer, 51, 1);
}
// OLED-RENDER-END

void drawOled() {
  if (!oledOk) return;
  static uint64_t last = 0;
  if (nowMs() - last < 250) return;
  last = nowMs();
  int pending; xSemaphoreTake(mtx, portMAX_DELAY); pending = qCount; xSemaphoreGive(mtx);

  OledView v = {};
  v.blink = (nowMs() / 500) % 2;
  v.bigSize = 3;

  // header: connection state
  const char* net = authError ? "Key error" : (wifiOk ? (syncOk ? "WiFi OK" : "WiFi ...") : "No WiFi");
  if (bleConnected) snprintf(v.header, sizeof(v.header), "%s + BLE", net);
  else if (pending > 0 && !syncOk) snprintf(v.header, sizeof(v.header), "%s (%d)", net, pending > 99 ? 99 : pending);
  else strlcpy(v.header, net, sizeof(v.header));

  uint64_t goalMs = (uint64_t)clampSessionMin(cfgSessionMin) * 60000ULL;
  char act[10];
  if (bleConnected && !bleSecure) {                   // first pairing with a phone
    v.screen = SCR_PAIRING;
    strlcpy(v.header, "Bluetooth pairing", sizeof(v.header));
    snprintf(v.big, sizeof(v.big), "%06lu", (unsigned long)BLE_PASSKEY);
    v.bigSize = 2;
    strlcpy(v.footer, "ENTER PIN ON PHONE", sizeof(v.footer));
  } else if (mstate == ST_FAULT) {
    v.screen = SCR_FAULT;
    strlcpy(v.big, "FAULT", sizeof(v.big));
    snprintf(v.header, sizeof(v.header), "Fault: %s", faultReason);
    strlcpy(v.footer, "HOLD RESET 2 SEC", sizeof(v.footer));
  } else if (S.active) {
    v.screen = (mstate == ST_PAUSED) ? SCR_PAUSED : SCR_RUNNING;
    fmtMS(v.big, sizeof(v.big), S.activeMs < goalMs ? goalMs - S.activeMs : 0);     // time left
    v.progress = (uint8_t)(S.activeMs >= goalMs ? 100 : (S.activeMs * 100 / goalMs));
    fmtMS(act, sizeof(act), S.activeMs);
    if (v.screen == SCR_RUNNING)  snprintf(v.footer, sizeof(v.footer), "RUN %u%% ACT %s", (unsigned)dutyPct(), act);
    else if (recovered)           strlcpy(v.footer, "START = RESUME", sizeof(v.footer));
    else                          snprintf(v.footer, sizeof(v.footer), "PAUSED ACT %s", act);
  } else {
    v.screen = SCR_READY;
    fmtMS(v.big, sizeof(v.big), goalMs);              // full session length
    if (!wifiOk && !bleConnected) strlcpy(v.footer, "WIFI NOT FOUND", sizeof(v.footer));
    else snprintf(v.footer, sizeof(v.footer), "PRESS START  %u%%", (unsigned)cfgSpeedPct);
  }
  oledRender(oled, v);
  oled.display();
}

// =====================================================================
// 10. BLUETOOTH LOW ENERGY (for the Kotlin app when there is no internet)
// =====================================================================
class ServerCB : public NimBLEServerCallbacks {
  void onConnect(NimBLEServer* s, NimBLEConnInfo& info) override {
    bleConnected = true; bleSecure = info.isEncrypted(); bleLostAt = 0;
    NimBLEDevice::startSecurity(info.getConnHandle());       // ask the phone to pair (PIN on the OLED)
  }
  void onDisconnect(NimBLEServer* s, NimBLEConnInfo& info, int reason) override {
    bleConnected = false; bleSecure = false; bleLostAt = nowMs();
  }
  uint32_t onPassKeyDisplay() override { return BLE_PASSKEY; }
  void onAuthenticationComplete(NimBLEConnInfo& info) override {
    bleSecure = info.isEncrypted() && info.isAuthenticated();
    if (!bleSecure) NimBLEDevice::getServer()->disconnect(info.getConnHandle());
  }
};

class CommandCB : public NimBLECharacteristicCallbacks {
  void onWrite(NimBLECharacteristic* c, NimBLEConnInfo& info) override {
    std::string v = c->getValue();
    char text[24] = {0};
    size_t n = 0;
    for (char ch : v) {                                    // keep it simple: lower-case, no spaces
      if (n >= sizeof(text) - 1) break;
      if (ch == ' ' || ch == '\r' || ch == '\n') continue;
      text[n++] = (ch >= 'A' && ch <= 'Z') ? ch + 32 : ch;
    }
    if (n) pushCommand(nullptr, text, SRC_BLE);
  }
};

static ServerCB  serverCB;
static CommandCB commandCB;

void bleBegin() {
  NimBLEDevice::init(BLE_NAME);
  NimBLEDevice::setPower(9);                               // dBm
  NimBLEDevice::setSecurityAuth(true, true, true);         // bonding, MITM protection, secure connections
  NimBLEDevice::setSecurityPasskey(BLE_PASSKEY);
  NimBLEDevice::setSecurityIOCap(BLE_HS_IO_DISPLAY_ONLY);  // the ESP32 "displays" the PIN (on the OLED)

  NimBLEServer* server = NimBLEDevice::createServer();
  server->setCallbacks(&serverCB);
  server->advertiseOnDisconnect(true);
  NimBLEService* svc = server->createService(BLE_SERVICE_UUID);
  bleStatusChr = svc->createCharacteristic(BLE_STATUS_UUID, NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::READ_ENC | NIMBLE_PROPERTY::NOTIFY);
  NimBLECharacteristic* cmdChr = svc->createCharacteristic(BLE_COMMAND_UUID,
      NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_ENC | NIMBLE_PROPERTY::WRITE_AUTHEN);
  cmdChr->setCallbacks(&commandCB);
  bleResultChr = svc->createCharacteristic(BLE_RESULT_UUID, NIMBLE_PROPERTY::NOTIFY);
  server->start();

  NimBLEAdvertising* adv = NimBLEDevice::getAdvertising();
  adv->setName(BLE_NAME);
  adv->addServiceUUID(BLE_SERVICE_UUID);
  adv->enableScanResponse(true);
  adv->start();
  Serial.println("[ble] advertising as " BLE_NAME);
}

// {"st":"running","left":812,"t":420,"a":388,"p":32,"spd":50,"set":50,"min":20,"max":80,"sess":"SL-01-..","fault":"","net":1,"cloud":1,"by":"ble"}
void bleNotifyStatus() {
  if (!bleStatusChr) return;
  uint64_t goal = (uint64_t)clampSessionMin(cfgSessionMin) * 60000ULL;
  char buf[260];
  snprintf(buf, sizeof(buf),
    "{\"st\":\"%s\",\"left\":%lu,\"t\":%lu,\"a\":%lu,\"p\":%lu,\"spd\":%u,\"set\":%u,\"min\":%u,\"max\":%u,"
    "\"sess\":\"%s\",\"fault\":\"%s\",\"net\":%d,\"cloud\":%d,\"by\":\"%s\"}",
    stateName(mstate),
    (unsigned long)((S.active ? (S.activeMs < goal ? goal - S.activeMs : 0) : goal) / 1000),
    (unsigned long)(S.totalMs / 1000), (unsigned long)(S.activeMs / 1000), (unsigned long)(S.pausedMs / 1000),
    dutyPct(), (unsigned)cfgSpeedPct, (unsigned)cfgSessionMin, (unsigned)MAX_SPEED_PCT,
    S.active ? S.code : "", faultReason, wifiOk ? 1 : 0, syncOk ? 1 : 0, srcName(startedBy));
  bleStatusChr->setValue((uint8_t*)buf, strlen(buf));
  if (bleConnected && bleSecure) bleStatusChr->notify();
}

void bleNotifyResult(const char* cmd, const char* err) {
  if (!bleResultChr || !bleConnected) return;
  char buf[120];
  snprintf(buf, sizeof(buf), "{\"cmd\":\"%s\",\"ok\":%s,\"err\":\"%s\"}", cmd, err ? "false" : "true", err ? err : "");
  bleResultChr->setValue((uint8_t*)buf, strlen(buf));
  bleResultChr->notify();
}

// =====================================================================
// 11. CLOUD SYNC (Supabase, runs on core 0)
// =====================================================================
bool postBatch(int& httpCode) {
  Evt batch[MAX_EVENTS_PER_POST];
  int n, pending;
  SessionState snap; MState ms;
  xSemaphoreTake(mtx, portMAX_DELAY);
  n = qCount < MAX_EVENTS_PER_POST ? qCount : MAX_EVENTS_PER_POST;
  for (int i = 0; i < n; i++) batch[i] = q[(qHead + i) % QUEUE_CAP];
  snap = S; ms = mstate; pending = qCount;
  uint8_t speedNow = dutyPct(), speedSet = (uint8_t)clampSpeed(cfgSpeedPct);
  uint16_t sessMin = cfgSessionMin; int32_t ver = cfgVer;
  CmdResult results[CMD_CAP * 2]; int nRes = cmdOutCount;
  memcpy(results, cmdOut, sizeof(CmdResult) * nRes);
  xSemaphoreGive(mtx);

  JsonDocument doc;
  doc["p_device_code"] = DEVICE_CODE;
  doc["p_device_key"]  = DEVICE_KEY;
  JsonObject p = doc["p_payload"].to<JsonObject>();
  p["fw"] = FW_VERSION;
  p["rssi"] = WiFi.RSSI();
  p["state"] = stateName(ms);
  p["pending"] = pending;
  p["speed"] = speedNow;
  p["speed_set"] = speedSet;
  p["speed_max"] = MAX_SPEED_PCT;
  p["session_min"] = sessMin;
  p["cfg_ver"] = ver;
  if (nRes) {
    JsonArray cr = p["cmd_results"].to<JsonArray>();
    for (int i = 0; i < nRes; i++) {
      JsonObject o = cr.add<JsonObject>();
      o["id"] = results[i].id; o["ok"] = results[i].ok;
      if (!results[i].ok) o["error"] = results[i].err;
    }
  }
  if (snap.active) {
    JsonObject s = p["session"].to<JsonObject>();
    s["code"] = snap.code;
    s["status"] = (ms == ST_PAUSED) ? "paused" : "active";
    s["total_s"] = (uint32_t)(snap.totalMs / 1000);
    s["active_s"] = (uint32_t)(snap.activeMs / 1000);
    s["paused_s"] = (uint32_t)(snap.pausedMs / 1000);
  }
  JsonArray ev = p["events"].to<JsonArray>();
  uint64_t tNow = nowMs();
  for (int i = 0; i < n; i++) {
    JsonObject e = ev.add<JsonObject>();
    e["uid"] = batch[i].uid;
    e["type"] = batch[i].type;
    if (batch[i].sess[0]) e["session"] = batch[i].sess;
    uint32_t ts = batch[i].epoch;
    if (!ts && clockSynced() && batch[i].monoMs != UINT64_MAX)
      ts = (uint32_t)time(nullptr) - (uint32_t)((tNow - batch[i].monoMs) / 1000);
    e["ts"] = ts;
    e["details"] = serialized(batch[i].details);
  }
  String body; serializeJson(doc, body);

  WiFiClientSecure client;
#if TLS_VERIFY
  client.setCACert(ROOT_CA);
#else
  client.setInsecure();
#endif
  client.setTimeout(10);
  HTTPClient http;
  http.setTimeout(10000);
  if (!http.begin(client, String(SUPABASE_URL) + "/rest/v1/rpc/device_ingest")) { httpCode = -1; return false; }
  http.addHeader("Content-Type", "application/json");
  http.addHeader("apikey", SUPABASE_ANON_KEY);
  http.addHeader("Authorization", String("Bearer ") + SUPABASE_ANON_KEY);
  httpCode = http.POST(body);
  String resp = http.getString();
  http.end();

  if (httpCode != 200) {
    Serial.printf("[net] HTTP %d %s\n", httpCode, resp.c_str());
    authError = (httpCode == 401 || httpCode == 403);
    return false;
  }
  authError = false;

  JsonDocument r;
  if (deserializeJson(r, resp)) return false;

  xSemaphoreTake(mtx, portMAX_DELAY);
  bool endsChanged = false;
  for (JsonVariant v : r["acked"].as<JsonArray>()) {          // drop events the server stored
    const char* u = v.as<const char*>();
    if (!u) continue;
    for (int i = 0; i < qCount; i++) {
      if (strcmp(q[(qHead + i) % QUEUE_CAP].uid, u) == 0) {
        for (int j = i; j < qCount - 1; j++) q[(qHead + j) % QUEUE_CAP] = q[(qHead + j + 1) % QUEUE_CAP];
        qCount--;
        break;
      }
    }
    for (int k = 0; k < ENDREC_CAP; k++)
      if (endRecs[k].used && strcmp(endRecs[k].uid, u) == 0) { endRecs[k].used = 0; endsChanged = true; }
  }
  if (endsChanged) saveEndRecs();
  if (nRes) {                                                   // reported results are done
    int keep = cmdOutCount - nRes;
    if (keep > 0) memmove(cmdOut, cmdOut + nRes, sizeof(CmdResult) * keep);
    cmdOutCount = keep > 0 ? keep : 0;
  }
  JsonObject cfg = r["config"];
  if (!cfg.isNull() && cfg["ver"].is<int>() && cfg["ver"].as<int>() != cfgVer)
    pendingCfg = { true, cfg["ver"].as<int>(), cfg["speed"] | (int)DEFAULT_SPEED_PCT, cfg["session_min"] | (int)DEFAULT_SESSION_MIN };
  xSemaphoreGive(mtx);

  for (JsonObject c : r["commands"].as<JsonArray>()) {          // START / STOP / RESET from website or app
    const char* id = c["id"]; const char* type = c["type"];
    if (id && type) pushCommand(id, type, SRC_WEB);
  }
  return true;
}

void netTask(void*) {
  uint32_t backoff = BACKOFF_MIN_MS;
  uint64_t nextAttempt = 0, lastPost = 0, lastBegin = 0;
  bool ntpStarted = false, wasConnected = false;

  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  for (;;) {
    bool conn = WiFi.status() == WL_CONNECTED;
    wifiOk = conn;
    if (conn != wasConnected) {
      wasConnected = conn;
      if (conn) {
        char d[48]; snprintf(d, sizeof(d), "{\"rssi\":%d}", WiFi.RSSI());
        enqueue("wifi_connected", d, false);
        if (!ntpStarted) { configTime(0, 0, "pool.ntp.org", "time.google.com"); ntpStarted = true; }
        nextAttempt = 0;
      } else {
        enqueue("wifi_lost", "{}", false);
        syncOk = false;
      }
    }
    if (!conn) {
      if (nowMs() - lastBegin > 15000) { lastBegin = nowMs(); WiFi.disconnect(); WiFi.begin(WIFI_SSID, WIFI_PASSWORD); }
      vTaskDelay(pdMS_TO_TICKS(500));
      continue;
    }
    int pending; xSemaphoreTake(mtx, portMAX_DELAY); pending = qCount; xSemaphoreGive(mtx);
    uint64_t t = nowMs();
    if (t >= nextAttempt && (pending > 0 || t - lastPost >= HEARTBEAT_MS)) {
      int code = 0;
      bool ok = postBatch(code);
      lastPost = nowMs();
      if (ok) { syncOk = true; lastSyncOkMs = lastPost; backoff = BACKOFF_MIN_MS; nextAttempt = 0; }
      else {
        syncOk = false;
        nextAttempt = lastPost + backoff;
        backoff = backoff * 2 > BACKOFF_MAX_MS ? BACKOFF_MAX_MS : backoff * 2;
      }
    }
    vTaskDelay(pdMS_TO_TICKS(200));
  }
}

// =====================================================================
// 12. SETUP / LOOP
// =====================================================================
void setup() {
  pinMode(PIN_MOTOR_EN, OUTPUT); digitalWrite(PIN_MOTOR_EN, LOW);     // motor OFF before anything else
  pinMode(PIN_LPWM, OUTPUT);     digitalWrite(PIN_LPWM, LOW);
  ledcAttach(PIN_RPWM, PWM_FREQ_HZ, PWM_RES_BITS);
  ledcWrite(PIN_RPWM, 0);

  Serial.begin(115200);
  delay(50);
  Serial.println("\nSmartLeg " FW_VERSION);

  btnStart.begin(PIN_BTN_START);
  btnStop.begin(PIN_BTN_STOP);
  btnReset.begin(PIN_BTN_RESET);
#if ESTOP_SENSE_ENABLED
  pinMode(PIN_ESTOP_SENSE, INPUT_PULLUP);
#endif
#if CURRENT_SENSE_ENABLED
  analogSetPinAttenuation(PIN_CURRENT_SENSE, ADC_11db);
#endif

  Wire.begin(21, 22);
  oledOk = oled.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  if (oledOk) { oled.clearDisplay(); oled.setTextColor(SSD1306_WHITE); oled.setCursor(0, 0); oled.print("SmartLeg " FW_VERSION); oled.display(); }

  mtx = xSemaphoreCreateMutex();
  prefs.begin("smartleg", false);
  bootCount     = prefs.getUInt("boots", 0) + 1; prefs.putUInt("boots", bootCount);
  sessCounter   = prefs.getUInt("scount", 0);
  cfgSpeedPct   = (uint8_t)clampSpeed(prefs.getUChar("spd", DEFAULT_SPEED_PCT));
  cfgSessionMin = (uint16_t)clampSessionMin(prefs.getUShort("smin", DEFAULT_SESSION_MIN));
  cfgVer        = prefs.getInt("cver", 0);

  if (prefs.getBytesLength("ends") == sizeof(endRecs)) {               // re-send session results not yet uploaded
    prefs.getBytes("ends", endRecs, sizeof(endRecs));
    for (int i = 0; i < ENDREC_CAP; i++) if (endRecs[i].used) {
      Evt e = {};
      strlcpy(e.uid, endRecs[i].uid, sizeof(e.uid));
      strlcpy(e.type, "session_end", sizeof(e.type));
      strlcpy(e.sess, endRecs[i].sess, sizeof(e.sess));
      strlcpy(e.details, endRecs[i].details, sizeof(e.details));
      e.epoch = endRecs[i].epoch; e.monoMs = UINT64_MAX;
      qPushLocked(e);
    }
  } else memset(endRecs, 0, sizeof(endRecs));

  char d[96];
  snprintf(d, sizeof(d), "{\"reset_reason\":%d,\"boot\":%lu,\"fw\":\"%s\"}", (int)esp_reset_reason(), (unsigned long)bootCount, FW_VERSION);
  enqueue("reboot", d, false);

  memset(&S, 0, sizeof(S));                                            // unfinished session after a power cut: restore PAUSED
  if (prefs.getBytesLength("sess") == sizeof(S)) {
    prefs.getBytes("sess", &S, sizeof(S));
    if (S.active) {
      mstate = ST_PAUSED; S.state = ST_PAUSED;
      S.pauseStartedTotalS = (uint32_t)(S.totalMs / 1000);
      recovered = true;
      recoveryDeadline = nowMs() + (uint64_t)RECOVERY_WINDOW_MIN * 60000ULL;
      char t[160]; timingJson(t, sizeof(t), "\"note\":\"restored paused after reboot\"");
      enqueue("session_recovered", t);
      saveSession();
    } else memset(&S, 0, sizeof(S));
  }

  bleBegin();

  esp_task_wdt_config_t wdt = { .timeout_ms = 5000, .idle_core_mask = 0, .trigger_panic = true };
  if (esp_task_wdt_init(&wdt) == ESP_ERR_INVALID_STATE) esp_task_wdt_reconfigure(&wdt);
  esp_task_wdt_add(NULL);

  xTaskCreatePinnedToCore(netTask, "net", 16384, nullptr, 1, nullptr, 0);
  lastTickMs = nowMs();
}

void loop() {
  esp_task_wdt_reset();
  checkSafety();
  applyPendingConfig();
  processCommands();
  handleButtons();
  tickTiming();
  drawOled();
  static uint64_t lastStatus = 0;
  if (bleConnected && nowMs() - lastStatus >= BLE_STATUS_MS) { lastStatus = nowMs(); bleNotifyStatus(); }
  delay(10);
}
