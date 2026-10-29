/*
  ESP32 Peltier water-bath controller, v2 (agent-ready)

  Based on the v1 logger sketch. What changed, and why (see CHANGES.md for the data behind each point):
    - PID: integral is no longer wiped on every error sign flip, input is low-pass filtered, Kd lowered, Ki raised,
      integral is kept while idle. Every tuning value is now a runtime parameter ('params', 'get', 'set', 'save').
    - Guards: wrong-direction guard compares window endpoints on the filtered reading (a noise spike can no longer
      trip it). Open-circuit NTC readings (ADC saturation) are rejected instead of reading -33.7 C.
    - Schedule: every step waits for the water to reach the new target (90 min timeout), then holds for 1 hour.
    - Remote: MQTT over TLS in its own task (telemetry, events, commands), dead-man timer, OTA pull from a
      whitelisted URL. Manual f/r, SD and clock commands are refused when they arrive over MQTT.
    - Log file: same columns as v1, plus temp3f_c (the filtered value the controller actually uses) at the end.

  Forward (RPWM) = COOLING, Reverse (LPWM) = HEATING. NTC3 (water) drives the PID. NTC1/NTC2 are only logged.

  Wiring is unchanged from v1:
    D25 -> R_EN + L_EN, D27 -> RPWM, D26 -> LPWM (add 10k pull-downs on RPWM/LPWM)
    3V3 -> resistor -> D32 / D33 / D34 -> NTC1 / NTC2 / NTC3 -> GND
    SD: D5 CS, D18 SCK, D23 MISO, D19 MOSI

  Libraries: PubSubClient (Nick O'Leary). Everything else ships with the ESP32 Arduino core.
  No credentials are compiled in: after the first USB flash, set them with 'cfg wifi <ssid>;<pass>' and
  'cfg mqtt <host>;<port>;<user>;<pass>;<base>' (USB serial only), then 'reboot'.

  Commands (USB serial at 115200 "Newline", the PC program on TCP 3333, or MQTT <base>/cmd):
    Profile:  a, s, sp <C>, clear, guard
    Tuning:   kp/ki/kd <v>, pid, params, get <name>, set <name> <value>, save, defaults
    Remote:   hb, deadman <minutes|0>, tag <text>, ota <url>, reboot, ver
    Manual (USB/TCP only): f<0-255> (cool), r<0-255> (heat)
    SD (USB/TCP only): list, last, dump <n>, del <n>, sd
    Other:    time, time YYYY-MM-DD HH:MM:SS (USB/TCP only), ip, status, q

  CSV: datetime,time_ms,pwm_pct,direction,temp1_c,temp2_c,temp3_c,setpoint_c,pid_p,pid_i,pid_d,phase,temp3f_c
*/
#include <Arduino.h>
#include <SPI.h>
#include <SD.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <HTTPUpdate.h>
#include <ESPmDNS.h>
#include <Preferences.h>
#include <PubSubClient.h>
#include <esp_system.h>
#include <time.h>
#include <sys/time.h>
#include <stdarg.h>
#include <math.h>
#define FW_VERSION "2.0.1-agent"

// Firmware images are public, so no credentials are compiled in. WiFi and broker settings are stored in flash and
// set once over USB with the 'cfg' commands. 'ota' only accepts URLs under this prefix.
#define OTA_URL_PREFIX "https://github.com/goku03l/peltierbench_agent/releases/download/"
char cfgSsid[40] = "", cfgPass[64] = "";
char cfgHost[64] = "", cfgUser[32] = "", cfgMqPass[64] = "", cfgBase[48] = "peltier/rig";
uint16_t cfgPort = 8883;

// Hardware watchdog on the main loop: reboots the ESP32 if the program hangs (the profile then resumes)
#define USE_WATCHDOG 1
#if USE_WATCHDOG
  #define FEED_WDT() feedLoopWDT()
#else
  #define FEED_WDT() do {} while (0)
#endif

// ---------- WiFi / time ----------
const char* MDNS_NAME   = "motor-logger";
const uint16_t TCP_PORT = 3333;
const char* TZ_INFO     = "IST-5:30";
const char* NTP_SERVER1 = "pool.ntp.org";
const char* NTP_SERVER2 = "time.google.com";
const unsigned long WIFI_TIMEOUT_MS = 15000;
const unsigned long STATUS_MS       = 5000;
const unsigned long WIFI_KICK_MS    = 30000;
const unsigned long WIFI_RESET_MS   = 300000;
const bool TIME_IN_LIVE_LINE = true;

// ---------- Cloud (MQTT over TLS) ----------
const bool CLOUD_ON = true;
const unsigned long CLOUD_MS = 1000;           // telemetry once per second

// ---------- Pins ----------
#define EN_PIN      25
#define RPWM_PIN    27
#define LPWM_PIN    26
#define NTC1_PIN    32
#define NTC2_PIN    33
#define NTC3_PIN    34
#define SD_CS_PIN   5
#define SD_SCK_PIN  18
#define SD_MISO_PIN 23
#define SD_MOSI_PIN 19

// ---------- Motor ----------
const int   PWM_FREQ        = 20000;
const int   PWM_RES         = 8;
const unsigned long LOG_MS  = 100;

// ---------- NTC ----------
const float SERIES_R  = 8000.0;
const float NOMINAL_R = 10000.0;
const float BETA      = 3435.0;
const float SERIES_R3  = 10000.0;
const float NOMINAL_R3 = 10000.0;
const float BETA3      = 3950.0;
const float NOMINAL_T = 25.0;
const float VCC_MV    = 3300.0;
const float ADC_SAT_MV = 3050.0;      // the 11 dB ADC tops out near here: at/above it the NTC is open-circuit

// ---------- SD logging ----------
const unsigned long FLUSH_MS    = 1000;
const unsigned long SD_RETRY_MS = 30000;
const size_t MAX_LOG_BYTES      = 150UL * 1024UL * 1024UL;
const char* CSV_HEADER = "datetime,time_ms,pwm_pct,direction,temp1_c,temp2_c,temp3_c,setpoint_c,pid_p,pid_i,pid_d,phase,temp3f_c";
File logFile;
bool sdMounted = false;
bool sdOk      = false;
char currentName[16] = "";
unsigned long lastFlush = 0;
bool liveOutput = true;

// ---------- Network ----------
WiFiServer server(TCP_PORT);
WiFiClient client;
String tcpBuf;
bool wifiEnabled = false;
bool wifiWasConnected = false;
Print* replyTo = &Serial;

bool timeValid = false;

int  pwmValue  = 0;
int  direction = 0;     // 1 = forward (RPWM, COOLING), -1 = reverse (LPWM, HEATING), 0 = stop
bool autoMode  = true;

Preferences prefs;

// =============================================================================
//  MQTT bridge: runs in its own task so a slow TLS handshake can never stall the control loop
// =============================================================================
enum { KIND_TELE = 0, KIND_EVT = 1, KIND_RESP = 2, KIND_CMD = 3 };
struct Msg { uint8_t kind; char text[620]; };
QueueHandle_t qIn  = nullptr;          // commands arriving from MQTT
QueueHandle_t qOut = nullptr;          // telemetry / events / replies leaving
volatile bool mqttUp = false;
char tTele[96], tEvt[96], tResp[96], tCmd[96], tStatus[96];

static void enqueueOut(uint8_t kind, const char* text) {
  if (!qOut || !mqttUp) return;
  Msg m; m.kind = kind;
  strncpy(m.text, text, sizeof(m.text) - 1); m.text[sizeof(m.text) - 1] = 0;
  xQueueSend(qOut, &m, 0);             // never blocks; dropped if the queue is full
}

// Collects reply text line by line and queues each line for MQTT
class MqttPrint : public Print {
  char buf[460]; size_t n = 0;
 public:
  size_t write(uint8_t c) override {
    if (c == '\n' || c == '\r') { flush(); return 1; }
    if (n < sizeof(buf) - 1) buf[n++] = c;
    return 1;
  }
  size_t write(const uint8_t* b, size_t l) override { for (size_t i = 0; i < l; i++) write(b[i]); return l; }
  void flush() override { if (n) { buf[n] = 0; enqueueOut(KIND_RESP, buf); n = 0; } }
};
MqttPrint mqttPrint;

static WiFiClientSecure mqttNet;
static PubSubClient mqtt(mqttNet);

static void mqttCallback(char*, byte* payload, unsigned int len) {
  Msg m; m.kind = KIND_CMD;
  size_t n = len < sizeof(m.text) - 1 ? len : sizeof(m.text) - 1;
  memcpy(m.text, payload, n); m.text[n] = 0;
  xQueueSend(qIn, &m, 0);
}

static void mqttTask(void*) {
  mqttNet.setInsecure();               // encrypted, but the broker certificate is not checked (see CHANGES.md)
  mqtt.setServer(cfgHost, cfgPort);
  mqtt.setCallback(mqttCallback);
  mqtt.setBufferSize(900);
  mqtt.setKeepAlive(30);
  unsigned long nextTry = 0;
  Msg m;
  for (;;) {
    if (CLOUD_ON && cfgHost[0] && WiFi.status() == WL_CONNECTED) {
      if (!mqtt.connected()) {
        mqttUp = false;
        while (xQueueReceive(qOut, &m, 0) == pdTRUE) {}      // drop stale output
        if (millis() >= nextTry) {
          char id[24];
          snprintf(id, sizeof(id), "rig-%06x", (unsigned)(ESP.getEfuseMac() & 0xFFFFFF));
          if (mqtt.connect(id, cfgUser, cfgMqPass, tStatus, 1, true, "0")) {
            mqtt.publish(tStatus, "1", true);
            mqtt.subscribe(tCmd, 1);
            mqttUp = true;
          } else nextTry = millis() + 10000;
        }
      } else {
        mqtt.loop();
        while (xQueueReceive(qOut, &m, 0) == pdTRUE) {
          const char* t = m.kind == KIND_TELE ? tTele : (m.kind == KIND_EVT ? tEvt : tResp);
          mqtt.publish(t, m.text, m.kind == KIND_TELE);
        }
      }
    } else mqttUp = false;
    vTaskDelay(pdMS_TO_TICKS(20));
  }
}

// =============================================================================
//  PID + schedule
// =============================================================================

// ---- Runtime parameters (see the PARAMS table; 'set <name> <value>' changes them live) ----
const float HARD_CAP_COOL = 220.0f;    // compile-time ceilings: raise deliberately, after checking supply and driver
const float HARD_CAP_HEAT = 100.0f;
static float Kp = 60.0f;
static float Ki = 0.60f;               // v1: 0.15
static float Kd = 20.0f;               // v1: 150
static float pAlpha      = 0.95f;      // input low-pass on the water reading, per 100 ms sample (0 = off)
static float pMinCool    = 40.0f;      // v1: 60
static float pMinHeat    = 35.0f;      // v1: 50
static float pMaxCool    = 220.0f;
static float pMaxHeat    = 100.0f;
static float pStartErr   = 0.30f;      // idle -> regulating above this |err| (when the deadband is on)
static float pDeadband   = 0.10f;      // regulating -> idle below this |err|
static float pUseDead    = 1.0f;       // 1 = deadband with dwell, 0 = regulate continuously
static float pDwellS     = 30.0f;      // minimum time in each state when the deadband is on
static float pSpSlew     = 0.5f;       // setpoint ramp, C/s
static float pDFilter    = 0.70f;      // weight of the previous filtered value in the D term
static float pResetFlip  = 0.0f;       // v1 behaviour was 1: wipe the integral when the error changes sign
static float pIdleDecay  = 0.0f;       // v1 behaviour was 1: decay the integral by 2%/s while idle or saturated
static float pWrongDirC  = 2.0f;       // wrong-direction guard threshold, C

struct Param { const char* name; float* ptr; float lo, hi; };
static Param PARAMS[] = {
  {"kp", &Kp, 0, 500}, {"ki", &Ki, 0, 5}, {"kd", &Kd, 0, 1000}, {"alpha", &pAlpha, 0, 0.99f},
  {"mincool", &pMinCool, 0, 120}, {"minheat", &pMinHeat, 0, 120},
  {"maxcool", &pMaxCool, 0, HARD_CAP_COOL}, {"maxheat", &pMaxHeat, 0, HARD_CAP_HEAT},
  {"starterr", &pStartErr, 0.05f, 2}, {"deadband", &pDeadband, 0, 1}, {"usedead", &pUseDead, 0, 1},
  {"dwell", &pDwellS, 0, 120}, {"spslew", &pSpSlew, 0.05f, 2}, {"dfilt", &pDFilter, 0, 0.95f},
  {"resetflip", &pResetFlip, 0, 1}, {"idledecay", &pIdleDecay, 0, 1}, {"wdrift", &pWrongDirC, 0.5f, 10}
};
const int NPARAMS = sizeof(PARAMS) / sizeof(PARAMS[0]);

static inline float capCool() { return fminf(pMaxCool, HARD_CAP_COOL); }
static inline float capHeat() { return fminf(pMaxHeat, HARD_CAP_HEAT); }

// ---- Schedule ----
const float START_SETPOINT = 15.0f;
const float SETPOINT_STEP  = 2.5f;
const unsigned long STEP_INTERVAL_MS = 3600000UL;
const float MAX_SETPOINT   = 40.0f;
const float REACHED_BAND   = 0.5f;
const bool  END_AT_MAX     = true;
const unsigned long REACH_TIMEOUT_MS = 90UL * 60UL * 1000UL;   // applies to every step now

// ---- Floor detection ----
const unsigned long FLOOR_WINDOW_MS = 15UL * 60UL * 1000UL;
const float FLOOR_IMPROVE_C = 0.2f;

// ---- Hardware mapping ----
const int COOL_DIR = 1;

// ---- Control constants ----
const unsigned long PID_MS     = 1000;
const float PELTIER_SLEW_PER_S = 25.0f;
const float DIR_HYST_C         = 0.10f;
const float INT_LIMIT          = 200.0f;
const uint32_t REVERSAL_MIN_OFF_MS = 10000;

// ---- Cooling-progress watchdog ----
const float    CW_RANGE_LO_C         = 25.0f;
const float    CW_RANGE_HI_C         = 30.0f;
const float    CW_MIN_DROP_C_PER_MIN = 0.2f;
const uint32_t CW_WINDOW_MS          = 3UL * 60UL * 1000UL;
const uint32_t CW_COOLDOWN_MS        = 4UL * 60UL * 1000UL;
const uint8_t  CW_MAX_TRIPS          = 5;

// =============================================================================
//  Safety guardrails
// =============================================================================
const float SENSOR_MIN_C   = -10.0f;
const float SENSOR_MAX_C   = 90.0f;
const float SENSOR_SPIKE_C = 2.0f;
const int   SENSOR_SPIKE_N = 5;
const unsigned long SENSOR_FAULT_MS = 10000;
const unsigned long SENSOR_OK_MS    = 5000;

const float WATER_MIN_C       = 4.0f;
const float WATER_MIN_CLEAR_C = 6.0f;
const float WATER_MAX_C       = 45.0f;
const float WATER_MAX_CLEAR_C = 43.0f;
const float WATER_HARD_MAX_C  = 55.0f;

const float SP_MIN_C = 8.0f;

const unsigned long WRONGDIR_COOL_MS   = 5UL * 60UL * 1000UL;
const unsigned long WRONGDIR_HEAT_MS   = 10UL * 60UL * 1000UL;
const unsigned long WRONGDIR_COOLDOWN_MS = 5UL * 60UL * 1000UL;
const int WRONGDIR_MAX_TRIPS           = 3;

const unsigned long MANUAL_MAX_MS = 30UL * 60UL * 1000UL;

const bool RESUME_AFTER_REBOOT = true;
const unsigned long RESUME_MAX_GAP_S = 3600;
const unsigned long STATE_SAVE_MS    = 120000;

struct ZoneState {
  float    pidIntegral     = 0.0f;
  float    sp_ramp         = START_SETPOINT;
  float    pv_filt         = NAN;
  float    uSlew           = 0.0f;
  uint32_t cwStartMs       = 0;
  float    cwStartPv       = 0.0f;
  uint32_t cwCooldownEndMs = 0;
  bool     active          = false;
  uint32_t stateSinceMs    = 0;
  int8_t   lastDir         = 0;
  uint32_t zeroSinceMs     = 0;
  float    lastP = 0.0f, lastD = 0.0f;
  int      tgtRaw          = 0;
  bool     held            = false;
  int8_t   errSign         = 0;
  uint8_t  cwTrips         = 0;
  bool     cwUnhealthy     = false;
};

ZoneState zW;
float setpoint        = START_SETPOINT;
bool  setpointReached = false;
bool  reachTimedOut   = false;
bool  profileDone     = false;
unsigned long holdStart  = 0;
unsigned long reachStart = 0;
int   zTarget         = 0;
unsigned long pidLastCtl  = 0;
unsigned long pidLastFast = 0;
float lastPv = NAN;                    // filtered water temperature used by the controller
bool  rampLatch = false;               // phase label hysteresis

bool  floorSearch = true;
unsigned long floorSince = 0;
float floorRefT = NAN, floorMin = NAN;

bool  faultLatched = false;
char  faultText[72] = "";
bool  sensorFault  = false;
unsigned long blockedUntil = 0;
bool  coolAllowed  = true;
bool  heatAllowed  = true;
float lastGoodWater = NAN;
int   spikeCount   = 0;
unsigned long badSince = 0, goodSince = 0;
int   wdMode = 0;
unsigned long wdStart = 0;
float wdRef = NAN;
int   wdTrips = 0;
unsigned long wdLastTrip = 0;
unsigned long manualStartMs = 0;

// remote supervision
unsigned long deadmanMs = 0;           // 0 = disarmed
unsigned long lastRemote = 0;
char expTag[16] = "";

bool blocked(unsigned long now) {
  return blockedUntil != 0 && (int32_t)(blockedUntil - now) > 0;
}

const char* phaseName() {
  if (faultLatched) return "FAULT";
  if (!autoMode) return direction == 0 ? "STOPPED" : "MANUAL";
  if (profileDone) return "DONE";
  if (sensorFault) return "SENSOR_FAULT";
  if (blocked(millis())) return "COOLDOWN";
  if (!setpointReached) return "REACHING";
  if (reachTimedOut) return "UNREACHED";
  if (!isnan(lastPv)) {
    float e = fabsf(lastPv - setpoint);
    if (!rampLatch && e > REACHED_BAND + 0.3f) rampLatch = true;
    else if (rampLatch && e <= REACHED_BAND)   rampLatch = false;
  }
  return rampLatch ? "RAMPING" : "HOLD";
}

void handleCommand(String cmd);

// ================= Output helpers =================

bool clientOk() { return client && client.connected(); }

// Message to everyone: USB, PC program, and the MQTT events topic
void say(const char *fmt, ...) {
  char buf[256];
  va_list a; va_start(a, fmt); vsnprintf(buf, sizeof(buf), fmt, a); va_end(a);
  Serial.print(buf);
  if (clientOk()) client.print(buf);
  size_t n = strlen(buf);
  if (n && buf[n - 1] == '\n') buf[n - 1] = 0;
  if (buf[0]) enqueueOut(KIND_EVT, buf);
}

void reply(const char *fmt, ...) {
  char buf[256];
  va_list a; va_start(a, fmt); vsnprintf(buf, sizeof(buf), fmt, a); va_end(a);
  replyTo->print(buf);
}

void printHelp(Print &p) {
  p.println("# Profile:  a = start fresh (15C, +2.5C/h) | s = stop | sp <C> = set target now");
  p.println("# Safety:   guard = show guard status | clear = clear a latched fault");
  p.println("# Tuning:   kp/ki/kd <v> | pid | params | get <name> | set <name> <v> | save | defaults");
  p.println("# Remote:   hb | deadman <min|0> | tag <text> | ota <url> | reboot | ver");
  p.println("# Manual:   f<0-255> = cool | r<0-255> = heat  (USB/PC only, auto-stops after 30 min)");
  p.println("# SD:       list, last, dump <n>, del <n>, sd = reconnect SD card (USB/PC only)");
  p.println("# Time:     time   |   time YYYY-MM-DD HH:MM:SS");
  p.println("# Other:    ip | status | q = live data on/off");
}

// ================= Motor =================

void applyMotor() {
  digitalWrite(EN_PIN, faultLatched ? LOW : HIGH);
  if (direction == 1) {
    ledcWrite(LPWM_PIN, 0);
    ledcWrite(RPWM_PIN, pwmValue);
  } else if (direction == -1) {
    ledcWrite(RPWM_PIN, 0);
    ledcWrite(LPWM_PIN, pwmValue);
  } else {
    ledcWrite(RPWM_PIN, 0);
    ledcWrite(LPWM_PIN, 0);
  }
}

void stopMotor() {
  autoMode = false;
  direction = 0;
  pwmValue = 0;
  applyMotor();
}

void enforceLimits() {
  bool cut = false;
  if (direction == COOL_DIR  && !coolAllowed) cut = true;
  if (direction == -COOL_DIR && !heatAllowed) cut = true;
  if (cut) {
    direction = 0; pwmValue = 0;
    if (autoMode) { zW.uSlew = 0.0f; zTarget = 0; }
  }
  pwmValue = constrain(pwmValue, 0, 255);
  if (direction == 0) pwmValue = 0;
}

// ================= PID =================

static inline float clampf(float v, float lo, float hi) {
  return v < lo ? lo : (v > hi ? hi : v);
}

static inline uint8_t dutyWithMinF(float mag, uint8_t minDuty) {
  if (mag <= 0.0f) return 0;
  if (mag < (float)minDuty) return minDuty;
  if (mag > 255.0f) return 255;
  return (uint8_t)lroundf(mag);
}

static inline float lowpass(float prev, float x, float alpha) {
  return alpha * prev + (1.0f - alpha) * x;
}

static inline float slewToward(float current, float target, float maxStep) {
  if (target > current) return fminf(current + maxStep, target);
  if (target < current) return fmaxf(current - maxStep, target);
  return target;
}

// Returns a signed duty: negative = cooling, positive = heating, 0 = idle.
static int computeZoneControl(ZoneState& z, float setTemp, float pv, float dt, uint32_t nowMs) {
  z.lastP = z.lastD = 0.0f;
  if (isnan(pv)) { z.active = false; return 0; }

  if (fabsf(setTemp - z.sp_ramp) > 1.0f) z.sp_ramp = setTemp;
  else {
    float maxStep = pSpSlew * dt;
    if (setTemp > z.sp_ramp) z.sp_ramp = fminf(z.sp_ramp + maxStep, setTemp);
    else                     z.sp_ramp = fmaxf(z.sp_ramp - maxStep, setTemp);
  }

  float err_raw = z.sp_ramp - pv;      // positive = too cold -> heat, negative = too warm -> cool
  float aerr    = fabsf(err_raw);

  if (isnan(z.pv_filt)) z.pv_filt = pv;
  float pv_filt_prev = z.pv_filt;
  z.pv_filt = lowpass(z.pv_filt, pv, pDFilter);
  float dMeas = (z.pv_filt - pv_filt_prev) / dt;

  // v1 wiped the integral on every sign flip, which sensor noise triggers constantly near the setpoint.
  int8_t es = (err_raw > 0) ? 1 : -1;
  if (pResetFlip > 0.5f && z.errSign != 0 && es != z.errSign) z.pidIntegral = 0.0f;
  z.errSign = es;

  uint32_t dwellMs = (uint32_t)(pDwellS * 1000.0f);
  uint32_t inState = nowMs - z.stateSinceMs;
  bool farOut = aerr >= 1.0f;
  if (pUseDead > 0.5f) {
    if (!z.active) {
      if (aerr > pStartErr && (inState >= dwellMs || farOut)) { z.active = true; z.stateSinceMs = nowMs; }
    } else {
      if (aerr < pDeadband && inState >= dwellMs)             { z.active = false; z.stateSinceMs = nowMs; }
    }
  } else {
    z.active = true;
  }
  if (!z.active) {
    if (pIdleDecay > 0.5f) z.pidIntegral *= 0.98f;
    return 0;
  }

  float P = Kp * err_raw;
  float D = -Kd * dMeas;
  z.lastP = P; z.lastD = D;
  float outMin = -capCool(), outMax = capHeat();
  float u = clampf(P + z.pidIntegral + D, outMin, outMax);

  bool satHigh = (u >= outMax - 1e-6f) && err_raw > 0;
  bool satLow  = (u <= outMin + 1e-6f) && err_raw < 0;
  if (!satHigh && !satLow) z.pidIntegral = clampf(z.pidIntegral + Ki * err_raw * dt, -INT_LIMIT, INT_LIMIT);
  else if (pIdleDecay > 0.5f) z.pidIntegral *= 0.98f;

  if (err_raw <= -DIR_HYST_C && u > 0) u = 0;
  if (err_raw >=  DIR_HYST_C && u < 0) u = 0;

  if (u < 0.0f) return -(int)dutyWithMinF(-u, (uint8_t)pMinCool);
  return (int)dutyWithMinF(u, (uint8_t)pMinHeat);
}

static int polarityInterlockImpl(ZoneState& z, int target, uint32_t nowMs) {
  if (z.uSlew != 0.0f) { z.lastDir = (z.uSlew < 0) ? -1 : 1; z.zeroSinceMs = 0; }
  else if (z.zeroSinceMs == 0) z.zeroSinceMs = nowMs ? nowMs : 1;

  int8_t want = (target < 0) ? -1 : (target > 0 ? 1 : 0);
  if (want == 0 || z.lastDir == 0 || want == z.lastDir) return target;

  if (z.uSlew == 0.0f && (nowMs - z.zeroSinceMs) >= REVERSAL_MIN_OFF_MS) {
    z.lastDir = want;
    return target;
  }
  z.pidIntegral = 0.0f;
  return 0;
}

static int polarityInterlock(ZoneState& z, int target, uint32_t nowMs) {
  z.tgtRaw = target;
  int r = polarityInterlockImpl(z, target, nowMs);
  z.held = (target != 0 && r == 0);
  return r;
}

static bool coolingWatchdog(ZoneState& z, const char* name, float pv, uint32_t nowMs) {
  if (z.cwUnhealthy) return true;
  if (z.cwCooldownEndMs != 0) {
    if ((int32_t)(nowMs - z.cwCooldownEndMs) < 0) return true;
    z.cwCooldownEndMs = 0;
    z.cwStartMs = 0;
    say("# [CWD] %s cooldown over - cooling re-enabled\n", name);
  }
  bool fullLoad = z.uSlew <= -capCool();
  if (isnan(pv) || !fullLoad || pv < CW_RANGE_LO_C || pv > CW_RANGE_HI_C) { z.cwStartMs = 0; return false; }
  if (z.cwStartMs == 0) { z.cwStartMs = nowMs ? nowMs : 1; z.cwStartPv = pv; return false; }
  uint32_t elapsed = nowMs - z.cwStartMs;
  if (elapsed < CW_WINDOW_MS) return false;
  float minutes = elapsed / 60000.0f;
  float drop    = z.cwStartPv - pv;
  if (drop >= CW_MIN_DROP_C_PER_MIN * minutes) {
    z.cwStartMs = nowMs; z.cwStartPv = pv;
    z.cwTrips = 0;
    return false;
  }
  if (++z.cwTrips > CW_MAX_TRIPS) {
    z.cwUnhealthy = true;
    z.cwStartMs = 0;
    say("# [CWD] *** %s PELTIER UNHEALTHY *** cooling stalled %u times - Peltier OFF until 'a' or reboot. "
        "Check Peltier, water flow, heatsink.\n", name, (unsigned)z.cwTrips);
    return true;
  }
  z.cwCooldownEndMs = nowMs + CW_COOLDOWN_MS;
  z.cwStartMs = 0;
  say("# [CWD] %s cooling stalled: %.2fC drop in %.1f min (< %.1fC/min) - Peltier off %lu min\n",
      name, drop, minutes, CW_MIN_DROP_C_PER_MIN, (unsigned long)(CW_COOLDOWN_MS / 60000UL));
  return true;
}

void softResetController() {
  zW.pidIntegral = 0.0f;
  zW.pv_filt = NAN;
  zW.uSlew = 0.0f;
  zW.active = false;
  zW.errSign = 0;
  zW.cwStartMs = 0; zW.cwCooldownEndMs = 0; zW.cwTrips = 0; zW.cwUnhealthy = false;
  zTarget = 0;
  pidLastCtl = 0;
  pidLastFast = 0;
}

void resetPID() {
  int cur = 0;
  if (direction != 0 && pwmValue > 0) cur = (direction == COOL_DIR) ? -pwmValue : pwmValue;

  zW = ZoneState();
  zW.uSlew        = (float)cur;
  zW.lastDir      = (cur < 0) ? -1 : (cur > 0 ? 1 : 0);
  zW.stateSinceMs = millis();
  zTarget         = cur;
  setpoint        = START_SETPOINT;
  setpointReached = false;
  reachTimedOut   = false;
  profileDone     = false;
  holdStart       = 0;
  reachStart      = millis();
  zW.sp_ramp      = setpoint;
  pidLastCtl      = 0;
  pidLastFast     = 0;
  floorSearch     = true;
  floorSince      = 0;
  floorRefT       = NAN;
  floorMin        = NAN;
  rampLatch       = false;
}

void runPID(float pv) {
  unsigned long now = millis();

  if (pidLastCtl == 0) {
    pidLastCtl = now ? now : 1;
  } else if (now - pidLastCtl >= PID_MS) {
    float dt = (now - pidLastCtl) / 1000.0f;
    if (dt > 3.0f) dt = 3.0f;
    pidLastCtl = now;

    // ---- schedule ----
    if (!profileDone && !isnan(pv)) {
      if (!setpointReached) {
        if (fabsf(pv - setpoint) <= REACHED_BAND) {
          setpointReached = true;
          reachTimedOut = false;
          holdStart = now;
          say("# Target %.1f C reached - holding for 1 hour\n", setpoint);
        } else if (now - reachStart >= REACH_TIMEOUT_MS) {
          setpointReached = true;
          reachTimedOut = true;
          holdStart = now;
          floorSearch = false;
          say("# WARNING: could not reach %.1f C within %lu min (now %.1f C) - continuing the schedule anyway\n",
              setpoint, (unsigned long)(REACH_TIMEOUT_MS / 60000UL), pv);
        }
      } else if (now - holdStart >= STEP_INTERVAL_MS) {
        if (setpoint < MAX_SETPOINT) {
          setpoint = fminf(setpoint + SETPOINT_STEP, MAX_SETPOINT);
          // v1 kept the hold timer running through the step; now each step waits until the water gets there
          setpointReached = false;
          reachTimedOut = false;
          holdStart = 0;
          reachStart = now;
          floorSearch = false;
          say("# New target: %.1f C\n", setpoint);
        } else if (END_AT_MAX) {
          profileDone = true;
          say("# PROFILE COMPLETE - 1 hour held at %.1f C. Peltier off; logging continues. Type 'a' to run again.\n", setpoint);
        }
      }
    }

    if (profileDone) {
      zTarget = 0;
    } else {
      int target = computeZoneControl(zW, setpoint, pv, dt, now);
      if (coolingWatchdog(zW, "WATER", pv, now)) target = 0;
      target = polarityInterlock(zW, target, now);
      zTarget = target;

      if (floorSearch) {
        if (setpointReached) {
          floorSearch = false;
        } else if (!isnan(pv)) {
          float pf = isnan(zW.pv_filt) ? pv : zW.pv_filt;
          bool coolingHard = zW.uSlew <= -pMinCool;
          bool above = (pv - setpoint) > REACHED_BAND;
          if (!above || !coolingHard || isnan(floorRefT)) {
            floorSince = now; floorRefT = pf; floorMin = pf;
          } else {
            if (pf < floorRefT - FLOOR_IMPROVE_C) { floorRefT = pf; floorMin = pf; floorSince = now; }
            else if (pf < floorMin) floorMin = pf;
            if (now - floorSince >= FLOOR_WINDOW_MS) {
              float m = ceilf((floorMin - REACHED_BAND) / SETPOINT_STEP) * SETPOINT_STEP;
              if (m < setpoint + SETPOINT_STEP) m = setpoint + SETPOINT_STEP;
              if (m > MAX_SETPOINT) m = MAX_SETPOINT;
              say("# Cannot reach %.1f C (lowest %.1f C, no new low for %lu min) - new start target %.1f C\n",
                  setpoint, floorMin, (unsigned long)(FLOOR_WINDOW_MS / 60000UL), m);
              setpoint = m;
              setpointReached = false;
              holdStart = 0;
              reachStart = now;
              floorSearch = false;
            }
          }
        }
      }
    }
  }

  if (pidLastFast == 0) pidLastFast = now;
  float dtFast = (now - pidLastFast) / 1000.0f;
  if (dtFast > 0.5f) dtFast = 0.5f;
  pidLastFast = now;
  zW.uSlew = slewToward(zW.uSlew, (float)zTarget, PELTIER_SLEW_PER_S * dtFast);

  int u = (int)lroundf(zW.uSlew);
  if (u < 0)      { direction = COOL_DIR;  pwmValue = -u; }
  else if (u > 0) { direction = -COOL_DIR; pwmValue = u;  }
  else            { direction = 0;         pwmValue = 0;  }
}

// ================= Safety =================

float conditionWater(float raw) {
  if (isnan(raw) || raw < SENSOR_MIN_C || raw > SENSOR_MAX_C) { spikeCount = 0; return NAN; }
  if (isnan(lastGoodWater)) { lastGoodWater = raw; return raw; }
  if (fabsf(raw - lastGoodWater) > SENSOR_SPIKE_C) {
    if (++spikeCount < SENSOR_SPIKE_N) return lastGoodWater;
  }
  spikeCount = 0;
  lastGoodWater = raw;
  return raw;
}

void latchFault(const char* why, float pv) {
  if (faultLatched) return;
  faultLatched = true;
  snprintf(faultText, sizeof(faultText), "%s (%.1f C)", why, pv);
  direction = 0; pwmValue = 0;
  zW.uSlew = 0.0f; zTarget = 0;
  applyMotor();
  say("# *** FAULT: %s - Peltier OFF, driver disabled. Fix it, then type 'clear' ***\n", faultText);
}

void tripWrongDirection(const char* what, float pv) {
  unsigned long now = millis();
  if (wdTrips > 0 && now - wdLastTrip > 3600000UL) wdTrips = 0;
  wdTrips++;
  wdLastTrip = now;
  wdMode = 0;
  if (wdTrips >= WRONGDIR_MAX_TRIPS) {
    latchFault("temperature keeps moving the wrong way - check hot-side cooling, polarity, water flow", pv);
    return;
  }
  blockedUntil = now + WRONGDIR_COOLDOWN_MS;
  if (blockedUntil == 0) blockedUntil = 1;
  say("# GUARD: %s (%.1f C) - Peltier off for %lu min (trip %d of %d). Check hot-side fan/heatsink and polarity\n",
      what, pv, (unsigned long)(WRONGDIR_COOLDOWN_MS / 60000UL), wdTrips, WRONGDIR_MAX_TRIPS);
}

// 'pv' is the filtered water temperature.
void safetyCheck(float pv) {
  unsigned long now = millis();
  if (now == 0) now = 1;

  if (isnan(pv)) {
    goodSince = 0;
    wdMode = 0;
    if (badSince == 0) badSince = now;
    if (!sensorFault && now - badSince >= SENSOR_FAULT_MS) {
      sensorFault = true;
      say("# GUARD: water sensor (NTC3) not reading - Peltier OFF until it returns. Check the wiring\n");
    }
    return;
  }
  badSince = 0;
  if (sensorFault) {
    if (goodSince == 0) goodSince = now;
    if (now - goodSince >= SENSOR_OK_MS) {
      sensorFault = false;
      goodSince = 0;
      softResetController();
      say("# GUARD: water sensor is back - resuming\n");
    }
  }

  if (pv >= WATER_HARD_MAX_C) { latchFault("water over-temperature", pv); return; }

  if (coolAllowed && pv <= WATER_MIN_C) {
    coolAllowed = false;
    say("# GUARD: water %.1f C is near freezing - cooling blocked until %.1f C\n", pv, WATER_MIN_CLEAR_C);
  } else if (!coolAllowed && pv >= WATER_MIN_CLEAR_C) {
    coolAllowed = true;
    say("# GUARD: water %.1f C - cooling allowed again\n", pv);
  }
  if (heatAllowed && pv >= WATER_MAX_C) {
    heatAllowed = false;
    say("# GUARD: water %.1f C is very warm - heating blocked until %.1f C\n", pv, WATER_MAX_CLEAR_C);
  } else if (!heatAllowed && pv <= WATER_MAX_CLEAR_C) {
    heatAllowed = true;
    say("# GUARD: water %.1f C - heating allowed again\n", pv);
  }

  // Wrong-direction guard. v1 tracked the running peak, so one noise spike plus the normal slow rise looked like a
  // 2 C reversal (false trips 10 min after each setpoint step). Now it compares the filtered value at the end of the
  // window with the value at the start, and only trips on a real drift against the drive.
  int signedU = 0;
  if (direction != 0 && pwmValue > 0) signedU = (direction == COOL_DIR) ? -pwmValue : pwmValue;
  int mode = 0;
  if (signedU <= -(int)pMinCool) mode = -1;
  else if (signedU >= (int)pMinHeat) mode = 1;

  if (mode == 0 || mode != wdMode || isnan(wdRef)) {
    wdMode = mode; wdStart = now; wdRef = pv;
  } else {
    unsigned long win = (mode == -1) ? WRONGDIR_COOL_MS : WRONGDIR_HEAT_MS;
    if (now - wdStart >= win) {
      float drift = pv - wdRef;                       // positive = warmer
      if (mode == -1 && drift >= pWrongDirC)       tripWrongDirection("water is WARMING while the Peltier is cooling", pv);
      else if (mode == 1 && -drift >= pWrongDirC)  tripWrongDirection("water is COOLING while the Peltier is heating", pv);
      else { wdStart = now; wdRef = pv; }
    }
  }
}

void controlOutputs(float pv) {
  unsigned long now = millis();
  lastPv = pv;
  bool inhibit = faultLatched || sensorFault || blocked(now);

  if (inhibit) {
    direction = 0; pwmValue = 0;
    zW.uSlew = 0.0f; zTarget = 0;
    zW.pidIntegral = 0.0f; zW.pv_filt = NAN;
    pidLastCtl = 0; pidLastFast = 0;
  } else if (autoMode) {
    runPID(pv);
  } else if (direction != 0 && now - manualStartMs >= MANUAL_MAX_MS) {
    direction = 0; pwmValue = 0;
    say("# Manual run stopped after %lu min (safety limit)\n", (unsigned long)(MANUAL_MAX_MS / 60000UL));
  }
  enforceLimits();
  applyMotor();
}

// ================= Temperature =================

// Returns C, or NAN if the sensor wire is loose, shorted, or open-circuit (ADC saturated).
float readTempC(int pin, float seriesR, float nominalR, float beta) {
  long sum = 0;
  for (int i = 0; i < 32; i++) sum += analogReadMilliVolts(pin);
  float v = sum / 32.0;
  if (v <= 5 || v >= ADC_SAT_MV) return NAN;       // v1 only checked VCC-5 mV, which the ADC never reaches

  float r  = seriesR * v / (VCC_MV - v);
  float tK = 1.0 / (1.0 / (NOMINAL_T + 273.15) + log(r / nominalR) / beta);
  float c = tK - 273.15;
  if (c < -30.0f || c > 130.0f) return NAN;
  return c;
}

// ================= Real time =================

void getTimestamp(char *buf, size_t len) {
  if (!timeValid) { snprintf(buf, len, "NO_TIME"); return; }
  struct timeval tv;
  gettimeofday(&tv, NULL);
  struct tm tm;
  localtime_r(&tv.tv_sec, &tm);
  snprintf(buf, len, "%04d-%02d-%02d %02d:%02d:%02d.%03ld",
           tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday,
           tm.tm_hour, tm.tm_min, tm.tm_sec, (long)(tv.tv_usec / 1000));
}

bool setTimeFromString(const String &s) {
  int Y, M, D, h, m, sec;
  if (sscanf(s.c_str(), "%d-%d-%d %d:%d:%d", &Y, &M, &D, &h, &m, &sec) != 6) return false;
  if (Y < 2024 || Y > 2099 || M < 1 || M > 12 || D < 1 || D > 31 ||
      h < 0 || h > 23 || m < 0 || m > 59 || sec < 0 || sec > 59) return false;
  struct tm tm = {};
  tm.tm_year = Y - 1900; tm.tm_mon = M - 1; tm.tm_mday = D;
  tm.tm_hour = h;        tm.tm_min = m;     tm.tm_sec  = sec;
  tm.tm_isdst = 0;
  time_t t = mktime(&tm);
  struct timeval tv = { t, 0 };
  settimeofday(&tv, NULL);
  timeValid = true;
  return true;
}

void printStatus(Print &p) {
  char ts[32];
  getTimestamp(ts, sizeof(ts));
  bool wifiUp = (WiFi.status() == WL_CONNECTED);
  char hold[40] = "reaching target";
  if (!autoMode) snprintf(hold, sizeof(hold), "PID off");
  else if (profileDone) snprintf(hold, sizeof(hold), "profile complete");
  else if (setpointReached) {
    unsigned long el = millis() - holdStart;
    long left = (el >= STEP_INTERVAL_MS) ? 0 : (long)((STEP_INTERVAL_MS - el) / 60000UL);
    if (setpoint >= MAX_SETPOINT) snprintf(hold, sizeof(hold), "final hold, %ld min left", left);
    else snprintf(hold, sizeof(hold), "next step in %ld min", left);
  }
  p.printf("# Status: fw %s | time %s | %s | phase %s | SP %.1fC (%s) | guard %s | SD %s | WiFi %s (%d dBm) | MQTT %s\n",
           FW_VERSION, ts, autoMode ? "AUTO" : "MANUAL", phaseName(), setpoint, hold,
           faultLatched ? faultText : (sensorFault ? "SENSOR LOST" : "ok"),
           sdOk ? currentName : "NOT logging",
           wifiUp ? WiFi.localIP().toString().c_str() : "not connected",
           wifiUp ? (int)WiFi.RSSI() : 0, mqttUp ? "up" : "down");
}

// ================= Parameters (runtime tuning, saved in flash) =================

static Param* findParam(const String& name) {
  for (int i = 0; i < NPARAMS; i++) if (name == PARAMS[i].name) return &PARAMS[i];
  return nullptr;
}

void saveParams() {
  char key[20];
  for (int i = 0; i < NPARAMS; i++) { snprintf(key, sizeof(key), "p_%s", PARAMS[i].name); prefs.putFloat(key, *PARAMS[i].ptr); }
}

void loadParams() {
  char key[20];
  for (int i = 0; i < NPARAMS; i++) {
    snprintf(key, sizeof(key), "p_%s", PARAMS[i].name);
    if (prefs.isKey(key)) *PARAMS[i].ptr = clampf(prefs.getFloat(key, *PARAMS[i].ptr), PARAMS[i].lo, PARAMS[i].hi);
  }
}

// ---- Credentials in flash (set over USB with 'cfg'; never compiled in, never sent over MQTT) ----
static void getCfg(const char* key, char* dst, size_t n) { if (prefs.isKey(key)) prefs.getString(key, dst, n); }

void loadConfig() {
  getCfg("c_ssid", cfgSsid, sizeof(cfgSsid));   getCfg("c_pass", cfgPass, sizeof(cfgPass));
  getCfg("c_host", cfgHost, sizeof(cfgHost));   getCfg("c_user", cfgUser, sizeof(cfgUser));
  getCfg("c_mqp",  cfgMqPass, sizeof(cfgMqPass)); getCfg("c_base", cfgBase, sizeof(cfgBase));
  if (prefs.isKey("c_port")) cfgPort = prefs.getUShort("c_port", 8883);
}

static int splitSemi(const String& s, String* out, int maxParts) {
  int n = 0, from = 0;
  while (n < maxParts) {
    int i = s.indexOf(';', from);
    if (i < 0) { out[n++] = s.substring(from); break; }
    out[n++] = s.substring(from, i); from = i + 1;
  }
  return n;
}

void clearSavedParams() {
  char key[20];
  for (int i = 0; i < NPARAMS; i++) { snprintf(key, sizeof(key), "p_%s", PARAMS[i].name); prefs.remove(key); }
}

// ================= Resume after reboot =================

const char* resetReasonText() {
  switch (esp_reset_reason()) {
    case ESP_RST_POWERON:  return "power-on";
    case ESP_RST_SW:       return "software reset";
    case ESP_RST_PANIC:    return "CRASH (panic)";
    case ESP_RST_INT_WDT:
    case ESP_RST_TASK_WDT:
    case ESP_RST_WDT:      return "WATCHDOG (program hung)";
    case ESP_RST_BROWNOUT: return "BROWNOUT (supply dipped - use a stronger 5V supply)";
    default:               return "other";
  }
}

void saveState() {
  prefs.putBool("run", autoMode);
  prefs.putFloat("sp", setpoint);
  prefs.putBool("reach", setpointReached);
  prefs.putUInt("holdEl", setpointReached ? (uint32_t)(millis() - holdStart) : 0);
  prefs.putBool("floor", floorSearch);
  prefs.putBool("done", profileDone);
  prefs.putBool("unr", reachTimedOut);
  prefs.putUInt("ts", timeValid ? (uint32_t)time(nullptr) : 0);
}

void maybeSaveState() {
  static bool pAuto = false, pReach = false, pDone = false;
  static float pSp = -1.0f;
  static unsigned long lastSave = 0;
  if (autoMode != pAuto || setpoint != pSp || setpointReached != pReach || profileDone != pDone ||
      millis() - lastSave >= STATE_SAVE_MS) {
    pAuto = autoMode; pSp = setpoint; pReach = setpointReached; pDone = profileDone;
    lastSave = millis();
    saveState();
  }
}

bool loadState() {
  if (!RESUME_AFTER_REBOOT || !prefs.isKey("run")) return false;

  if (!prefs.getBool("run", false)) {
    autoMode = false;
    say("# Last session was stopped - motor stays off. Type 'a' to start the profile\n");
    return true;
  }
  uint32_t ts = prefs.getUInt("ts", 0);
  if (ts > 0 && timeValid) {
    long gap = (long)time(nullptr) - (long)ts;
    if (gap > (long)RESUME_MAX_GAP_S) {
      say("# Saved profile is %ld min old - starting fresh\n", gap / 60);
      return false;
    }
  }
  setpoint        = constrain(prefs.getFloat("sp", START_SETPOINT), SP_MIN_C, MAX_SETPOINT);
  setpointReached = prefs.getBool("reach", false);
  floorSearch     = prefs.getBool("floor", true);
  profileDone     = prefs.getBool("done", false);
  reachTimedOut   = prefs.getBool("unr", false);
  uint32_t holdEl = prefs.getUInt("holdEl", 0);
  holdStart       = millis() - holdEl;
  reachStart      = millis();
  zW.sp_ramp      = setpoint;
  autoMode        = true;
  if (profileDone) {
    say("# RESUMED: profile was already complete - Peltier stays off. Type 'a' to run again\n");
  } else {
    say("# RESUMED after reboot: target %.1f C, %s", setpoint, setpointReached ? "hold " : "still reaching it");
    if (setpointReached) say("%lu min already done", (unsigned long)(holdEl / 60000UL));
    say(". Type 'a' to restart from %.1f C instead\n", START_SETPOINT);
  }
  return true;
}

// ================= WiFi =================

void onWiFiConnected() {
  wifiWasConnected = true;
  say("# WiFi connected. PC program -> %s port %u  (or %s.local)\n",
      WiFi.localIP().toString().c_str(), TCP_PORT, MDNS_NAME);
  MDNS.end();
  if (MDNS.begin(MDNS_NAME)) MDNS.addService("telnet", "tcp", TCP_PORT);
}

void startWiFi() {
  if (strlen(cfgSsid) == 0) {
    say("# WiFi not configured - USB only. Use 'cfg wifi <ssid>;<pass>' then 'reboot', or set the clock with 'time YYYY-MM-DD HH:MM:SS'\n");
    return;
  }
  wifiEnabled = true;
  WiFi.persistent(false);
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.setAutoReconnect(true);
  WiFi.begin(cfgSsid, cfgPass);
  configTzTime(TZ_INFO, NTP_SERVER1, NTP_SERVER2);
  server.begin();
  server.setNoDelay(true);

  Serial.printf("# Connecting to WiFi '%s'", cfgSsid);
  unsigned long t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < WIFI_TIMEOUT_MS) {
    delay(250);
    Serial.print(".");
  }
  Serial.println();

  if (WiFi.status() == WL_CONNECTED) {
    onWiFiConnected();
    struct tm tm;
    if (getLocalTime(&tm, 10000)) {
      timeValid = true;
      char ts[32];
      getTimestamp(ts, sizeof(ts));
      say("# Time synced from internet: %s\n", ts);
    } else {
      say("# Time server not answering yet - will keep trying\n");
    }
  } else {
    say("# WiFi not connected yet - will keep trying (SD logging still works)\n");
  }
}

void wifiKeepAlive() {
  static unsigned long downSince = 0, lastKick = 0;
  if (WiFi.status() == WL_CONNECTED) { downSince = 0; return; }
  unsigned long now = millis();
  if (downSince == 0) { downSince = now ? now : 1; lastKick = now; return; }
  if (now - lastKick < WIFI_KICK_MS) return;
  lastKick = now;
  if (now - downSince >= WIFI_RESET_MS) {
    Serial.println("# WiFi down for 5 min - restarting the radio");
    WiFi.disconnect(true);
    delay(100);
    WiFi.mode(WIFI_STA);
    WiFi.setSleep(false);
    WiFi.setAutoReconnect(true);
    WiFi.begin(cfgSsid, cfgPass);
    downSince = now;
  } else {
    WiFi.reconnect();
  }
}

void handleWiFi() {
  if (!wifiEnabled) return;

  wifiKeepAlive();

  bool connected = (WiFi.status() == WL_CONNECTED);
  if (connected && !wifiWasConnected) onWiFiConnected();
  if (!connected && wifiWasConnected) {
    wifiWasConnected = false;
    Serial.println("# WiFi lost - SD logging continues, reconnecting...");
  }

  if (!timeValid && time(nullptr) > 1700000000) {
    timeValid = true;
    say("# Time synced from internet\n");
  }

  if (server.hasClient()) {
    WiFiClient newClient = server.accept();
    if (clientOk()) {
      client.println("# Another PC connected - closing this connection");
      client.stop();
    }
    client = newClient;
    client.setNoDelay(true);
    tcpBuf = "";
    client.println("# Connected to ESP32 motor + temperature logger");
    printStatus(client);
    printHelp(client);
    Serial.printf("# PC connected from %s\n", client.remoteIP().toString().c_str());
  }

  while (clientOk() && client.available()) {
    char ch = client.read();
    if (ch == '\n') {
      replyTo = &client;
      handleCommand(tcpBuf);
      replyTo = &Serial;
      tcpBuf = "";
    } else if (ch != '\r' && tcpBuf.length() < 100) {
      tcpBuf += ch;
    }
  }

  static unsigned long lastStatus = 0;
  if (STATUS_MS > 0 && clientOk() && millis() - lastStatus >= STATUS_MS) {
    lastStatus = millis();
    printStatus(client);
  }
}

// ================= Telemetry (one JSON snapshot per second) =================

static void sendTelemetry(float pwmPct, float t1, float t2, float t3, float t3f, float p, float i, float d) {
  if (!CLOUD_ON || !mqttUp) return;
  char ts[32]; getTimestamp(ts, sizeof(ts));
  char a[12] = "null", b[12] = "null", c[12] = "null", f[12] = "null";
  if (!isnan(t1))  snprintf(a, sizeof(a), "%.1f", t1);
  if (!isnan(t2))  snprintf(b, sizeof(b), "%.1f", t2);
  if (!isnan(t3))  snprintf(c, sizeof(c), "%.1f", t3);
  if (!isnan(t3f)) snprintf(f, sizeof(f), "%.2f", t3f);
  unsigned long holdMin = (autoMode && setpointReached && !profileDone) ? (millis() - holdStart) / 60000UL : 0;
  Msg m; m.kind = KIND_TELE;
  snprintf(m.text, sizeof(m.text),
    "{\"time\":\"%s\",\"pwm\":%.1f,\"dir\":%d,\"t1\":%s,\"t2\":%s,\"t3\":%s,\"t3f\":%s,\"sp\":%.1f,"
    "\"ph\":\"%s\",\"p\":%.1f,\"i\":%.1f,\"d\":%.1f,\"flt\":%d,\"mode\":\"%s\",\"sd\":%d,\"rssi\":%d,\"up\":%lu,"
    "\"fw\":\"%s\",\"tag\":\"%s\",\"kp\":%.2f,\"ki\":%.3f,\"kd\":%.1f,\"hold\":%lu,\"dm\":%lu}",
    ts, pwmPct, direction, a, b, c, f, setpoint, phaseName(), p, i, d,
    (faultLatched || sensorFault) ? 1 : 0, autoMode ? "AUTO" : "MANUAL", sdOk ? 1 : 0,
    WiFi.status() == WL_CONNECTED ? (int)WiFi.RSSI() : 0, millis() / 1000, FW_VERSION, expTag,
    Kp, Ki, Kd, holdMin, deadmanMs / 60000UL);
  xQueueSend(qOut, &m, 0);
}

// ================= SD setup / reconnect / recovery =================

void makeName(int n, char *buf, size_t len) {
  snprintf(buf, len, "/log_%03d.csv", n);
}

bool openNewLog() {
  for (int n = 1; n < 1000; n++) {
    makeName(n, currentName, sizeof(currentName));
    if (!SD.exists(currentName)) break;
  }
  logFile = SD.open(currentName, FILE_WRITE);
  if (!logFile) {
    say("# Could not create log file\n");
    memset(currentName, 0, sizeof(currentName));
    sdOk = false;
    return false;
  }
  logFile.println(CSV_HEADER);
  logFile.flush();
  sdOk = true;
  say("# Logging to SD file %s\n", currentName);
  return true;
}

void initSD() {
  SPI.begin(SD_SCK_PIN, SD_MISO_PIN, SD_MOSI_PIN, SD_CS_PIN);
  if (!SD.begin(SD_CS_PIN, SPI)) {
    sdMounted = false;
    say("# SD card init FAILED - no SD logging (retrying every 30 s). Fix card/wiring, or type 'sd'\n");
    return;
  }
  sdMounted = true;
  openNewLog();
}

void reconnectSD() {
  say("# Reconnecting SD card...\n");
  if (sdOk) logFile.close();
  sdOk = false;
  sdMounted = false;
  memset(currentName, 0, sizeof(currentName));
  SD.end();
  delay(100);
  initSD();
}

void sdAutoRecover() {
  static unsigned long lastTry = 0;
  if (sdOk || millis() - lastTry < SD_RETRY_MS) return;
  lastTry = millis();
  if (logFile) logFile.close();
  SD.end();
  delay(20);
  if (SD.begin(SD_CS_PIN, SPI)) {
    sdMounted = true;
    if (currentName[0] != 0) logFile = SD.open(currentName, FILE_APPEND);
    if (logFile) {
      sdOk = true;
      say("# SD card recovered - logging continues in %s\n", currentName);
      return;
    }
  }
  SD.end();
  initSD();
}

// ================= SD card viewing commands =================

bool sdReady() {
  if (!sdMounted) { reply("# No SD card available (type 'sd' to retry)\n"); return false; }
  return true;
}

void listFiles() {
  if (!sdReady()) return;
  File root = SD.open("/");
  if (!root) { reply("# Could not open card\n"); return; }
  reply("# ---- Files on SD card ----\n");
  int count = 0;
  File f = root.openNextFile();
  while (f) {
    FEED_WDT();
    if (!f.isDirectory()) {
      bool isCurrent = (strcmp(f.name(), currentName + 1) == 0);
      reply("#   %-14s %10u bytes%s\n", f.name(), (unsigned)f.size(),
            isCurrent ? "   <- current" : "");
      count++;
    }
    f.close();
    f = root.openNextFile();
  }
  root.close();
  reply("# ---- %d file(s) ----\n", count);
}

void dumpFile(int n) {
  if (!sdReady()) return;
  char name[16];
  makeName(n, name, sizeof(name));
  if (!SD.exists(name)) { reply("# %s not found (try 'list')\n", name); return; }
  if (sdOk && strcmp(name, currentName) == 0) logFile.flush();

  File f = SD.open(name, FILE_READ);
  if (!f) { reply("# Could not open %s\n", name); return; }
  reply("# ---- BEGIN %s (%u bytes) ----\n", name, (unsigned)f.size());
  uint8_t buf[512];
  while (f.available()) {
    FEED_WDT();
    size_t got = f.read(buf, sizeof(buf));
    replyTo->write(buf, got);
  }
  f.close();
  reply("# ---- END %s ----\n", name);
}

void showLast() {
  if (!sdReady()) return;
  if (!sdOk) { reply("# No active log file\n"); return; }
  logFile.flush();
  File f = SD.open(currentName, FILE_READ);
  if (!f) { reply("# Could not open current log\n"); return; }

  size_t size  = f.size();
  size_t start = (size > 1500) ? size - 1500 : 0;
  f.seek(start);
  if (start > 0) {
    while (f.available() && f.read() != '\n') {}
  }
  reply("# ---- Last lines of %s ----\n", currentName);
  reply("# %s\n", CSV_HEADER);
  uint8_t buf[256];
  while (f.available()) {
    size_t got = f.read(buf, sizeof(buf));
    replyTo->write(buf, got);
  }
  f.close();
  reply("# ---- END ----\n");
}

void deleteFile(int n) {
  if (!sdReady()) return;
  char name[16];
  makeName(n, name, sizeof(name));
  if (strcmp(name, currentName) == 0) {
    reply("# Cannot delete the file currently being written\n");
    return;
  }
  if (!SD.exists(name)) { reply("# %s not found\n", name); return; }
  if (SD.remove(name)) reply("# Deleted %s\n", name);
  else                 reply("# Could not delete %s\n", name);
}

// ================= OTA (pull from a whitelisted URL) =================

void doOta(const String& url) {
  if (!url.startsWith(OTA_URL_PREFIX)) { reply("# OTA refused: URL must start with the configured prefix\n"); return; }
  if (WiFi.status() != WL_CONNECTED)   { reply("# OTA refused: WiFi is down\n"); return; }
  reply("# OTA: Peltier off, downloading. The board reboots into the new firmware if it succeeds\n");
  stopMotor();
  saveState();
  WiFiClientSecure net;
  net.setInsecure();
  httpUpdate.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);
  httpUpdate.rebootOnUpdate(true);
#if USE_WATCHDOG
  disableLoopWDT();                      // the download blocks for longer than the watchdog allows
#endif
  t_httpUpdate_return r = httpUpdate.update(net, url);
#if USE_WATCHDOG
  enableLoopWDT();
#endif
  if (r == HTTP_UPDATE_FAILED)          reply("# OTA failed: %s\n", httpUpdate.getLastErrorString().c_str());
  else if (r == HTTP_UPDATE_NO_UPDATES) reply("# OTA: no update available\n");
}

// ================= Command handling =================

void printPid() {
  reply("# PID: SP=%.2f  Kp=%.4f  Ki=%.4f  Kd=%.4f  (cool %d-%d, heat %d-%d)\n",
        setpoint, Kp, Ki, Kd, (int)pMinCool, (int)capCool(), (int)pMinHeat, (int)capHeat());
}

void printGuards() {
  reply("# Guards: fault=%s | sensor=%s | cooling %s | heating %s | cooldown %s | profile %s\n",
        faultLatched ? faultText : "none", sensorFault ? "LOST" : "ok",
        coolAllowed ? "ok" : "BLOCKED", heatAllowed ? "ok" : "BLOCKED",
        blocked(millis()) ? "ACTIVE" : "no",
        profileDone ? "complete" : (autoMode ? "running" : "stopped"));
}

void printParams() {
  char buf[440]; size_t n = 0;
  n += snprintf(buf + n, sizeof(buf) - n, "# params:");
  for (int i = 0; i < NPARAMS && n < sizeof(buf) - 24; i++)
    n += snprintf(buf + n, sizeof(buf) - n, " %s=%g", PARAMS[i].name, (double)*PARAMS[i].ptr);
  replyTo->println(buf);
}

static bool allDigits(const String &s) {
  if (s.length() == 0) return false;
  for (unsigned i = 0; i < s.length(); i++) if (s[i] < '0' || s[i] > '9') return false;
  return true;
}

static bool setGain(float &g, const String &arg, float maxv) {
  float v = arg.toFloat();
  if (isnan(v) || v < 0.0f || v > maxv) { reply("# Value must be 0..%.0f\n", maxv); return false; }
  g = v;
  return true;
}

// Commands that may arrive over MQTT. Manual drive, SD access and setting the clock are USB/PC only.
static bool remoteAllowed(const String& cmd) {
  int sp = cmd.indexOf(' ');
  String head = sp < 0 ? cmd : cmd.substring(0, sp);
  static const char* OK[] = {"status", "guard", "pid", "params", "get", "set", "save", "defaults", "sp", "kp", "ki", "kd",
                             "a", "s", "clear", "hb", "deadman", "tag", "ota", "reboot", "ver", "ip"};
  for (const char* o : OK) if (head == o) return true;
  return cmd == "time";
}

void handleCommand(String cmd) {
  cmd.trim();
  String raw = cmd;                      // keeps the case, needed for URLs and tags
  cmd.toLowerCase();
  if (cmd.length() == 0) return;

  if (replyTo == &mqttPrint && !remoteAllowed(cmd)) {
    reply("# Not allowed over MQTT: %s\n", cmd.c_str());
    return;
  }

  // --- Credentials: USB serial only ---
  if (cmd == "cfg" || cmd.startsWith("cfg ")) {
    if (replyTo != &Serial) { reply("# cfg works over USB only\n"); return; }
    String a = raw.length() > 4 ? raw.substring(4) : String("");
    a.trim();
    if (a.startsWith("wifi ")) {
      String p[2]; int n = splitSemi(a.substring(5), p, 2);
      p[0].trim();
      if (n < 1 || p[0].length() == 0 || p[0].length() >= sizeof(cfgSsid) || p[1].length() >= sizeof(cfgPass)) {
        reply("# Use: cfg wifi <ssid>;<password>\n"); return;
      }
      prefs.putString("c_ssid", p[0]); prefs.putString("c_pass", p[1]);
      p[0].toCharArray(cfgSsid, sizeof(cfgSsid)); p[1].toCharArray(cfgPass, sizeof(cfgPass));
      reply("# WiFi saved. Type 'reboot' to connect\n"); return;
    }
    if (a.startsWith("mqtt ")) {
      String p[5]; int n = splitSemi(a.substring(5), p, 5);
      int port = p[1].toInt();
      for (int i = 0; i < 5; i++) p[i].trim();
      if (n < 5 || p[0].length() == 0 || p[0].length() >= sizeof(cfgHost) || port < 1 || port > 65535 ||
          p[2].length() >= sizeof(cfgUser) || p[3].length() >= sizeof(cfgMqPass) ||
          p[4].length() == 0 || p[4].length() >= sizeof(cfgBase)) {
        reply("# Use: cfg mqtt <host>;<port>;<user>;<password>;<topic base>\n"); return;
      }
      prefs.putString("c_host", p[0]); prefs.putUShort("c_port", (uint16_t)port); prefs.putString("c_user", p[2]);
      prefs.putString("c_mqp", p[3]);  prefs.putString("c_base", p[4]);
      p[0].toCharArray(cfgHost, sizeof(cfgHost)); cfgPort = (uint16_t)port; p[2].toCharArray(cfgUser, sizeof(cfgUser));
      p[3].toCharArray(cfgMqPass, sizeof(cfgMqPass)); p[4].toCharArray(cfgBase, sizeof(cfgBase));
      reply("# MQTT settings saved. Type 'reboot' to connect\n"); return;
    }
    if (a == "show") {
      reply("# cfg: wifi ssid '%s' (password %s) | mqtt %s:%u user '%s' (password %s) base '%s'\n",
            cfgSsid, cfgPass[0] ? "set" : "empty", cfgHost, (unsigned)cfgPort, cfgUser, cfgMqPass[0] ? "set" : "empty", cfgBase);
      return;
    }
    if (a == "clear") {
      const char* keys[] = {"c_ssid", "c_pass", "c_host", "c_port", "c_user", "c_mqp", "c_base"};
      for (const char* k : keys) prefs.remove(k);
      reply("# Saved WiFi and MQTT settings erased. Type 'reboot'\n"); return;
    }
    reply("# cfg wifi <ssid>;<pass> | cfg mqtt <host>;<port>;<user>;<pass>;<base> | cfg show | cfg clear\n");
    return;
  }

  // --- Time ---
  if (cmd == "time") {
    char ts[32];
    getTimestamp(ts, sizeof(ts));
    reply("# Current time: %s\n", ts);
    return;
  }
  if (cmd.startsWith("time ")) {
    if (setTimeFromString(cmd.substring(5))) {
      char ts[32];
      getTimestamp(ts, sizeof(ts));
      reply("# Time set: %s\n", ts);
    } else {
      reply("# Format: time 2026-10-24 22:30:00\n");
    }
    return;
  }

  // --- Network info / status ---
  if (cmd == "ip") {
    if (!wifiEnabled) reply("# WiFi not configured\n");
    else if (WiFi.status() == WL_CONNECTED)
      reply("# WiFi OK: %s port %u, signal %d dBm, PC %s, MQTT %s\n",
            WiFi.localIP().toString().c_str(), TCP_PORT, WiFi.RSSI(),
            clientOk() ? "connected" : "not connected", mqttUp ? "up" : "down");
    else reply("# WiFi not connected (retrying)\n");
    return;
  }
  if (cmd == "status") { printStatus(*replyTo); return; }
  if (cmd == "guard")  { printGuards(); return; }
  if (cmd == "ver")    { reply("# fw %s | last reset: %s\n", FW_VERSION, resetReasonText()); return; }
  if (cmd == "hb")     { reply("# hb ok\n"); return; }

  // --- Remote supervision ---
  if (cmd.startsWith("deadman")) {
    long m = cmd.substring(7).toInt();
    if (m < 0 || m > 240) { reply("# Use: deadman <0-240 minutes>, 0 = off\n"); return; }
    deadmanMs = (unsigned long)m * 60000UL;
    lastRemote = millis();
    reply("# Dead-man timer %s\n", m == 0 ? "off" : "armed: Peltier stops if no MQTT command arrives in time");
    return;
  }
  if (cmd.startsWith("tag")) {
    String t = raw.length() > 4 ? raw.substring(4) : String("");
    t.trim();
    size_t n = 0;
    for (unsigned i = 0; i < t.length() && n < sizeof(expTag) - 1; i++) {
      char c = t[i];
      if (isalnum((unsigned char)c) || c == '-' || c == '_') expTag[n++] = c;
    }
    expTag[n] = 0;
    reply("# Tag: %s\n", expTag);
    return;
  }
  if (cmd.startsWith("ota ")) { doOta(raw.substring(4)); return; }
  if (cmd == "reboot") {
    reply("# Rebooting\n");
    stopMotor(); saveState();
    delay(300);
    ESP.restart();
    return;
  }

  // --- Parameters ---
  if (cmd == "params") { printParams(); return; }
  if (cmd.startsWith("get ")) {
    Param* p = findParam(cmd.substring(4));
    if (!p) { reply("# Unknown parameter\n"); return; }
    reply("# %s=%g (allowed %g..%g)\n", p->name, (double)*p->ptr, (double)p->lo, (double)p->hi);
    return;
  }
  if (cmd.startsWith("set ")) {
    String rest = cmd.substring(4);
    int sp = rest.indexOf(' ');
    if (sp < 0) { reply("# Use: set <name> <value>\n"); return; }
    Param* p = findParam(rest.substring(0, sp));
    if (!p) { reply("# Unknown parameter (see 'params')\n"); return; }
    float v = rest.substring(sp + 1).toFloat();
    if (isnan(v) || v < p->lo || v > p->hi) { reply("# %s must be %g..%g\n", p->name, (double)p->lo, (double)p->hi); return; }
    *p->ptr = v;
    reply("# %s=%g\n", p->name, (double)v);
    return;
  }
  if (cmd == "save")     { saveParams(); reply("# Parameters saved to flash (loaded at every boot)\n"); return; }
  if (cmd == "defaults") { clearSavedParams(); reply("# Saved parameters erased. Reboot to return to the built-in defaults\n"); return; }

  // --- SD card ---
  if (cmd == "sd")   { reconnectSD(); return; }
  if (cmd == "list") { listFiles();   return; }
  if (cmd == "last") { showLast();    return; }
  if (cmd.startsWith("dump")) {
    int n = cmd.substring(4).toInt();
    if (n < 1 || n > 999) { reply("# Use: dump <1-999>\n"); return; }
    dumpFile(n);
    return;
  }
  if (cmd.startsWith("del")) {
    int n = cmd.substring(3).toInt();
    if (n < 1 || n > 999) { reply("# Use: del <1-999>\n"); return; }
    deleteFile(n);
    return;
  }
  if (cmd == "q") {
    liveOutput = !liveOutput;
    reply("# Live data %s (SD logging continues)\n", liveOutput ? "ON" : "OFF");
    return;
  }

  // --- Faults ---
  if (cmd == "clear") {
    faultLatched = false;
    faultText[0] = 0;
    blockedUntil = 0;
    wdTrips = 0; wdMode = 0;
    softResetController();
    reply("# Faults cleared (they come straight back if the cause is still there)\n");
    return;
  }

  // --- PID tuning / setpoint ---
  if (cmd == "pid") { printPid(); return; }
  if (cmd.startsWith("sp ")) {
    float v = cmd.substring(3).toFloat();
    if (isnan(v) || v < SP_MIN_C || v > MAX_SETPOINT) { reply("# Setpoint must be %.0f..%.0f C\n", SP_MIN_C, MAX_SETPOINT); return; }
    if (faultLatched) { reply("# Fault active - type 'clear' first\n"); return; }
    if (!autoMode) { autoMode = true; resetPID(); }
    setpoint = v;
    setpointReached = false;
    reachTimedOut = false;
    profileDone = false;
    holdStart = 0;
    reachStart = millis();
    floorSearch = false;
    zW.sp_ramp = v;
    printPid();
    return;
  }
  if (cmd.startsWith("kp ")) { if (setGain(Kp, cmd.substring(3), 500.0f))  printPid(); return; }
  if (cmd.startsWith("ki ")) { if (setGain(Ki, cmd.substring(3), 5.0f))    printPid(); return; }
  if (cmd.startsWith("kd ")) { if (setGain(Kd, cmd.substring(3), 1000.0f)) printPid(); return; }

  // --- Profile start / stop ---
  if (cmd == "s") { stopMotor(); reply("# Motor stopped, PID off\n"); return; }
  if (cmd == "a") {
    if (faultLatched) { reply("# Fault active - type 'clear' first\n"); return; }
    autoMode = true;
    resetPID();
    reply("# Profile started fresh: first target %.1f C, then +%.1f C every hour\n", START_SETPOINT, SETPOINT_STEP);
    return;
  }

  // --- Manual motor: f<0-255> / r<0-255> ---
  char c = cmd[0];
  if ((c == 'f' || c == 'r') && allDigits(cmd.substring(1))) {
    if (faultLatched) { reply("# Fault active - type 'clear' first\n"); return; }
    int newDir = (c == 'f') ? 1 : -1;
    int cap = (newDir == COOL_DIR) ? (int)capCool() : (int)capHeat();
    int val = constrain(cmd.substring(1).toInt(), 0, 255);
    if (val > cap) { val = cap; reply("# Limited to the safety cap of %d\n", cap); }

    autoMode = false;
    if (newDir != direction && direction != 0) {
      pwmValue = 0;
      applyMotor();
      delay(300);
    }
    direction = (val > 0) ? newDir : 0;
    pwmValue  = val;
    manualStartMs = millis();
    reply("# Manual: %s at PWM %d (PID off, stops by itself after %lu min)\n",
          newDir == COOL_DIR ? "COOLING" : "HEATING", val, (unsigned long)(MANUAL_MAX_MS / 60000UL));
    return;
  }
  reply("# Unknown command\n");
}

void handleSerial() {
  if (!Serial.available()) return;
  String cmd = Serial.readStringUntil('\n');
  replyTo = &Serial;
  handleCommand(cmd);
}

// Commands that arrived over MQTT, executed here in the main loop (never inside the MQTT task)
void handleRemote() {
  Msg m;
  while (xQueueReceive(qIn, &m, 0) == pdTRUE) {
    lastRemote = millis();
    replyTo = &mqttPrint;
    handleCommand(String(m.text));
    mqttPrint.flush();
    replyTo = &Serial;
  }
}

// ================= Main =================

void setup() {
  Serial.begin(115200);
  Serial.setTimeout(20);

  // Motor driver stays disabled until both PWM outputs are at 0
  pinMode(EN_PIN, OUTPUT);
  digitalWrite(EN_PIN, LOW);
  ledcAttach(RPWM_PIN, PWM_FREQ, PWM_RES);
  ledcAttach(LPWM_PIN, PWM_FREQ, PWM_RES);
  applyMotor();
  analogSetAttenuation(ADC_11db);

  setenv("TZ", TZ_INFO, 1);
  tzset();

  prefs.begin("logger", false);
  loadParams();

  loadConfig();
  snprintf(tTele,   sizeof(tTele),   "%s/telemetry", cfgBase);
  snprintf(tEvt,    sizeof(tEvt),    "%s/events",    cfgBase);
  snprintf(tResp,   sizeof(tResp),   "%s/resp",      cfgBase);
  snprintf(tCmd,    sizeof(tCmd),    "%s/cmd",       cfgBase);
  snprintf(tStatus, sizeof(tStatus), "%s/status",    cfgBase);
  qIn  = xQueueCreate(6,  sizeof(Msg));
  qOut = xQueueCreate(10, sizeof(Msg));

  startWiFi();
  initSD();
  if (CLOUD_ON) xTaskCreatePinnedToCore(mqttTask, "mqtt", 10240, nullptr, 1, nullptr, 0);

  printHelp(Serial);
  say("# Firmware %s, last reset: %s\n", FW_VERSION, resetReasonText());

  resetPID();
  if (!loadState()) {
    autoMode = true;
    say("# Fresh profile started: forward = cooling, reverse = heating. First target %.1f C\n", START_SETPOINT);
  }

#if USE_WATCHDOG
  enableLoopWDT();
#endif
}

void loop() {
  FEED_WDT();
  handleSerial();
  handleWiFi();
  handleRemote();
  applyMotor();

  // Dead-man: if armed and the agent has gone silent, switch the Peltier off
  if (deadmanMs != 0 && millis() - lastRemote > deadmanMs) {
    deadmanMs = 0;
    if (autoMode || direction != 0) {
      stopMotor();
      say("# DEAD-MAN: no remote command for too long - Peltier OFF, PID stopped. Type 'a' or 'sp <C>' to resume\n");
    }
  }

  static unsigned long lastLog = 0;
  if (millis() - lastLog >= LOG_MS) {
    lastLog = millis();
    float t1 = readTempC(NTC1_PIN, SERIES_R,  NOMINAL_R,  BETA);
    float t2 = readTempC(NTC2_PIN, SERIES_R,  NOMINAL_R,  BETA);
    float t3 = conditionWater(readTempC(NTC3_PIN, SERIES_R3, NOMINAL_R3, BETA3));

    // Low-pass the water reading: the guards and the PID both use the filtered value
    static float t3f = NAN;
    if (isnan(t3)) t3f = NAN;
    else t3f = isnan(t3f) ? t3 : pAlpha * t3f + (1.0f - pAlpha) * t3;

    safetyCheck(t3f);
    controlOutputs(t3f);

    float pwmPct = pwmValue * 100.0 / 255.0;

    char ts[32];
    getTimestamp(ts, sizeof(ts));

    if (liveOutput) {
      char line[260];
      int n = 0;
      if (TIME_IN_LIVE_LINE) n = snprintf(line, sizeof(line), "Time:%s,", ts);
      snprintf(line + n, sizeof(line) - n,
               "PWM_pct:%.1f,Direction:%d,Temp1_C:%.1f,Temp2_C:%.1f,Temp3_C:%.1f,Temp3f_C:%.2f,Setpoint_C:%.1f,Phase:%s\n",
               pwmPct, direction, t1, t2, t3, t3f, setpoint, phaseName());
      Serial.print(line);
      if (clientOk()) client.print(line);
    }

    bool pidRunning = autoMode && !faultLatched && !sensorFault;
    float p = pidRunning ? zW.lastP : 0.0f;
    float i = pidRunning ? zW.pidIntegral : 0.0f;
    float d = pidRunning ? zW.lastD : 0.0f;

    static unsigned long lastCloud = 0;
    if (millis() - lastCloud >= CLOUD_MS) {
      lastCloud = millis();
      sendTelemetry(pwmPct, t1, t2, t3, t3f, p, i, d);
    }

    if (sdOk) {
      size_t written = logFile.printf("%s,%lu,%.1f,%d,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%s,%.2f\n",
                                      ts, lastLog, pwmPct, direction, t1, t2, t3, setpoint, p, i, d, phaseName(), t3f);
      if (written == 0) {
        sdOk = false;
        say("# SD write failed - card removed? Auto-recovery will retry every 30 s\n");
      } else if (millis() - lastFlush >= FLUSH_MS) {
        lastFlush = millis();
        logFile.flush();
        if (logFile.size() > MAX_LOG_BYTES) {
          logFile.close();
          sdOk = false;
          say("# Log file is large - continuing in a new file\n");
          openNewLog();
        }
      }
    }

    maybeSaveState();
    sdAutoRecover();
  }
}
