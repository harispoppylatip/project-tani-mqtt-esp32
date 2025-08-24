#include <Arduino.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <Preferences.h>
#include <time.h>
#include <HTTPClient.h>

// ===================== KONFIGURASI PIN =====================
#define PIN_DS18B20   15
#define PIN_SOIL1     34
#define PIN_SOIL2     32
#define PIN_PH        35
#define PIN_RELAY     25

// ===================== LCD 16x2 (I2C) =====================
LiquidCrystal_I2C lcd(0x27, 16, 2);

// ===================== DS18B20 =====================
OneWire oneWire(PIN_DS18B20);
DallasTemperature sensors(&oneWire);

// ===================== WiFi & MQTT (TLS) =====================
const char* WIFI_SSID   = "NANABUN";
const char* WIFI_PASS   = "Alifzah05";

const char* MQTT_HOST   = "d61b7602f60d488a985eb51f2e8a7e65.s1.eu.hivemq.cloud";
const uint16_t MQTT_PORT = 8883;
const char* MQTT_USER   = "manunggal";
const char* MQTT_PASSWD = "Jayaa333";

WiFiClientSecure secureClient;
PubSubClient mqtt(secureClient);
Preferences prefs;

// ===================== Topic MQTT =====================
const char* TOPIC_TELE        = "growy/telemetry";
const char* TOPIC_RELAYSET    = "growy/relay/set";
const char* TOPIC_RELAYSTAT   = "growy/relay/state";
const char* TOPIC_SCH_SET     = "growy/relay/schedule/set";
const char* TOPIC_SCH_STATE   = "growy/relay/schedule/state";

// ===================== HTTP API (opsional, default OFF) =====================
#define ENABLE_HTTP_POST 0
const char* API_URL = "https://haris-iot.my.id/senddata.php";
const char* API_KEY = "GROWY_SECRET_123";

// ===================== KALIBRASI SOIL =====================
// Soil 1 (PIN 34)
const int SOIL1_WET = 1230;
const int SOIL1_DRY = 2550;
// Soil 2 (PIN 32)
const int SOIL2_WET = 1230;
const int SOIL2_DRY = 2550;

// ===================== INTERVAL (ms) =====================
const unsigned long SENSOR_POLL_MS   = 1000;   // baca sensor periodik
const unsigned long LCD_REFRESH_MS   = 400;    // refresh LCD (lebih jarang -> stabil)
const unsigned long PAGE_SWITCH_MS   = 2000;   // ganti halaman LCD
const unsigned long TELEMETRY_MS     = 3000;   // publish MQTT
const unsigned long SCHEDULE_TICK_MS = 200;    // cek jadwal
const unsigned long WIFI_RETRY_MS    = 1000;   // coba konek WiFi
const unsigned long MQTT_RETRY_MS    = 2000;   // coba konek MQTT
const unsigned long NTP_CHECK_MS     = 5000;   // cek NTP
const unsigned long PH_SAMPLE_MS     = 100;    // jarak waktu sampling pH
const unsigned long PH_STALE_MS      = 5000;   // >5s tanpa nilai baru -> tampil "--.-"

#if ENABLE_HTTP_POST
const unsigned long DB_POST_INTERVAL_MS = 10UL * 60UL * 1000UL; // 10 menit
#endif

// ===================== RELAY LOGIC =====================
const bool RELAY_ACTIVE_LOW = false;
bool relayState = false;

// ===================== MOVING AVERAGE (SMA) =====================
template <size_t N>
class SMA {
public:
  SMA() : idx(0), count(0), sum(0) { for (size_t i=0;i<N;i++) buf[i]=0; }
  void add(float x) { sum += x - buf[idx]; buf[idx]=x; idx=(idx+1)%N; if(count<N)count++; }
  float value() const { return (count==0)?NAN:(float)(sum/count); }
private:
  float buf[N]; size_t idx, count; double sum;
};

SMA<8> maTemp;
SMA<8> maSoil1;
SMA<8> maSoil2;

// ===================== NTP & Timezone (WITA) =====================
const long GMT_OFFSET_SEC = 8 * 3600; // Asia/Makassar (WITA)
const int  DST_OFFSET_SEC = 0;
const char* NTP1 = "pool.ntp.org";
const char* NTP2 = "time.google.com";
bool timeReady = false;

// ===================== Jadwal =====================
struct Schedule {
  bool enabled = false;
  uint8_t hour = 6;            // 0..23
  uint8_t minute = 0;          // 0..59
  uint16_t duration_min = 10;  // durasi ON
  time_t run_until = 0;        // epoch kapan berhenti
  bool running = false;
} scheduleCfg;

long last_trigger_minute = -1;

// ===================== pH METHOD (sampling non-blocking) =====================
float calibration_value = 21.03f; // kalibrasi sesuai kebiasaanmu
int   ph_buffer[10];
int   ph_index = 0;
bool  phReady = false;            // 10 sampel lengkap dan dihitung
float phValue = NAN;              // nilai pH terakhir
unsigned long lastPhSampleMS = 0;
unsigned long lastPhUpdateMS = 0; // kapan terakhir update pH

// ===================== FORWARD DECLARATIONS =====================
int   readSoil1Percent();
int   readSoil2Percent();
float readTemperatureC();
void  startPhSampling();
void  tickPhSampling();
void  setRelay(bool on);
void  publishRelayState();
void  publishScheduleState();
void  saveRelayToNVS();
void  saveScheduleToNVS();
void  loadFromNVS();
void  cancelScheduleRun(const char* reason = nullptr);
void  handleScheduleTick();
void  mqttCallback(char* topic, byte* payload, unsigned int length);
void  ensureWiFi();
void  ensureMQTT();
void  ensureTime();
void  drawLCD(float suhuC, float phVal, int soil1Pct, int soil2Pct, bool rly);
void  publishTelemetry(float suhuC, int soil1Pct, int soil2Pct, float phVal);
#if ENABLE_HTTP_POST
void  postTelemetryToHTTP(float suhuC, int soil1Pct, int soil2Pct, float phVal);
#endif

// ===================== Helper Debug =====================
void logMqttRx(const char* topic, const byte* payload, unsigned int length) {
  Serial.print("[MQTT] RX topic="); Serial.print(topic); Serial.print(" payload='");
  for (unsigned int i = 0; i < length; i++) Serial.print((char)payload[i]);
  Serial.println("'");
}

// ===================== Persistensi (NVS) =====================
void saveRelayToNVS() {
  prefs.begin("growy", false);
  prefs.putBool("relay", relayState);
  prefs.end();
}
void saveScheduleToNVS() {
  prefs.begin("growy", false);
  prefs.putBool("sch_en", scheduleCfg.enabled);
  prefs.putUChar("sch_h", scheduleCfg.hour);
  prefs.putUChar("sch_m", scheduleCfg.minute);
  prefs.putUShort("sch_dur", scheduleCfg.duration_min);
  prefs.end();
}
void loadFromNVS() {
  prefs.begin("growy", true);
  relayState = prefs.getBool("relay", false);
  scheduleCfg.enabled = prefs.getBool("sch_en", false);
  scheduleCfg.hour = prefs.getUChar("sch_h", 6);
  scheduleCfg.minute = prefs.getUChar("sch_m", 0);
  scheduleCfg.duration_min = prefs.getUShort("sch_dur", 10);
  prefs.end();
  pinMode(PIN_RELAY, OUTPUT);
  digitalWrite(PIN_RELAY, RELAY_ACTIVE_LOW ? !relayState : relayState);
}

// ===================== Relay & Jadwal =====================
void setRelay(bool on) {
  relayState = on;
  digitalWrite(PIN_RELAY, RELAY_ACTIVE_LOW ? !on : on);
  saveRelayToNVS();
}
void cancelScheduleRun(const char* reason) {
  if (scheduleCfg.running) {
    scheduleCfg.running = false;
    scheduleCfg.run_until = 0;
    if (reason) Serial.printf("[SCHEDULE] Canceled: %s\n", reason);
    publishScheduleState();
  }
}
void handleScheduleTick() {
  if (!timeReady || !scheduleCfg.enabled) return;

  time_t nowSec = time(nullptr);
  struct tm localTm;
  localtime_r(&nowSec, &localTm);

  long thisMinuteStamp = (long)(nowSec / 60);

  // stop jika sudah selesai
  if (scheduleCfg.running) {
    if (nowSec >= scheduleCfg.run_until) {
      scheduleCfg.running = false;
      setRelay(false);
      publishRelayState();
      publishScheduleState();
    }
    return;
  }

  // trigger sekali pada menit yang ditentukan
  if (localTm.tm_hour == scheduleCfg.hour &&
      localTm.tm_min  == scheduleCfg.minute &&
      last_trigger_minute != thisMinuteStamp) {

    scheduleCfg.running = true;
    setRelay(true);
    scheduleCfg.run_until = nowSec + (time_t)scheduleCfg.duration_min * 60;
    last_trigger_minute = thisMinuteStamp;
    publishRelayState();
    publishScheduleState();
  }
}

// ===================== pH sampling =====================
void startPhSampling() {
  ph_index = 0;
  phReady = false;
}
void tickPhSampling() {
  if (ph_index >= 10) return;                     // sudah penuh
  if (millis() - lastPhSampleMS < PH_SAMPLE_MS) return;
  lastPhSampleMS = millis();
  analogSetPinAttenuation(PIN_PH, ADC_11db);
  ph_buffer[ph_index++] = analogRead(PIN_PH);     // 0..4095
  if (ph_index >= 10) {
    // sort
    for (int i = 0; i < 9; i++) {
      for (int j = i + 1; j < 10; j++) {
        if (ph_buffer[i] > ph_buffer[j]) {
          int tmp = ph_buffer[i];
          ph_buffer[i] = ph_buffer[j];
          ph_buffer[j] = tmp;
        }
      }
    }
    // ambil 6 nilai tengah (index 2..7)
    unsigned long sum = 0;
    for (int i = 2; i < 8; i++) sum += ph_buffer[i];

    float volt = (float)sum * 3.3f / 4095.0f / 6.0f;     // konversi ke Volt
    float ph_act = -5.70f * volt + calibration_value;    // rumus lama
    ph_act = constrain(ph_act, 0.0f, 14.0f);

    phValue = ph_act;
    phReady = true;            // siap dipakai
    lastPhUpdateMS = millis(); // tandai saat update terakhir
  }
}

// ===================== Sensor helpers =====================
int readSoil1Percent() {
  analogSetPinAttenuation(PIN_SOIL1, ADC_11db);
  int raw = analogRead(PIN_SOIL1);
  int percent = map(raw, SOIL1_DRY, SOIL1_WET, 0, 100);
  return constrain(percent, 0, 100);
}
int readSoil2Percent() {
  analogSetPinAttenuation(PIN_SOIL2, ADC_11db);
  int raw = analogRead(PIN_SOIL2);
  int percent = map(raw, SOIL2_DRY, SOIL2_WET, 0, 100);
  return constrain(percent, 0, 100);
}
float readTemperatureC() {
  // ambil hasil dari konversi sebelumnya
  float t = sensors.getTempCByIndex(0);
  // minta konversi berikutnya (pipeline)
  sensors.requestTemperatures();
  return t;
}

// ===================== WiFi / MQTT / TIME (non-blocking) =====================
unsigned long lastWifiTry = 0;
unsigned long lastMqttTry = 0;
unsigned long lastNtpCheck = 0;

void ensureWiFi() {
  if (WiFi.status() == WL_CONNECTED) return;
  if (millis() - lastWifiTry < WIFI_RETRY_MS) return;
  lastWifiTry = millis();
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.println("[WiFi] connecting...");
}
void ensureMQTT() {
  if (WiFi.status() != WL_CONNECTED) return;
  if (mqtt.connected()) return;
  if (millis() - lastMqttTry < MQTT_RETRY_MS) return;
  lastMqttTry = millis();

  secureClient.setTimeout(15);
  secureClient.setInsecure(); // cepat; ganti setCACert() jika mau verifikasi CA

  mqtt.setServer(MQTT_HOST, MQTT_PORT);
  mqtt.setBufferSize(512);
  mqtt.setCallback(mqttCallback);

  String cid = "esp32-growy-" + String((uint32_t)ESP.getEfuseMac(), HEX);
  Serial.print("[MQTT] connecting...");
  if (mqtt.connect(cid.c_str(), MQTT_USER, MQTT_PASSWD)) {
    bool s1 = mqtt.subscribe(TOPIC_RELAYSET);
    bool s2 = mqtt.subscribe(TOPIC_SCH_SET);
    publishRelayState();
    publishScheduleState();
    Serial.printf(" OK (subs: %s=%s, %s=%s)\n",
      TOPIC_RELAYSET, s1?"OK":"FAIL",
      TOPIC_SCH_SET,  s2?"OK":"FAIL");
  } else {
    Serial.printf(" FAIL rc=%d\n", mqtt.state());
  }
}
void ensureTime() {
  if (timeReady) return;
  if (millis() - lastNtpCheck < NTP_CHECK_MS) return;
  lastNtpCheck = millis();

  static bool configured = false;
  if (!configured) {
    configTime(GMT_OFFSET_SEC, DST_OFFSET_SEC, NTP1, NTP2);
    configured = true;
  }
  time_t now = time(nullptr);
  if (now > 1700000000) {
    timeReady = true;
    Serial.println("[NTP] time ready");
  } else {
    Serial.println("[NTP] waiting time...");
  }
}

// ===================== LCD (ANTI-FLICKER) & Telemetry =====================
// Helper: pad/truncate ke 16 kolom
static inline String fit16(const String& s) {
  if (s.length() >= 16) return s.substring(0, 16);
  String out = s; out.reserve(16);
  while (out.length() < 16) out += ' ';
  return out;
}

// Cache line per page supaya hanya update kalau berubah
void drawLCD(float suhuC, float phVal, int soil1Pct, int soil2Pct, bool rly) {
  static uint8_t page = 0;
  static unsigned long lastPageSwitch = 0;

  // cache teks terakhir ditampilkan (per-row)
  static String lastLine0 = "";
  static String lastLine1 = "";
  static uint8_t lastPageShown = 255;

  // ganti halaman tiap PAGE_SWITCH_MS
  if (millis() - lastPageSwitch >= PAGE_SWITCH_MS) {
    lastPageSwitch = millis();
    page ^= 1;
  }

  // bentuk string untuk page aktif
  String line0, line1;

  if (page == 0) {
    // Page 0: "R:ON  pH:7.0" | "S1:45% S2:40%"
    line0 = String("Relay:") + (rly ? "ON " : "OFF") + " pH:" + (isnan(phVal) ? String("--.-") : String(phVal, 1));
    line1 = String("S1:") + soil1Pct + "%    S2:" + soil2Pct + "%";
  } else {
    // Page 1: "Temp:25.1C" | "S1:45% S2:40%"
    line0 = String("Temp:") + ((isnan(suhuC) || suhuC < -100) ? String("--.-C") : String(String(suhuC, 1) + "C"));
    line1 = String("S1:") + soil1Pct + "%    S2:" + soil2Pct + "%";
  }

  // pad/truncate ke 16 kolom
  line0 = fit16(line0);
  line1 = fit16(line1);

  // Jika ganti page, paksa redraw (tanpa clear)
  bool force = (lastPageShown != page);
  if (force || line0 != lastLine0) {
    lcd.setCursor(0, 0);
    lcd.print(line0);
    lastLine0 = line0;
  }
  if (force || line1 != lastLine1) {
    lcd.setCursor(0, 1);
    lcd.print(line1);
    lastLine1 = line1;
  }

  lastPageShown = page;
}

void publishRelayState() {
  StaticJsonDocument<64> doc;
  doc["relay"] = relayState;
  char buf[64];
  size_t n = serializeJson(doc, buf, sizeof(buf));
  mqtt.publish(TOPIC_RELAYSTAT, buf, n);
}
void publishScheduleState() {
  StaticJsonDocument<160> doc;
  doc["enabled"] = scheduleCfg.enabled;
  doc["hour"] = scheduleCfg.hour;
  doc["minute"] = scheduleCfg.minute;
  doc["duration_min"] = scheduleCfg.duration_min;
  doc["running"] = scheduleCfg.running;
  if (scheduleCfg.running) doc["run_until"] = (uint32_t)scheduleCfg.run_until;
  char buf[200];
  size_t n = serializeJson(doc, buf, sizeof(buf));
  mqtt.publish(TOPIC_SCH_STATE, buf, n);
}
void publishTelemetry(float suhuC, int soil1Pct, int soil2Pct, float phVal) {
  StaticJsonDocument<256> doc;
  if (isnan(suhuC) || suhuC < -100) doc["suhu"] = nullptr; else doc["suhu"] = suhuC;
  if (isnan(phVal))                 doc["ph"]   = nullptr; else doc["ph"]   = phVal;
  doc["kelembapan_tanah"]  = soil1Pct; // soil1
  doc["kelembapan_tanah2"] = soil2Pct; // soil2
  doc["relay"] = relayState;
  char buf[280];
  size_t n = serializeJson(doc, buf, sizeof(buf));
  mqtt.publish(TOPIC_TELE, buf, n);
}

// ===================== MQTT callback =====================
void mqttCallback(char* topic, byte* payload, unsigned int length) {
  logMqttRx(topic, payload, length);
  String t = topic;
  String msg; msg.reserve(length + 1);
  for (unsigned int i = 0; i < length; i++) msg += (char)payload[i];

  if (t == TOPIC_RELAYSET) {
    StaticJsonDocument<64> doc;
    DeserializationError err = deserializeJson(doc, msg);
    bool handled = false;
    if (!err) {
      if (doc.containsKey("relay")) {
        bool want = doc["relay"];
        setRelay(want);
        publishRelayState();
        handled = true;
        if (!want) cancelScheduleRun("manual OFF via JSON");
      }
    }
    if (!handled) {
      String up = msg; up.toUpperCase();
      if (up == "ON")  { setRelay(true);  publishRelayState(); handled = true; }
      if (up == "OFF") { setRelay(false); publishRelayState(); handled = true; cancelScheduleRun("manual OFF via string"); }
    }
  }
  else if (t == TOPIC_SCH_SET) {
    StaticJsonDocument<192> doc;
    if (deserializeJson(doc, msg) == DeserializationError::Ok) {
      bool prevEnabled = scheduleCfg.enabled;
      if (doc.containsKey("enable"))       scheduleCfg.enabled = doc["enable"];
      if (doc.containsKey("hour"))         scheduleCfg.hour = constrain((int)doc["hour"], 0, 23);
      if (doc.containsKey("minute"))       scheduleCfg.minute = constrain((int)doc["minute"], 0, 59);
      if (doc.containsKey("duration_min")) scheduleCfg.duration_min = max(1, (int)doc["duration_min"]);

      if (prevEnabled && !scheduleCfg.enabled) {
        cancelScheduleRun("schedule disabled");
        setRelay(false);
        publishRelayState();
      }
      saveScheduleToNVS();
      publishScheduleState();
    }
  }
}

#if ENABLE_HTTP_POST
void postTelemetryToHTTP(float suhuC, int soil1Pct, int soil2Pct, float phVal) {
  if (WiFi.status() != WL_CONNECTED) return;
  StaticJsonDocument<256> doc;
  if (isnan(suhuC) || suhuC < -100) doc["suhu"] = nullptr; else doc["suhu"] = suhuC;
  if (isnan(phVal))                 doc["ph"]   = nullptr; else doc["ph"]   = phVal;
  doc["kelembapan_tanah"]  = soil1Pct;
  doc["kelembapan_tanah2"] = soil2Pct;
  char jsonBuf[280];
  size_t n = serializeJson(doc, jsonBuf, sizeof(jsonBuf));
  HTTPClient http; WiFiClientSecure https; https.setInsecure();
  if (!http.begin(https, API_URL)) { Serial.println("[HTTP] begin failed"); return; }
  http.addHeader("Content-Type", "application/json");
  http.addHeader("X-API-KEY", API_KEY);
  int code = http.POST((uint8_t*)jsonBuf, n);
  Serial.printf("[HTTP] POST code=%d\n", code);
  http.end();
}
#endif

// ===================== Timers =====================
unsigned long tSensor = 0;
unsigned long tTele   = 0;
unsigned long tSched  = 0;
#if ENABLE_HTTP_POST
unsigned long tHttp   = 0;
#endif

// ===================== SETUP & LOOP =====================
void setup() {
  Serial.begin(115200);

  // I2C lebih cepat untuk kurangi flicker
  Wire.begin();
  Wire.setClock(400000);

  lcd.init();
  lcd.backlight();
  lcd.noBlink();
  lcd.noCursor();
  lcd.setCursor(0, 0); lcd.print("Monitoring...");
  lcd.setCursor(0, 1); lcd.print("WiFi+MQTT TLS...");

  pinMode(PIN_RELAY, OUTPUT);
  setRelay(false); // default OFF

  sensors.begin();
  sensors.requestTemperatures(); // seed pertama

  ensureWiFi();
  ensureMQTT();
  ensureTime();

  loadFromNVS();
  scheduleCfg.running = false;
  scheduleCfg.run_until = 0;
  last_trigger_minute = -1;

  publishScheduleState();

  startPhSampling(); // mulai siklus pH
  lastPhUpdateMS = 0; // awalnya belum ada data pH
}

void loop() {
  // konektivitas
  ensureWiFi();
  ensureMQTT();
  ensureTime();
  mqtt.loop();

  // sampling pH non-blocking
  tickPhSampling();

  const unsigned long nowMS = millis();

  // --- Polling suhu & tanah ---
  static float cachedSuhu = NAN;
  static int   cachedSoil1 = 0;
  static int   cachedSoil2 = 0;
  static unsigned long tSensor = 0;
  if (nowMS - tSensor >= SENSOR_POLL_MS) {
    tSensor = nowMS;

    float suhuRaw  = readTemperatureC();  // pipeline DS18B20
    int   soil1Raw = readSoil1Percent();
    int   soil2Raw = readSoil2Percent();

    if (!(isnan(suhuRaw) || suhuRaw < -100)) maTemp.add(suhuRaw);
    maSoil1.add((float)soil1Raw);
    maSoil2.add((float)soil2Raw);

    cachedSuhu  = isnan(maTemp.value())    ? suhuRaw   : maTemp.value();
    cachedSoil1 = (int)round(isnan(maSoil1.value()) ? soil1Raw : maSoil1.value());
    cachedSoil2 = (int)round(isnan(maSoil2.value()) ? soil2Raw : maSoil2.value());
  }

  // --- Jadwal tick ---
  static unsigned long tSched = 0;
  if (nowMS - tSched >= SCHEDULE_TICK_MS) {
    tSched = nowMS;
    handleScheduleTick();
  }

  // --- LCD refresh (anti-flicker): cukup panggil tiap loop, drawLCD internal yang bandingkan teks) ---
  float phShow = (!isnan(phValue) && (nowMS - lastPhUpdateMS <= PH_STALE_MS)) ? phValue : NAN;
  drawLCD(cachedSuhu, phShow, cachedSoil1, cachedSoil2, relayState);

  // --- Telemetry MQTT ---
  static unsigned long tTele = 0;
  if (nowMS - tTele >= TELEMETRY_MS) {
    tTele = nowMS;

    if (timeReady) {
      time_t nowSec = time(nullptr);
      struct tm localTm; localtime_r(&nowSec, &localTm);
      Serial.printf("%04d-%02d-%02d %02d:%02d:%02d WITA | pH=%s | T=%.1fC | Soil1=%d%% | Soil2=%d%% | Relay=%s\n",
                    localTm.tm_year + 1900, localTm.tm_mon + 1, localTm.tm_mday,
                    localTm.tm_hour, localTm.tm_min, localTm.tm_sec,
                    ((!isnan(phValue) && (nowMS - lastPhUpdateMS <= PH_STALE_MS)) ? String(phValue, 2).c_str() : "--.-"),
                    cachedSuhu, cachedSoil1, cachedSoil2, (relayState ? "ON" : "OFF"));
    } else {
      Serial.println("Waktu belum siap (NTP)...");
    }

    float phForPublish = (!isnan(phValue) && (nowMS - lastPhUpdateMS <= PH_STALE_MS)) ? phValue : NAN;
    publishTelemetry(cachedSuhu, cachedSoil1, cachedSoil2, phForPublish);

#if ENABLE_HTTP_POST
    static unsigned long tHttp = 0;
    if (nowMS - tHttp >= DB_POST_INTERVAL_MS) {
      tHttp = nowMS;
      postTelemetryToHTTP(cachedSuhu, cachedSoil1, cachedSoil2, phForPublish);
    }
#endif

    // mulai ulang siklus pH agar nilai selalu terbaru
    startPhSampling();
  }
}
