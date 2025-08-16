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
#define PIN_DS18B20   15     // DS18B20
#define PIN_SOIL      34     // Soil moisture (ADC)
#define PIN_PH        35     // pH analog (metode lama, lokal)
#define PIN_RELAY     25     // Relay

// ===================== LCD 16x2 (I2C) =====================
LiquidCrystal_I2C lcd(0x27, 16, 2); // ganti 0x27 -> 0x3F bila perlu

// ===================== DS18B20 =====================
OneWire oneWire(PIN_DS18B20);
DallasTemperature sensors(&oneWire);

// ===================== WiFi & MQTT (TLS) =====================
const char* WIFI_SSID     = "NANABUN";
const char* WIFI_PASS     = "Alifzah05";

const char* MQTT_HOST     = "d61b7602f60d488a985eb51f2e8a7e65.s1.eu.hivemq.cloud";
const uint16_t MQTT_PORT  = 8883;     // TLS
const char* MQTT_USER     = "manunggal";
const char* MQTT_PASSWD   = "Jayaa333";

WiFiClientSecure secureClient;
PubSubClient mqtt(secureClient);
Preferences prefs;

// ===================== Topic MQTT =====================
const char* TOPIC_TELE        = "growy/telemetry";             // publish data
const char* TOPIC_RELAYSET    = "growy/relay/set";
const char* TOPIC_RELAYSTAT   = "growy/relay/state";
const char* TOPIC_SCH_SET     = "growy/relay/schedule/set";
const char* TOPIC_SCH_STATE   = "growy/relay/schedule/state";

// ===================== HTTP API (HOSTING WEB) =====================
// GANTI dengan URL endpoint PHP kamu
const char* API_URL = "https://haris-iot.my.id/senddata.php";
const char* API_KEY = "GROWY_SECRET_123";

// ===================== KALIBRASI SOIL =====================
const int SOIL_WET = 1230;   // ADC basah (kalibrasi!)
const int SOIL_DRY = 2550;   // ADC kering (kalibrasi!)

// ===================== INTERVAL =====================
// Interval publish MQTT (ms)
unsigned long lastPublish = 0;
const unsigned long PUBLISH_EVERY_MS = 5000; // 5 detik

// Interval kirim ke DATABASE (menit) — GANTI ANGKA INI SAJA
const uint16_t DB_POST_EVERY_MIN = 10; // <<< tinggal ganti
unsigned long lastDbPost = 0;
const unsigned long DB_POST_INTERVAL_MS = (unsigned long)DB_POST_EVERY_MIN * 60UL * 1000UL;

// ===================== RELAY LOGIC =====================
const bool RELAY_ACTIVE_LOW = false;
bool relayState = false;

// ===================== MOVING AVERAGE (SMA) =====================
template <size_t N>
class SMA {
public:
  SMA() : idx(0), count(0), sum(0) { for (size_t i=0;i<N;i++) buf[i]=0; }
  void add(float x) {
    sum += x - buf[idx];
    buf[idx] = x;
    idx = (idx + 1) % N;
    if (count < N) count++;
  }
  float value() const { return (count == 0) ? NAN : (float)(sum / count); }
private:
  float buf[N]; size_t idx, count; double sum;
};

SMA<8> maTemp;
SMA<8> maSoil;

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

// ===================== pH METHOD (SESUAI SCRIPT LAMA) =====================
float calibration_value = 21.34f + 1.5f; // sesuaikan hasil kalibrasi
unsigned long int ph_avgval;
int ph_buffer[10], ph_temp;

float readPhOld() {
  analogSetPinAttenuation(PIN_PH, ADC_11db); // ~0..3.3V
  for (int i = 0; i < 10; i++) {
    ph_buffer[i] = analogRead(PIN_PH);
    delay(30);
  }
  for (int i = 0; i < 9; i++) {
    for (int j = i + 1; j < 10; j++) {
      if (ph_buffer[i] > ph_buffer[j]) {
        ph_temp = ph_buffer[i];
        ph_buffer[i] = ph_buffer[j];
        ph_buffer[j] = ph_temp;
      }
    }
  }
  ph_avgval = 0;
  for (int i = 2; i < 8; i++) ph_avgval += ph_buffer[i];

  float volt = (float)ph_avgval * 3.3f / 4095.0f / 6.0f;
  float ph_act = -5.70f * volt + calibration_value;
  if (ph_act < 0.0f)  ph_act = 0.0f;
  if (ph_act > 14.0f) ph_act = 14.0f;
  return ph_act;
}

// ===================== Deklarasi fungsi =====================
void publishRelayState();
void publishScheduleState();
void saveRelayToNVS();
void saveScheduleToNVS();
void loadFromNVS();
void ensureWiFi();
void ensureMQTT();
void ensureTime();
void cancelScheduleRun(const char* reason = nullptr);
void postTelemetryToHTTP(float suhuC, int soilPct, float phVal);

// ===================== Helper Debug =====================
void logMqttRx(const char* topic, const byte* payload, unsigned int length) {
  Serial.print("[MQTT] RX topic="); Serial.print(topic); Serial.print(" payload='");
  for (unsigned int i = 0; i < length; i++) Serial.print((char)payload[i]);
  Serial.println("'");
}

void setRelay(bool on) {
  relayState = on;
  if (RELAY_ACTIVE_LOW) digitalWrite(PIN_RELAY, on ? LOW : HIGH);
  else                  digitalWrite(PIN_RELAY, on ? HIGH : LOW);
  saveRelayToNVS();
}

int readSoilPercent() {
  analogSetPinAttenuation(PIN_SOIL, ADC_11db);
  int raw = analogRead(PIN_SOIL);
  int percent = map(raw, SOIL_DRY, SOIL_WET, 0, 100);
  return constrain(percent, 0, 100);
}

float readTemperatureC() {
  sensors.requestTemperatures();
  float t = sensors.getTempCByIndex(0);
  return t; // bisa -127 jika error
}

// ===================== LCD =====================
void drawLCD(float suhuC, float phVal, int soilPct, bool rly) {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Suhu:");
  if (isnan(suhuC) || suhuC < -100) lcd.print("--.-C");
  else { lcd.print(String(suhuC, 1)); lcd.print("C"); }
  lcd.print(" ");
  lcd.print("pH:");
  if (isnan(phVal)) lcd.print("--.-");
  else              lcd.print(String(phVal, 1));

  lcd.setCursor(0, 1);
  lcd.print("Tnh:");
  lcd.print(soilPct);
  lcd.print("% ");
  lcd.print("Relay:");
  lcd.print(rly ? "ON" : "OFF");
}

// ===================== MQTT publish =====================
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
  if (scheduleCfg.running) {
    doc["run_until"] = (uint32_t)scheduleCfg.run_until;
  }
  char buf[200];
  size_t n = serializeJson(doc, buf, sizeof(buf));
  mqtt.publish(TOPIC_SCH_STATE, buf, n);
}

void publishTelemetry(float suhuC, int soilPct, float phVal) {
  StaticJsonDocument<192> doc;
  if (isnan(suhuC) || suhuC < -100) doc["suhu"] = nullptr; else doc["suhu"] = suhuC;
  if (isnan(phVal))                 doc["ph"]   = nullptr; else doc["ph"]   = phVal;
  doc["kelembapan_tanah"] = soilPct;
  doc["relay"] = relayState;

  char buf[220];
  size_t n = serializeJson(doc, buf, sizeof(buf));
  mqtt.publish(TOPIC_TELE, buf, n);
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
  setRelay(relayState);
}

// ===================== Pembatal Jadwal =====================
void cancelScheduleRun(const char* reason) {
  if (scheduleCfg.running) {
    scheduleCfg.running = false;
    scheduleCfg.run_until = 0;
    if (reason) Serial.printf("[SCHEDULE] Canceled: %s\n", reason);
    publishScheduleState();
  }
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

// ===================== WiFi / MQTT / TIME =====================
void ensureWiFi() {
  if (WiFi.status() == WL_CONNECTED) return;
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  while (WiFi.status() != WL_CONNECTED) { delay(500); }
}

void ensureMQTT() {
  if (mqtt.connected()) return;

  secureClient.setTimeout(15);
  secureClient.setInsecure(); // cepat; bisa ganti setCACert(ROOT_CA)

  mqtt.setServer(MQTT_HOST, MQTT_PORT);
  mqtt.setBufferSize(512);
  mqtt.setCallback(mqttCallback);

  while (!mqtt.connected()) {
    String cid = "esp32-growy-" + String((uint32_t)ESP.getEfuseMac(), HEX);
    if (mqtt.connect(cid.c_str(), MQTT_USER, MQTT_PASSWD)) {
      bool s1 = mqtt.subscribe(TOPIC_RELAYSET);
      bool s2 = mqtt.subscribe(TOPIC_SCH_SET);
      publishRelayState();
      publishScheduleState();
      Serial.printf("[MQTT] connected, subscribed: %s=%s, %s=%s\n",
        TOPIC_RELAYSET, s1?"OK":"FAIL",
        TOPIC_SCH_SET,  s2?"OK":"FAIL");
    } else {
      Serial.printf("[MQTT] connect failed rc=%d, retry...\n", mqtt.state());
      delay(1500);
    }
  }
}

void ensureTime() {
  if (timeReady) return;
  configTime(GMT_OFFSET_SEC, DST_OFFSET_SEC, NTP1, NTP2);
  for (int i = 0; i < 20; i++) {
    time_t now = time(nullptr);
    if (now > 1700000000) { timeReady = true; break; }
    delay(500);
  }
}

// ===================== Jadwal harian =====================
void handleSchedule() {
  if (!timeReady || !scheduleCfg.enabled) return;

  time_t nowSec = time(nullptr);
  struct tm localTm;
  localtime_r(&nowSec, &localTm);

  long thisMinuteStamp = (long)(nowSec / 60);

  if (scheduleCfg.running) {
    if (nowSec >= scheduleCfg.run_until) {
      scheduleCfg.running = false;
      setRelay(false);
      publishRelayState();
      publishScheduleState();
    }
    return;
  }

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

// ===================== HTTP POST helper =====================
void postTelemetryToHTTP(float suhuC, int soilPct, float phVal) {
  if (WiFi.status() != WL_CONNECTED || API_URL == nullptr || strlen(API_URL) < 8) return;

  StaticJsonDocument<192> doc;
  if (isnan(suhuC) || suhuC < -100) doc["suhu"] = nullptr; else doc["suhu"] = suhuC;
  if (isnan(phVal))                 doc["ph"]   = nullptr; else doc["ph"]   = phVal;
  doc["kelembapan_tanah"] = soilPct;

  char jsonBuf[220];
  size_t n = serializeJson(doc, jsonBuf, sizeof(jsonBuf));

  HTTPClient http;
  http.setConnectTimeout(6000);
  http.setTimeout(8000);
  http.setFollowRedirects(HTTPC_FORCE_FOLLOW_REDIRECTS);

  String url(API_URL);
  int code = -1;

  if (url.startsWith("https://")) {
    WiFiClientSecure https;
    https.setInsecure();
    if (!http.begin(https, url)) { Serial.println("[HTTP] begin(https) failed"); return; }
  } else {
    WiFiClient client;
    if (!http.begin(client, url)) { Serial.println("[HTTP] begin(http) failed"); return; }
  }

  http.addHeader("Content-Type", "application/json");
  http.addHeader("X-API-KEY", API_KEY);

  code = http.POST((uint8_t*)jsonBuf, n);

  if (code > 0) {
    String resp = http.getString();
    Serial.printf("[HTTP] POST %d | %s\n", code, resp.c_str());
  } else {
    Serial.printf("[HTTP] POST failed, err=%s\n", http.errorToString(code).c_str());
  }
  http.end();
}

// ===================== SETUP & LOOP =====================
void setup() {
  Serial.begin(115200);

  pinMode(PIN_RELAY, OUTPUT);
  setRelay(false); // default OFF saat boot

  sensors.begin();

  Wire.begin(); // SDA=21, SCL=22 (default ESP32)
  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0, 0); lcd.print("Monitoring...");
  lcd.setCursor(0, 1); lcd.print("Hub WiFi+MQTT TLS");

  ensureWiFi();
  ensureMQTT();
  ensureTime();

  loadFromNVS();
  scheduleCfg.running = false;
  scheduleCfg.run_until = 0;
  last_trigger_minute = -1;

  publishScheduleState();

  lcd.clear();
}

void loop() {
  ensureWiFi();
  ensureMQTT();
  ensureTime();
  mqtt.loop();

  // --- Baca sensor (raw) ---
  float suhuRaw  = readTemperatureC();
  int   soilRaw  = readSoilPercent();
  float phVal    = readPhOld();   // pH lokal

  // --- Update Moving Average ---
  if (!(isnan(suhuRaw) || suhuRaw < -100)) maTemp.add(suhuRaw);
  maSoil.add((float)soilRaw);

  float suhuMA  = isnan(maTemp.value()) ? suhuRaw : maTemp.value();
  int   soilMA  = (int)round(isnan(maSoil.value()) ? soilRaw : maSoil.value());

  // === CETAK WAKTU KE SERIAL ===
  if (timeReady) {
    time_t nowSec = time(nullptr);
    struct tm localTm;
    localtime_r(&nowSec, &localTm);
    Serial.printf("%04d-%02d-%02d %02d:%02d:%02d WITA | pH=%.2f | T=%.1fC | Soil=%d%%\n",
                  localTm.tm_year + 1900,
                  localTm.tm_mon + 1,
                  localTm.tm_mday,
                  localTm.tm_hour,
                  localTm.tm_min,
                  localTm.tm_sec,
                  phVal, suhuMA, soilMA);
  } else {
    Serial.println("Waktu belum siap (NTP)...");
  }

  // --- Tampilkan ke LCD ---
  drawLCD(suhuMA, phVal, soilMA, relayState);

  // --- Jadwal harian ---
  handleSchedule();

  // --- Publish MQTT berkala ---
  unsigned long nowMs = millis();
  if (nowMs - lastPublish >= PUBLISH_EVERY_MS) {
    lastPublish = nowMs;
    publishTelemetry(suhuMA, soilMA, phVal);
  }

  // --- Kirim ke DATABASE hanya tiap X menit ---
  if (nowMs - lastDbPost >= DB_POST_INTERVAL_MS) {
    lastDbPost = nowMs;
    postTelemetryToHTTP(suhuMA, soilMA, phVal);
  }

  delay(500);
}
