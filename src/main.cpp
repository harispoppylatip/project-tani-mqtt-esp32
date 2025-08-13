#include <Arduino.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <Preferences.h>
#include <time.h>

// ===================== KONFIGURASI PIN =====================
#define PIN_DS18B20   15     // DS18B20
#define PIN_SOIL      34     // Soil moisture (ADC)
#define PIN_PH        35     // PH-4502C analog output -> ADC
#define PIN_RELAY     25     // Relay

// ===================== LCD 16x2 (I2C) =====================
LiquidCrystal_I2C lcd(0x27, 16, 2); // ganti 0x27 -> 0x3F bila perlu

// ===================== DS18B20 =====================
OneWire oneWire(PIN_DS18B20);
DallasTemperature sensors(&oneWire);

// ===================== WiFi & MQTT =====================
const char* WIFI_SSID     = "NANABUN";
const char* WIFI_PASS     = "Alifzah05";
const char* MQTT_HOST     = "broker.hivemq.com";
const uint16_t MQTT_PORT  = 1883;

// Topic MQTT
const char* TOPIC_TELE        = "growy/telemetry";            // publish data
const char* TOPIC_RELAYSET    = "growy/relay/set";            // subscribe kontrol relay (ON/OFF / {"relay":true})
const char* TOPIC_RELAYSTAT   = "growy/relay/state";          // publish status relay
const char* TOPIC_SCH_SET     = "growy/relay/schedule/set";   // set jadwal
const char* TOPIC_SCH_STATE   = "growy/relay/schedule/state"; // status jadwal

WiFiClient espClient;
PubSubClient mqtt(espClient);
Preferences prefs;

// ===================== KALIBRASI SOIL =====================
const int SOIL_WET = 1230;   // nilai ADC saat basah (kalibrasi!)
const int SOIL_DRY = 2550;   // nilai ADC saat kering (kalibrasi!)

// ===================== KALIBRASI pH =====================
// Isi dengan hasil bacaan miliVolt saat kalibrasi pH7 & pH4
float PH7_MV = 1500.0f;   // contoh
float PH4_MV = 2032.0f;   // contoh
// Rumus 2-titik: pH = 7 + (VmV - PH7_MV) * ((4 - 7) / (PH4_MV - PH7_MV))

// ===================== INTERVAL PUBLISH =====================
unsigned long lastPublish = 0;
const unsigned long PUBLISH_EVERY_MS = 5000;

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
const long GMT_OFFSET_SEC = 8 * 3600; // Samarinda (Asia/Makassar, WITA)
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
  time_t run_until = 0;        // epoch, kapan berhenti
  bool running = false;        // sedang menjalankan jadwal
} scheduleCfg;

// Gantikan last_run_yday dengan cap per-menit (epochMenit)
long last_trigger_minute = -1; // = floor(epochDetik/60) ketika terakhir trigger

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

void setRelay(bool on) {
  relayState = on;
  if (RELAY_ACTIVE_LOW) digitalWrite(PIN_RELAY, on ? LOW : HIGH);
  else                  digitalWrite(PIN_RELAY, on ? HIGH : LOW);
  saveRelayToNVS();
}

int readSoilPercent() {
  analogSetPinAttenuation(PIN_SOIL, ADC_11db); // ~0-3.6V
  int raw = analogRead(PIN_SOIL);              // 0..4095
  int percent = map(raw, SOIL_DRY, SOIL_WET, 0, 100);
  return constrain(percent, 0, 100);
}

float readTemperatureC() {
  sensors.requestTemperatures();
  float t = sensors.getTempCByIndex(0);
  return t; // bisa -127 jika error
}

// ===================== Pembacaan pH =====================
int readMilliVoltsPH(uint8_t samples = 10, uint16_t delayMs = 5) {
  analogSetPinAttenuation(PIN_PH, ADC_11db); // ~0-3.6V
  long acc = 0;
  for (uint8_t i = 0; i < samples; i++) {
    acc += analogReadMilliVolts(PIN_PH);
    delay(delayMs);
  }
  return (int)(acc / samples);
}

float readPH() {
  int mv = readMilliVoltsPH();
  if (fabs(PH4_MV - PH7_MV) < 1.0f) return NAN; // hindari div/0
  float slope = (4.0f - 7.0f) / (PH4_MV - PH7_MV); // -3 / (PH4-PH7)
  float ph = 7.0f + (mv - PH7_MV) * slope;
  if (ph < 0.0f) ph = 0.0f;
  if (ph > 14.0f) ph = 14.0f;
  return ph;
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
    doc["run_until"] = (uint32_t)scheduleCfg.run_until; // epoch seconds
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
  setRelay(relayState); // terapkan relay terakhir
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

        // Jika OFF manual via JSON, hentikan jadwal yang sedang berjalan
        if (!want) {
          cancelScheduleRun("manual OFF via JSON");
        }
      }
    }
    if (!handled) {
      String up = msg; up.toUpperCase();
      if (up == "ON")  {
        setRelay(true);
        publishRelayState();
        handled = true;
      }
      if (up == "OFF") {
        setRelay(false);
        publishRelayState();
        handled = true;
        cancelScheduleRun("manual OFF via string");
      }
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

      // Jika jadwal dimatikan, hentikan run aktif
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
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
  }
}

void ensureMQTT() {
  if (mqtt.connected()) return;
  while (!mqtt.connected()) {
    String cid = "esp32-growy-" + String((uint32_t)ESP.getEfuseMac(), HEX);
    if (mqtt.connect(cid.c_str())) {
      mqtt.subscribe(TOPIC_RELAYSET);
      mqtt.subscribe(TOPIC_SCH_SET);
      publishRelayState();
      publishScheduleState();
    } else {
      delay(1000);
    }
  }
}

void ensureTime() {
  if (timeReady) return;
  configTime(GMT_OFFSET_SEC, DST_OFFSET_SEC, NTP1, NTP2);
  // tunggu sampai waktu valid
  for (int i = 0; i < 20; i++) {
    time_t now = time(nullptr);
    if (now > 1700000000) { timeReady = true; break; } // ~2023-11-14
    delay(500);
  }
}

// ===================== Jadwal harian =====================
void handleSchedule() {
  if (!timeReady || !scheduleCfg.enabled) return;

  time_t nowSec = time(nullptr);
  struct tm localTm;
  localtime_r(&nowSec, &localTm);

  long thisMinuteStamp = (long)(nowSec / 60); // cap per-menit

  // Jika sedang running, cek apakah waktunya berhenti
  if (scheduleCfg.running) {
    if (nowSec >= scheduleCfg.run_until) {
      scheduleCfg.running = false;
      setRelay(false);
      publishRelayState();
      publishScheduleState();
    }
    return;
  }

  // Belum running: trigger SEKALI pada menit yang dijadwalkan
  if (localTm.tm_hour == scheduleCfg.hour &&
      localTm.tm_min  == scheduleCfg.minute &&
      last_trigger_minute != thisMinuteStamp) {

    // Mulai jadwal
    scheduleCfg.running = true;
    setRelay(true);
    scheduleCfg.run_until = nowSec + (time_t)scheduleCfg.duration_min * 60;
    last_trigger_minute = thisMinuteStamp; // cegah retrigger di menit yang sama
    publishRelayState();
    publishScheduleState();
  }
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
  lcd.setCursor(0, 1); lcd.print("Hub WiFi+MQTT");

  ensureWiFi();

  mqtt.setServer(MQTT_HOST, MQTT_PORT);
  mqtt.setCallback(mqttCallback);
  ensureMQTT();

  ensureTime();        // NTP + set zona WITA

  loadFromNVS();       // muat persistensi (relay & jadwal)
  // Reset status running saat boot
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
  float phVal    = readPH();           // pH sudah ada averaging internal (mV)

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
    Serial.printf("%04d-%02d-%02d %02d:%02d:%02d WITA\n",
                  localTm.tm_year + 1900,
                  localTm.tm_mon + 1,
                  localTm.tm_mday,
                  localTm.tm_hour,
                  localTm.tm_min,
                  localTm.tm_sec);
  } else {
    Serial.println("Waktu belum siap (NTP)...");
  }

  // --- Tampilkan ke LCD ---
  drawLCD(suhuMA, phVal, soilMA, relayState);

  // --- Jadwal harian ---
  handleSchedule();

  // --- Publish berkala ---
  unsigned long nowMs = millis();
  if (nowMs - lastPublish >= PUBLISH_EVERY_MS) {
    lastPublish = nowMs;
    publishTelemetry(suhuMA, soilMA, phVal);
  }

  delay(500);
}
