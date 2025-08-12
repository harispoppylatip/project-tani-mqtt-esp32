#include <Arduino.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

// ========== KONFIGURASI PIN ==========
#define PIN_DS18B20   15     // DS18B20
#define PIN_SOIL      34     // Soil moisture (ADC)
#define PIN_PH        35     // PH-4502C analog output -> ADC
#define PIN_RELAY     25     // Relay

// ========= LCD 16x2 (I2C) ============
LiquidCrystal_I2C lcd(0x27, 16, 2); // ganti 0x27 -> 0x3F jika perlu

// ========= DS18B20 ===========
OneWire oneWire(PIN_DS18B20);
DallasTemperature sensors(&oneWire);

// ========= WiFi & MQTT =========
const char* WIFI_SSID     = "NANABUN";
const char* WIFI_PASS     = "Alifzah05";
const char* MQTT_HOST     = "broker.hivemq.com";
const uint16_t MQTT_PORT  = 1883;

// Topic MQTT
const char* TOPIC_TELE      = "growy/telemetry";   // publish data
const char* TOPIC_RELAYSET  = "growy/relay/set";   // subscribe kontrol relay
const char* TOPIC_RELAYSTAT = "growy/relay/state"; // publish status relay

WiFiClient espClient;
PubSubClient mqtt(espClient);

// ======== KALIBRASI SOIL MOISTURE ========
const int SOIL_WET = 1230;   // nilai ADC saat basah (kalibrasi!)
const int SOIL_DRY = 2550;   // nilai ADC saat kering (kalibrasi!)

// ======== KALIBRASI pH (PH-4502C) ========
// Isi dengan hasil bacaan miliVolt saat kalibrasi pH7 & pH4
float PH7_MV = 1500.0f;   // contoh
float PH4_MV = 2032.0f;   // contoh
// Rumus 2-titik: pH = 7 + (VmV - PH7_MV) * ((4 - 7) / (PH4_MV - PH7_MV))

// ======== PUBLISH INTERVAL ========
unsigned long lastPublish = 0;
const unsigned long PUBLISH_EVERY_MS = 5000;

// ======== RELAY LOGIC ========
const bool RELAY_ACTIVE_LOW = false;
bool relayState = false;

// ======== MOVING AVERAGE (SMA) ========
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
  void reset(){ idx=0; count=0; sum=0; for(size_t i=0;i<N;i++) buf[i]=0; }
  size_t size() const { return N; }
  size_t filled() const { return count; }
private:
  float buf[N];
  size_t idx, count;
  double sum;
};

// atur ukuran window di sini
const uint8_t MA_SAMPLES_TEMP = 8;
const uint8_t MA_SAMPLES_SOIL = 8;

// pakai template dengan konstanta compile-time:
SMA<8> maTemp; // jika ubah MA_SAMPLES_TEMP != 8, ganti juga template <...>
SMA<8> maSoil; // jika ubah MA_SAMPLES_SOIL != 8, ganti juga template <...>

// ======== FUNGSI ========
void setRelay(bool on) {
  relayState = on;
  if (RELAY_ACTIVE_LOW) digitalWrite(PIN_RELAY, on ? LOW : HIGH);
  else                  digitalWrite(PIN_RELAY, on ? HIGH : LOW);
}

int readSoilPercent() {
  analogSetPinAttenuation(PIN_SOIL, ADC_11db); // ~0-3.6V
  int raw = analogRead(PIN_SOIL);              // 0..4095
  Serial.printf("Soil raw: %d\n", raw);
  int percent = map(raw, SOIL_DRY, SOIL_WET, 0, 100);
  return constrain(percent, 0, 100);
}

float readTemperatureC() {
  sensors.requestTemperatures();
  float t = sensors.getTempCByIndex(0);
  return t; // bisa -127 jika error
}

// ---- Pembacaan pH ----
int readMilliVoltsPH(uint8_t samples = 10, uint16_t delayMs = 5) {
  analogSetPinAttenuation(PIN_PH, ADC_11db); // ~0-3.6V
  long acc = 0;
  for (uint8_t i = 0; i < samples; i++) {
    acc += analogReadMilliVolts(PIN_PH);     // tersedia di core ESP32
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

// ---- LCD ----
// Baris 1: "Suhu:25.3C pH:6.8" (<=16 kolom)
// Baris 2: "Tnh:45% Relay:ON"
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

void publishRelayState() {
  StaticJsonDocument<64> doc;
  doc["relay"] = relayState;
  char buf[64];
  size_t n = serializeJson(doc, buf, sizeof(buf));
  mqtt.publish(TOPIC_RELAYSTAT, buf, n);
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

void mqttCallback(char* topic, byte* payload, unsigned int length) {
  String t = topic;
  String msg;
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
      }
    }
    if (!handled) {
      String up = msg; up.toUpperCase();
      if (up == "ON")  { setRelay(true);  publishRelayState(); handled = true; }
      if (up == "OFF") { setRelay(false); publishRelayState(); handled = true; }
    }
  }
}

void ensureWiFi() {
  if (WiFi.status() == WL_CONNECTED) return;
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  while (WiFi.status() != WL_CONNECTED) {
    Serial.println("Menghubungkan WiFi...");
    delay(500);
  }
  Serial.println("WiFi terhubung!");
  Serial.print("IP: ");
  Serial.println(WiFi.localIP());
}

void ensureMQTT() {
  if (mqtt.connected()) return;
  while (!mqtt.connected()) {
    String cid = "esp32-growy-" + String((uint32_t)ESP.getEfuseMac(), HEX);
    if (mqtt.connect(cid.c_str())) {
      mqtt.subscribe(TOPIC_RELAYSET);
      publishRelayState();
    } else {
      delay(1000);
    }
  }
}

void setup() {
  Serial.begin(115200);

  pinMode(PIN_RELAY, OUTPUT);
  setRelay(false); // default OFF saat boot

  sensors.begin();

  Wire.begin(); // SDA=21, SCL=22 (default ESP32)
  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Monitoring...");
  lcd.setCursor(0, 1);
  lcd.print("Menghub. WiFi");

  ensureWiFi();

  mqtt.setServer(MQTT_HOST, MQTT_PORT);
  mqtt.setCallback(mqttCallback);
  ensureMQTT();

  lcd.clear();
}

void loop() {
  ensureWiFi();
  ensureMQTT();
  mqtt.loop();

  // --- Baca sensor (raw) ---
  float suhuRaw  = readTemperatureC();
  int   soilRaw  = readSoilPercent();
  float phVal    = readPH();           // pH sudah ada averaging internal (mV)

  // --- Update Moving Average ---
  if (!(isnan(suhuRaw) || suhuRaw < -100)) maTemp.add(suhuRaw); // jangan masukkan nilai error
  maSoil.add((float)soilRaw);

  float suhuMA  = maTemp.value();                 // bisa NAN saat awal
  int   soilMA  = (int)round(maSoil.value());     // 0..100

  // Fallback: kalau buffer belum terisi, pakai raw
  if (isnan(suhuMA)) suhuMA = suhuRaw;
  if (isnan(maSoil.value())) soilMA = soilRaw;

  // --- Tampilkan ke LCD ---
  drawLCD(suhuMA, phVal, soilMA, relayState);

  // --- Publish berkala (pakai nilai yang sudah dihaluskan) ---
  unsigned long now = millis();
  if (now - lastPublish >= PUBLISH_EVERY_MS) {
    lastPublish = now;
    publishTelemetry(suhuMA, soilMA, phVal);
  }

  delay(1000);
}
