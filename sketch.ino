#include <WiFi.h>
#include <PubSubClient.h>
#include <LiquidCrystal_I2C.h>
#include <TinyGPS++.h>
#include <HardwareSerial.h>
#include <ArduinoJson.h>

// WiFi
const char* ssid = "Wokwi-GUEST";
const char* password = "";

// ThingsBoard
const char* tb_server = "demo.thingsboard.io";
const int tb_port = 1883;
const char* tb_token = "9mve05n4yndbhlbjbbft"; // Ganti dengan token device kamu

// MQTT Client
WiFiClient wifiClient;
PubSubClient client(wifiClient);

// LCD
LiquidCrystal_I2C lcd(0x27, 16, 2);

// Pin
#define RELAYPIN 2
#define LED_RED 5
#define LED_GREEN 18
#define LED_SEND 19

// GPS
TinyGPSPlus gps;
HardwareSerial gpsSerial(1); // Gunakan UART1

// Lokasi pusat untuk geofence
double latCenter = -24.466033;
double lngCenter = -52.840217;

unsigned long lastSend = 0;
const long interval = 10000; // Kirim data setiap 10 detik

void setup() {
  Serial.begin(115200);
  gpsSerial.begin(9600, SERIAL_8N1, 16, 17); // RX = 16, TX = 17

  pinMode(RELAYPIN, OUTPUT);
  pinMode(LED_RED, OUTPUT);
  pinMode(LED_GREEN, OUTPUT);
  pinMode(LED_SEND, OUTPUT);

  digitalWrite(RELAYPIN, HIGH); // simulasi mobil sudah jalan
  digitalWrite(LED_RED, LOW);
  digitalWrite(LED_GREEN, HIGH); // Indikator awal: GPS belum fix
  digitalWrite(LED_SEND, LOW);

  lcd.init();
  lcd.backlight();
  displayWelcome();
  wifiConnector();
  connectMQTT();
}

void loop() {
  while (gpsSerial.available() > 0) {
    char c = gpsSerial.read();
    gps.encode(c);
  }

  if (!client.connected()) {
    reconnectMQTT();
  }
  client.loop();

  // Cek jika waktunya kirim data
  if (millis() - lastSend > interval) {
    double latNow = gps.location.lat();
    double lngNow = gps.location.lng();

    if (gps.location.isValid()) {
      Serial.print("LAT: ");
      Serial.println(gps.location.lat(), 6);
      Serial.print("LNG: ");
      Serial.println(gps.location.lng(), 6);
      Serial.print("Updated: ");
      Serial.println(gps.location.isUpdated() ? "YES" : "NO");
      Serial.print("Satellites: ");
      Serial.println(gps.satellites.value());

      sendGPSData(latNow, lngNow);
      lastSend = millis();
      delay(10000);
    } else {
      Serial.println("[GPS] Tidak ada fix atau data tidak valid");
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("GPS: No Fix");
      digitalWrite(LED_RED, HIGH);
      delay(500);
      digitalWrite(LED_RED, LOW);
      delay(500);
      lcd.clear();
    }
  }

  // Geofence Check
  if (gps.location.isValid()) {
    double distance = calculateDistance(gps.location.lat(), gps.location.lng(), latCenter, lngCenter);
    Serial.print("[GPS] Jarak dari pusat: ");
    Serial.print(distance);
    Serial.println(" km");

    if (distance > 1.0) {
      digitalWrite(RELAYPIN, LOW); // mobil keadaan mati tidak bisa di hidupkan kembali ketika keluar area
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Keluar Area!");
      digitalWrite(LED_RED, HIGH);
      delay(500);
      digitalWrite(LED_RED, LOW);
      delay(500);
      lcd.clear();
    }
  }
}

// Fungsi Callback MQTT
void callback(char* topic, byte* payload, unsigned int length) {
  String message;
  for (int i = 0; i < length; i++) {
    message += (char)payload[i];
  }

  DynamicJsonDocument doc(200);
  deserializeJson(doc, message);

  String method = doc["method"];
  bool value = doc["params"];

  if (method == "setRelay") {
    digitalWrite(RELAYPIN, value ? HIGH : LOW);
    digitalWrite(LED_RED, value ? HIGH : LOW);
    digitalWrite(LED_GREEN, value ? LOW : HIGH);
    Serial.print("[MQTT] Relay set ke: ");
    Serial.println(value ? "ON" : "OFF");
  }
}

// Fungsi Jarak
double calculateDistance(double lat1, double lng1, double lat2, double lng2) {
  double R = 6371.0;
  double dLat = radians(lat2 - lat1);
  double dLng = radians(lng2 - lng1);
  double a = sin(dLat / 2) * sin(dLat / 2) + cos(radians(lat1)) * cos(radians(lat2)) * sin(dLng / 2) * sin(dLng / 2);
  double c = 2 * atan2(sqrt(a), sqrt(1 - a));
  return R * c;
}

// Tampilan Selamat Datang
void displayWelcome() {
  lcd.setCursor(0, 0);
  lcd.print("Welcome GPS Tracking");
  delay(2000);
  for (int i = 0; i < 16; i++) {
    lcd.scrollDisplayLeft();
    delay(300);
  }
  lcd.clear();
}

// Konek ke WiFi
void wifiConnector() {
  digitalWrite(LED_RED, HIGH);
  digitalWrite(LED_GREEN, LOW);

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Connecting WiFi");
  Serial.println("\n[WiFi] Connecting...");

  WiFi.begin(ssid, password);
  int retry = 0;
  while (WiFi.status() != WL_CONNECTED && retry < 20) {
    delay(500);
    Serial.print(".");
    retry++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\n[WiFi] Connected");
    lcd.setCursor(0, 0);
    lcd.print("WiFi Connected");
    digitalWrite(LED_RED, LOW);
    digitalWrite(LED_GREEN, HIGH);
    delay(1500);
    lcd.clear();
  } else {
    Serial.println("\n[WiFi] Failed");
    lcd.setCursor(0, 0);
    lcd.print("WiFi Failed");
    digitalWrite(LED_GREEN, LOW);
    digitalWrite(LED_RED, HIGH);
    while (true) delay(1000); // Stop
  }
}

// Konek ke MQTT
void connectMQTT() {
  client.setServer(tb_server, tb_port);
  client.setCallback(callback);
  reconnectMQTT();
}

// Reconnect MQTT
void reconnectMQTT() {
  digitalWrite(LED_RED, HIGH);
  digitalWrite(LED_SEND, LOW);
  digitalWrite(LED_GREEN, LOW);

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("MQTT Connecting");
  Serial.println("\n[MQTT] Connecting...");

  int retry = 0;
  while (!client.connect("ESP32_GPS", tb_token, NULL) && retry < 5) {
    Serial.print(".");
    retry++;
    delay(2000);
  }

  if (client.connected()) {
    digitalWrite(LED_RED, LOW);
    Serial.println("\n[MQTT] Connected");
    lcd.setCursor(0, 0);
    lcd.print("MQTT Connected");
    delay(1500);
    lcd.clear();
  } else {
    Serial.println("\n[MQTT] Connection Failed");
    lcd.setCursor(0, 0);
    lcd.print("MQTT Connect Fail");
    delay(2000);
    lcd.clear();
  }
}

// Kirim data GPS ke ThingsBoard
void sendGPSData(double lat, double lng) {
  StaticJsonDocument<256> doc;
  doc["latitude"] = lat;
  doc["longitude"] = lng;

  char buffer[512];
  size_t payloadSize = serializeJson(doc, buffer);

  if (client.connected()) {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Sending GPS Data");

    digitalWrite(LED_SEND, HIGH);
    client.publish("v1/devices/me/telemetry", buffer, payloadSize);
    digitalWrite(LED_SEND, LOW);

    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Data Sent");
    delay(1000);
    lcd.clear();
  } else {
    Serial.println("[MQTT] Tidak terhubung ❌");
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("MQTT Tidak Terhubung");
    delay(2000);
    lcd.clear();
  }
}