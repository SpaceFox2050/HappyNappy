#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>

// MUST define buffer size BEFORE including PubSubClient
// 500 samples * 6 chars + overhead = ~3524 bytes, use 4096 for safety
#define MQTT_MAX_PACKET_SIZE 4096

#include <PubSubClient.h>
#include "MAX30105.h"

MAX30105 particleSensor;

const char *kWifiSsid = "TP-LINK_26FE";
const char *kWifiPass = "67147526";

const char *kMqttHost = "192.168.0.102"; //192.168.0.102
const uint16_t kMqttPort = 1883;
const char *kMqttTopic = "sensors/max30102/data";

WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);

// I2C pins for ESP32
#define SDA_PIN 32
#define SCL_PIN 33

// Sampling configuration
const uint16_t kSampleRateHz = 50;  // 50 Hz => 20 ms per sample
const uint16_t kWindowSeconds = 10;  // Collect for 10 seconds
const uint16_t kSamplesPerWindow = kSampleRateHz * kWindowSeconds;  // 500 samples
const uint16_t kSampleDelayMs = 1000 / kSampleRateHz;

void setup() {
  delay(2000);  // Wait for serial monitor to connect
  Serial.begin(115200);
  delay(1000);

  Serial.println("\n\n");
  Serial.println("=====================================");
  Serial.println("Initializing MAX30102 Sensor...");

  WiFi.mode(WIFI_STA);
  WiFi.begin(kWifiSsid, kWifiPass);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(250);
    Serial.print(".");
  }
  Serial.println();
  Serial.print("WiFi connected, IP: ");
  Serial.println(WiFi.localIP());

  mqttClient.setServer(kMqttHost, kMqttPort);
  mqttClient.setBufferSize(4096);  // Explicitly set buffer size

  // Initialize I2C
  Wire.begin(SDA_PIN, SCL_PIN);

  // Initialize sensor
  if (!particleSensor.begin(Wire, I2C_SPEED_STANDARD)) {
    Serial.println("MAX30102 was not found. Please check wiring/power.");
    while (1);
  }

  Serial.println("MAX30102 Sensor Initialized Successfully!");

  // Configure sensor
  particleSensor.setup();  // Default configuration
  particleSensor.setPulseAmplitudeRed(0x0A);   // Turn on Red LED
  particleSensor.setPulseAmplitudeGreen(0);    // Turn off Green LED
  particleSensor.setPulseAmplitudeIR(0x0A);    // Turn on IR LED

  Serial.println("Sensor configured. Ready to read...");
  Serial.println("Place your finger on the sensor...");

  Serial.print("Free heap: ");
  Serial.println(ESP.getFreeHeap());
}

void loop() {
  if (!mqttClient.connected()) {
    while (!mqttClient.connected()) {
      String clientId = "esp32-max30102-" + String((uint32_t)ESP.getEfuseMac(), HEX);
      if (mqttClient.connect(clientId.c_str())) {
        Serial.println("MQTT connected");
      } else {
        Serial.print("MQTT connect failed, rc=");
        Serial.print(mqttClient.state());
        Serial.println(" retrying in 2s");
        delay(2000);
      }
    }
  }
  mqttClient.loop();

  // Collect samples for HeartPy analysis
  static uint16_t sampleIndex = 0;
  static long irSamples[kSamplesPerWindow];

  long irValue = particleSensor.getIR();
  irSamples[sampleIndex++] = irValue;

  if (sampleIndex >= kSamplesPerWindow) {
    String payload;
    payload.reserve(3600);
    payload += "{\"ir\":[";
    for (uint16_t i = 0; i < kSamplesPerWindow; ++i) {
      payload += String(irSamples[i]);
      if (i + 1 < kSamplesPerWindow) {
        payload += ',';
      }
    }
    payload += "],\"sample_rate\":";
    payload += String(kSampleRateHz);
    payload += '}';

    bool success = mqttClient.publish(kMqttTopic, payload.c_str());
    
    if (success) {
      Serial.print("✅ Published ");
      Serial.print(kSamplesPerWindow);
      Serial.println("-sample window successfully");
    } else {
      Serial.println("❌ Publish FAILED");
    }

    sampleIndex = 0;
  }

  delay(kSampleDelayMs);
}