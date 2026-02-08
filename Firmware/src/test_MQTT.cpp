#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include "MAX30105.h"

MAX30105 particleSensor;

const char *kWifiSsid = "TP-LINK_26FE";
const char *kWifiPass = "67147526";

const char *kMqttHost = "192.168.0.102";
const uint16_t kMqttPort = 1883;
const char *kMqttTopic = "sensors/max30102/data";

WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);

// I2C pins for ESP32
#define SDA_PIN 32
#define SCL_PIN 33

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

  // Read sensor values
  long irValue = particleSensor.getIR();
  long redValue = particleSensor.getRed();
  long greenValue = particleSensor.getGreen();

  // Print data to Serial Monitor
  Serial.print("IR: ");
  Serial.print(irValue);
  Serial.print(" | Red: ");
  Serial.print(redValue);
  Serial.print(" | Green: ");
  Serial.println(greenValue);

  // Check if finger is detected (IR value > 50000 typically means finger present)
  if (irValue > 15000) {
    Serial.println(">> Finger detected!");
  } else {
    Serial.println(">> No finger detected");
  }

  char payload[128];
  snprintf(payload, sizeof(payload),
           "{\"ir\":%ld,\"red\":%ld,\"green\":%ld,\"finger\":%s}",
           irValue, redValue, greenValue, (irValue > 15000) ? "true" : "false");
  mqttClient.publish(kMqttTopic, payload);

  delay(500);  // Read every 100ms
}