#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>

// MUST define BEFORE including PubSubClient
#define MQTT_MAX_PACKET_SIZE 4096

#include <PubSubClient.h>
#include <MAX30105.h>
#include <heartRate.h>
#include <ThreeWire.h>
#include <RtcDS1302.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <string>

using std::string;

// WiFi & MQTT Configuration
const char *kWifiSsid = "TP-LINK_26FE";
const char *kWifiPass = "67147526";
const char *kMqttHost = "192.168.0.102";
const uint16_t kMqttPort = 1883;
const char *kMqttTopic = "sensors/max30102/data";

WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);

// I2C Configuration
#define SDA_PIN 21
#define SCL_PIN 22

// OLED display configuration
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
#define SCREEN_ADDRESS 0x3C

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// DS1302 RTC pins
static const uint8_t kClkPin = 18;
static const uint8_t kDatPin = 19;
static const uint8_t kRstPin = 23;

ThreeWire rtcWire(kDatPin, kClkPin, kRstPin);
RtcDS1302<ThreeWire> rtc(rtcWire);

// Alarm configuration
RtcDateTime alarmTime;
bool alarmTriggered = false;
static const uint8_t kAlarmHour = 7;
static const uint8_t kAlarmMinute = 30;
static const uint8_t kAlarmSecond = 0;

MAX30105 particleSensor;

// Heart rate calculation variables
const byte RATE_SIZE = 4;
byte rates[RATE_SIZE];
byte rateSpot = 0;
long lastBeat = 0;

int32_t spo2 = 0;
int8_t validSpo2 = 0;
int32_t heartRate = 0;
int8_t validHeartRate = 0;

static const uint32_t kFingerThreshold = 15000;

// MQTT Sampling Configuration
const uint16_t kSampleRateHz = 50;
const uint16_t kWindowSeconds = 10;
const uint16_t kSamplesPerWindow = kSampleRateHz * kWindowSeconds;
const uint16_t kSampleDelayMs = 1000 / kSampleRateHz;
bool stopSampling = false;

// Button/Joystick
const int buttonPadPin = 12;
const int joystickPinX = 32;
const int joyStickPinY = 27;
const int readingTolerance = 60;
int buttonpadReadingValue;
int joystickReadingX;
int joystickReadingY;

// Vibration motor & Speaker
static const uint8_t kVibePin = 25;
static const uint8_t kSpeakerPin = 26;
static const uint8_t kSpeakerChannel = 0;
static const uint16_t kSpeakerFreq = 2000;
static const uint16_t kSnoozeMinutes = 5;

// Menu state
enum class Screen {
  Dashboard,
  MainMenu,
  ViewAlarm,
  SetAlarm,
  SetAlarmType,
  SystemSettingsMenu,
  AlarmOutput,
  SystemTone,
  TestAlarmOutput,
  Alert
};

enum class Button {
  None, Up, Down, Left, Right, A, B, C, D
};

Screen currentScreen = Screen::Dashboard;
uint8_t mainMenuIndex = 0;
uint8_t alertMenuIndex = 0;
uint8_t systemMenuIndex = 0;
uint8_t alarmOutputIndex = 0;
uint8_t alarmToneIndex = 0;
uint8_t setAlarmFieldIndex = 0;
uint8_t setAlarmTypeIndex = 0;
bool alarmEnabled = true;
uint16_t alarmYear = 2026;
uint8_t alarmMonth = 1;
uint8_t alarmDay = 1;
uint8_t alarmHour = kAlarmHour;
uint8_t alarmMinute = kAlarmMinute;
uint8_t alarmType = 2;  // 0=Normal, 1=Max Sleep, 2=Smart Sleep (MQTT-based)
uint8_t alarmOutput = 0;
bool testAlarmPlaying = false;
uint32_t testAlarmStartMs = 0;
uint8_t lastTonePreview = 255;

// Vibration pattern state
struct VibeState {
  bool active = false;
  bool repeat = false;
  uint8_t pattern = 0;
  uint8_t step = 0;
  uint32_t nextMs = 0;
  bool on = false;
};
VibeState vibe;

// Forward declarations
static void StartVibrationPattern(uint8_t pattern, bool repeat);

// MQTT Callback
void mqttCallback(char* topic, byte* payload, unsigned int length) {
  Serial.print("\n🚨 MQTT message on: ");
  Serial.println(topic);
  
  String message = "";
  for (unsigned int i = 0; i < length; i++) {
    message += (char)payload[i];
  }
  
  if (String(topic) == "esp32/wake_alert") {
    Serial.println("🚨 WAKE UP ALERT FROM BACKEND! 🚨");
    // Trigger local alarm
    alarmTriggered = true;
    currentScreen = Screen::Alert;
    alertMenuIndex = 0;
    StartVibrationPattern(alarmToneIndex, true);
  }
  else if (String(topic) == "esp32/control") {
    if (message == "STOP_SAMPLING") {
      stopSampling = true;
      Serial.println("🛑 Backend requested STOP");
    }
  }
}

// Helper functions
static void PrintDateTime(const RtcDateTime &dt) {
  char buf[32];
  snprintf(buf, sizeof(buf), "%04u-%02u-%02u %02u:%02u:%02u",
           dt.Year(), dt.Month(), dt.Day(),
           dt.Hour(), dt.Minute(), dt.Second());
  Serial.println(buf);
}

static void SetAlarmFromFields() {
  alarmTime = RtcDateTime(alarmYear, alarmMonth, alarmDay,
                          alarmHour, alarmMinute, kAlarmSecond);
  alarmTriggered = false;
}

static void SetAlarmDefaultsFromNow(const RtcDateTime &now) {
  RtcDateTime next = RtcDateTime(now.Unix32Time() + 3600);
  alarmYear = 2026;
  alarmMonth = next.Month();
  alarmDay = next.Day();
  alarmHour = next.Hour();
  alarmMinute = next.Minute();
  SetAlarmFromFields();
}

std::string buttonpadReading(int reading) {
  string pressed = "nopress";
  if (abs(reading - 511) < readingTolerance) pressed = "up";
  else if (abs(reading - 1807) < readingTolerance) pressed = "down";
  else if (abs(reading - 1168) < readingTolerance) pressed = "left";
  else if (abs(reading - 2440) < readingTolerance) pressed = "right";
  return pressed;
}

std::string joystickReading(int x, int y) {
  string pressed = "nopress";
  if (x > 4000) pressed = "left";
  else if (x < 50) pressed = "right";
  else if (y > 4000) pressed = "up";
  else if (y < 50) pressed = "down";
  return pressed;
}

static Button ToButton(const std::string &value) {
  if (value == "up") return Button::Up;
  if (value == "down") return Button::Down;
  if (value == "left") return Button::Left;
  if (value == "right") return Button::Right;
  return Button::None;
}

static Button ReadButtonEvent(uint32_t nowMs) {
  static std::string lastValue = "nopress";
  static uint32_t lastChangeMs = 0;
  buttonpadReadingValue = analogRead(buttonPadPin);
  std::string valueRead = buttonpadReading(buttonpadReadingValue);
  if (valueRead == "nopress") {
    lastValue = valueRead;
    return Button::None;
  }
  if (valueRead != lastValue && (nowMs - lastChangeMs) > 180) {
    lastValue = valueRead;
    lastChangeMs = nowMs;
    return ToButton(valueRead);
  }
  return Button::None;
}

static Button ReadJoystickEvent(uint32_t nowMs) {
  static std::string lastValue = "nopress";
  static uint32_t lastChangeMs = 0;
  std::string valueRead = joystickReading(joystickReadingX, joystickReadingY);
  if (valueRead == "nopress") {
    lastValue = valueRead;
    return Button::None;
  }
  if (valueRead != lastValue && (nowMs - lastChangeMs) > 180) {
    lastValue = valueRead;
    lastChangeMs = nowMs;
    return ToButton(valueRead);
  }
  return Button::None;
}

static void StartVibrationPattern(uint8_t pattern, bool repeat) {
  vibe.active = true;
  vibe.repeat = repeat;
  vibe.pattern = pattern;
  vibe.step = 0;
  vibe.on = true;
  vibe.nextMs = millis();
  digitalWrite(kVibePin, HIGH);
}

static void StopVibration() {
  vibe.active = false;
  digitalWrite(kVibePin, LOW);
}

static void UpdateVibration(uint32_t nowMs) {
  if (!vibe.active) return;
  const uint16_t pattern0[] = {200, 200, 200, 800};
  const uint16_t pattern1[] = {400, 200, 400, 200, 400, 800};
  const uint16_t pattern2[] = {400, 400, 400, 800};
  const uint16_t *pattern = pattern0;
  uint8_t length = 4;
  if (vibe.pattern == 1) { pattern = pattern1; length = 6; }
  else if (vibe.pattern == 2) { pattern = pattern2; length = 2; }
  if (nowMs < vibe.nextMs) return;
  vibe.on = !vibe.on;
  digitalWrite(kVibePin, vibe.on ? HIGH : LOW);
  uint16_t duration = pattern[vibe.step];
  vibe.nextMs = nowMs + duration;
  vibe.step++;
  if (vibe.step >= length) {
    if (vibe.repeat) {
      vibe.step = 0;
      vibe.on = true;
      digitalWrite(kVibePin, HIGH);
      vibe.nextMs = nowMs + pattern[0];
    } else {
      StopVibration();
    }
  }
}

static void DrawCenteredText(const char *text, int y, uint8_t size) {
  display.setTextSize(size);
  int16_t x1, y1;
  uint16_t w, h;
  display.getTextBounds(text, 0, 0, &x1, &y1, &w, &h);
  int x = (SCREEN_WIDTH - w) / 2;
  display.setCursor(x, y);
  display.println(text);
}

void setup() {
  Serial.begin(115200);
  delay(200);

  // Initialize I2C
  Wire.begin(SDA_PIN, SCL_PIN);

  // Initialize OLED
  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println("SSD1306 allocation failed");
  } else {
    Serial.println("OLED display initialized!");
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.println("Initializing...");
    display.display();
  }

  // Initialize peripherals
  pinMode(buttonPadPin, INPUT);
  pinMode(kVibePin, OUTPUT);
  digitalWrite(kVibePin, LOW);
  ledcSetup(kSpeakerChannel, kSpeakerFreq, 8);
  ledcAttachPin(kSpeakerPin, kSpeakerChannel);
  ledcWriteTone(kSpeakerChannel, 0);
  pinMode(joystickPinX, INPUT);
  pinMode(joyStickPinY, INPUT);
  analogSetAttenuation(ADC_11db);
  analogReadResolution(12);

  // Initialize MAX30102
  if (!particleSensor.begin(Wire, I2C_SPEED_STANDARD)) {
    Serial.println("MAX30102 not found!");
    while (1);
  }
  Serial.println("MAX30102 initialized!");
  particleSensor.setup();
  particleSensor.setPulseAmplitudeRed(0x0A);
  particleSensor.setPulseAmplitudeGreen(0);
  particleSensor.setPulseAmplitudeIR(0x0A);

  // Initialize RTC
  rtc.Begin();
  if (!rtc.GetIsRunning()) {
    rtc.SetIsRunning(true);
  }
  if (!rtc.IsDateTimeValid()) {
    rtc.SetDateTime(RtcDateTime(__DATE__, __TIME__));
  }
  RtcDateTime now = rtc.GetDateTime();
  if (rtc.IsDateTimeValid()) {
    SetAlarmDefaultsFromNow(now);
  }
  Serial.print("RTC initialized: ");
  PrintDateTime(now);

  // Connect to WiFi
  display.clearDisplay();
  display.setCursor(0, 0);
  display.println("Connecting WiFi...");
  display.display();
  
  WiFi.mode(WIFI_STA);
  WiFi.begin(kWifiSsid, kWifiPass);
  while (WiFi.status() != WL_CONNECTED) {
    delay(250);
    Serial.print(".");
  }
  Serial.print("\nWiFi connected: ");
  Serial.println(WiFi.localIP());

  // Setup MQTT
  mqttClient.setServer(kMqttHost, kMqttPort);
  mqttClient.setBufferSize(4096);
  mqttClient.setCallback(mqttCallback);
  
  display.clearDisplay();
  display.setCursor(0, 0);
  display.println("Ready!");
  display.display();
  delay(1000);
}

void loop() {
  static uint32_t lastVitalsMs = 0;
  static uint32_t lastDisplayMs = 0;
  static uint16_t sampleIndex = 0;
  static long irSamples[kSamplesPerWindow];
  
  const uint32_t nowMs = millis();

  // Handle MQTT connection
  if (!mqttClient.connected()) {
    String clientId = "esp32-alarm-" + String((uint32_t)ESP.getEfuseMac(), HEX);
    if (mqttClient.connect(clientId.c_str())) {
      Serial.println("MQTT connected");
      mqttClient.subscribe("esp32/wake_alert");
      mqttClient.subscribe("esp32/control");
    } else {
      delay(2000);
    }
  }
  mqttClient.loop();

  // Read joystick
  int x_sum = 0, y_sum = 0;
  for (int i = 0; i < 8; i++) {
    analogRead(joystickPinX); delayMicroseconds(250);
    x_sum += analogRead(joystickPinX); delayMicroseconds(100);
    analogRead(joyStickPinY); delayMicroseconds(250);
    y_sum += analogRead(joyStickPinY); delayMicroseconds(100);
  }
  joystickReadingX = x_sum / 8;
  joystickReadingY = y_sum / 8;

  UpdateVibration(nowMs);
  Button button = ReadJoystickEvent(nowMs);

  RtcDateTime now = rtc.GetDateTime();
  const bool timeValid = rtc.IsDateTimeValid();

  // Check for manual/timed alarm trigger (Normal mode only)
  if (timeValid && alarmEnabled && !alarmTriggered && alarmType == 0 && now >= alarmTime) {
    alarmTriggered = true;
    currentScreen = Screen::Alert;
    alertMenuIndex = 0;
    StartVibrationPattern(alarmToneIndex, true);
  }

  // Menu navigation logic (simplified from main.cpp - keeping only essential screens)
  switch (currentScreen) {
    case Screen::Dashboard:
      if (button == Button::Right) {
        currentScreen = Screen::MainMenu;
        mainMenuIndex = 0;
      }
      break;
    case Screen::MainMenu:
      if (button == Button::Up && mainMenuIndex > 0) mainMenuIndex--;
      else if (button == Button::Down && mainMenuIndex < 3) mainMenuIndex++;
      else if (button == Button::Right) {
        if (mainMenuIndex == 0) {
          alarmEnabled = !alarmEnabled;
          if (!alarmEnabled) {
            alarmTriggered = false;
            StopVibration();
          }
        } else if (mainMenuIndex == 1) {
          currentScreen = Screen::ViewAlarm;
        } else if (mainMenuIndex == 2) {
          if (timeValid) SetAlarmDefaultsFromNow(now);
          setAlarmFieldIndex = 0;
          currentScreen = Screen::SetAlarm;
        } else if (mainMenuIndex == 3) {
          setAlarmTypeIndex = alarmType;
          currentScreen = Screen::SetAlarmType;
        }
      } else if (button == Button::Left) {
        currentScreen = Screen::Dashboard;
      }
      break;
    case Screen::ViewAlarm:
      if (button == Button::Left) currentScreen = Screen::MainMenu;
      break;
    case Screen::SetAlarmType:
      if (button == Button::Up && setAlarmTypeIndex > 0) setAlarmTypeIndex--;
      else if (button == Button::Down && setAlarmTypeIndex < 2) setAlarmTypeIndex++;
      else if (button == Button::Right) {
        alarmType = setAlarmTypeIndex;
        currentScreen = Screen::MainMenu;
      } else if (button == Button::Left) {
        currentScreen = Screen::MainMenu;
      }
      break;
    case Screen::Alert:
      if (button == Button::Up || button == Button::Down) {
        alertMenuIndex = (alertMenuIndex == 0) ? 1 : 0;
      } else if (button == Button::Right) {
        if (alertMenuIndex == 0) {
          if (timeValid) {
            alarmTime = RtcDateTime(now.Unix32Time() + (kSnoozeMinutes * 60));
          }
        } else {
          if (timeValid && now >= alarmTime) {
            alarmTime = RtcDateTime(alarmTime.Unix32Time() + 86400);
          }
        }
        alarmTriggered = false;
        stopSampling = false;  // Resume sampling after dismissing alert
        StopVibration();
        currentScreen = Screen::Dashboard;
      }
      break;
  }

  // Sensor reading & heartbeat detection
  long irValue = particleSensor.getIR();
  if (irValue > kFingerThreshold) {
    if (checkForBeat(irValue)) {
      long delta = millis() - lastBeat;
      lastBeat = millis();
      int beatsPerMinute = 60 / (delta / 1000.0);
      if (beatsPerMinute < 255 && beatsPerMinute > 20) {
        rates[rateSpot++] = (byte)beatsPerMinute;
        rateSpot %= RATE_SIZE;
        int beatAvg = 0;
        for (byte x = 0; x < RATE_SIZE; x++) beatAvg += rates[x];
        beatAvg /= RATE_SIZE;
        heartRate = beatAvg;
        validHeartRate = (heartRate > 20 && heartRate < 200) ? 1 : 0;
        float ratio = (float)particleSensor.getRed() / (float)irValue;
        if (ratio > 0.4 && ratio < 2.0) {
          spo2 = (int32_t)(110 - 25 * ratio);
          if (spo2 > 100) spo2 = 100;
          if (spo2 < 80) spo2 = 80;
          validSpo2 = 1;
        } else validSpo2 = 0;
      }
    }
  } else {
    validHeartRate = 0;
    validSpo2 = 0;
  }

  // MQTT Sampling (if Smart Sleep mode enabled and not stopped)
  if (alarmType == 2 && !stopSampling) {
    irSamples[sampleIndex++] = irValue;
    if (sampleIndex >= kSamplesPerWindow) {
      String payload;
      payload.reserve(3600);
      payload += "{\"ir\":[";
      for (uint16_t i = 0; i < kSamplesPerWindow; ++i) {
        payload += String(irSamples[i]);
        if (i + 1 < kSamplesPerWindow) payload += ',';
      }
      payload += "],\"sample_rate\":";
      payload += String(kSampleRateHz);
      payload += '}';
      if (mqttClient.publish(kMqttTopic, payload.c_str())) {
        Serial.println("✅ Published 500-sample window");
      }
      sampleIndex = 0;
    }
    delay(kSampleDelayMs);
  }

  // Print vitals every second
  if (nowMs - lastVitalsMs >= 1000) {
    lastVitalsMs = nowMs;
    Serial.print("HR: ");
    Serial.print(validHeartRate ? String(heartRate) : "na");
    Serial.print(" | SpO2: ");
    Serial.println(validSpo2 ? String(spo2) : "na");
  }

  // Update display
  uint32_t displayIntervalMs = (currentScreen == Screen::Dashboard) ? 1000 : 200;
  if (nowMs - lastDisplayMs >= displayIntervalMs) {
    lastDisplayMs = nowMs;
    display.clearDisplay();

    if (currentScreen == Screen::Dashboard) {
      if (timeValid) {
        display.setTextSize(2);
        char timeStr[10];
        snprintf(timeStr, sizeof(timeStr), "%02u:%02u:%02u",
                 now.Hour(), now.Minute(), now.Second());
        DrawCenteredText(timeStr, 8, 2);
        display.setTextSize(1);
        char dateStr[12];
        snprintf(dateStr, sizeof(dateStr), "%04u-%02u-%02u",
                 now.Year(), now.Month(), now.Day());
        DrawCenteredText(dateStr, 30, 1);
      }
      display.setTextSize(1);
      char vitalStr[20];
      if (validHeartRate && validSpo2) {
        snprintf(vitalStr, sizeof(vitalStr), "HR:%d  SpO2:%d %%", heartRate, spo2);
      } else if (validHeartRate) {
        snprintf(vitalStr, sizeof(vitalStr), "HR:%d  SpO2:--", heartRate);
      } else {
        snprintf(vitalStr, sizeof(vitalStr), "HR:--  SpO2:--");
      }
      DrawCenteredText(vitalStr, 50, 1);
    } else if (currentScreen == Screen::MainMenu) {
      display.setTextSize(1);
      display.setCursor(0, 0);
      display.println("Main Menu");
      display.setCursor(0, 16);
      display.print(mainMenuIndex == 0 ? "> " : "  ");
      display.print("Alarm: ");
      display.println(alarmEnabled ? "On" : "Off");
      display.print(mainMenuIndex == 1 ? "> " : "  ");
      display.println("View Alarm");
      display.print(mainMenuIndex == 2 ? "> " : "  ");
      display.println("Set Alarm");
      display.print(mainMenuIndex == 3 ? "> " : "  ");
      display.println("Set Alarm Type");
    } else if (currentScreen == Screen::ViewAlarm) {
      display.setTextSize(1);
      display.setCursor(0, 0);
      display.println("Current Alarm");
      char alarmStr[12];
      snprintf(alarmStr, sizeof(alarmStr), "%02u:%02u", 
               alarmTime.Hour(), alarmTime.Minute());
      DrawCenteredText(alarmStr, 25, 2);
      const char *typeStr;
      if (alarmType == 0) typeStr = "Normal Alarm";
      else if (alarmType == 1) typeStr = "Max Sleep Mode";
      else typeStr = "Smart Sleep Mode";
      DrawCenteredText(typeStr, 45, 1);
    } else if (currentScreen == Screen::SetAlarmType) {
      display.setTextSize(1);
      display.setCursor(0, 0);
      display.println("Set Alarm Type");
      display.setCursor(0, 16);
      display.print(setAlarmTypeIndex == 0 ? "> " : "  ");
      display.println("Normal Alarm");
      display.print(setAlarmTypeIndex == 1 ? "> " : "  ");
      display.println("Max Sleep Mode");
      display.print(setAlarmTypeIndex == 2 ? "> " : "  ");
      display.println("Smart Sleep Mode");
      display.setCursor(0, 45);
      if (setAlarmTypeIndex == 0) {
        display.println("Sounds at set time");
      } else if (setAlarmTypeIndex == 1) {
        display.println("Local HR tracking");
      } else {
        display.println("Backend AI tracking");
      }
    } else if (currentScreen == Screen::Alert) {
      display.setTextSize(2);
      DrawCenteredText("ALARM!", 10, 2);
      display.setTextSize(1);
      display.setCursor(10, 35);
      display.print(alertMenuIndex == 0 ? "> " : "  ");
      display.println("Snooze 5 min");
      display.setCursor(10, 45);
      display.print(alertMenuIndex == 1 ? "> " : "  ");
      display.println("Dismiss");
    }

    display.display();
  }
}
