#include <Arduino.h>
#include <Wire.h>
#include <Wifi.h>
#include <MAX30105.h>
#include <heartRate.h>
#include <ThreeWire.h>
#include <RtcDS1302.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <PubSubClient.h>
#include <string>
#include <unordered_map>

using std::unordered_map;
using std::string;

#define SDA_PIN 21
#define SCL_PIN 22
#define MOTOR_PIN 25
#define MQTT_MAX_PACKET_SIZE 4096

// OLED display configuration
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
#define SCREEN_ADDRESS 0x3C

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);


const char *kWifiSsid = "TP-LINK_26FE";
const char *kWifiPass = "67147526";

const char *kMqttHost = "192.168.0.105"; //192.168.0.102
const uint16_t kMqttPort = 1883;
const char *kMqttTopic = "sensors/max30102/data";

WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);

bool stopSampling = false;  // Set to true when backend sends STOP_SAMPLING

// Sampling configuration
const uint16_t kSampleRateHz = 50;  // 50 Hz => 20 ms per sample
const uint16_t kWindowSeconds = 10;  // Collect for 10 seconds
const uint16_t kSamplesPerWindow = kSampleRateHz * kWindowSeconds;  // 500 samples
const uint16_t kSampleDelayMs = 1000 / kSampleRateHz;



// DS1302 pins (ESP32 GPIOs)
static const uint8_t kClkPin = 18;
static const uint8_t kDatPin = 19;
static const uint8_t kRstPin = 23;

// Alarm time (24-hour)
static const uint8_t kAlarmHour = 7;
static const uint8_t kAlarmMinute = 30;
static const uint8_t kAlarmSecond = 0;

ThreeWire rtcWire(kDatPin, kClkPin, kRstPin); // DAT, CLK, RST
RtcDS1302<ThreeWire> rtc(rtcWire);

RtcDateTime alarmTime;
bool alarmTriggered = false;

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

static const uint32_t kFingerThreshold = 15000;  // IR value threshold for finger detection


//buttonpad readings
int buttonpadReadingValue;

//joystick info (using separated ADC1 channels)
const int joystickPinX = 32;  // ADC1_CH4
const int joyStickPinY = 35;  // ADC1_CH5 (ADC2 unavailable due to WiFi)
int joystickReadingX;
int joystickReadingY;


// Vibration motor
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
  SmartAlarm,
  SmartAlarmInfo,
  SystemSettingsMenu,
  AlarmOutput,
  SystemTone,
  TestAlarmOutput,
  Alert
};

enum class Button {
  None,
  Up,
  Down,
  Left,
  Right,
  A,
  B,
  C,
  D
};

Screen currentScreen = Screen::Dashboard;
uint8_t mainMenuIndex = 0;
uint8_t alertMenuIndex = 0;
uint8_t systemMenuIndex = 0;
uint8_t smartAlarmMenuIndex = 0;
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
uint8_t alarmType = 0;        // 0 = Normal, 1 = Max Sleep, 2 = Smart Sleep
uint8_t alarmOutput = 0;      // 0 = Vibration, 1 = Speaker, 2 = Both
bool showSmartAlarmInfo = false;
bool showAlarmTypeInfo = false;
bool testAlarmPlaying = false;
uint32_t testAlarmStartMs = 0;

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
uint8_t lastTonePreview = 255;

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

// static RtcDateTime AddSeconds(const RtcDateTime &dt, uint32_t seconds) {
//   return RtcDateTime(now.Unix32Time() + seconds);
// }

static void SetAlarmDefaultsFromNow(const RtcDateTime &now) {
  RtcDateTime next = RtcDateTime(now.Unix32Time() + 3600);
  alarmYear = 2026;
  alarmMonth = next.Month();
  alarmDay = next.Day();
  alarmHour = next.Hour();
  alarmMinute = next.Minute();
  SetAlarmFromFields();
}


//reading the joystick for menu control
std::string joystickReading(int x, int y) {
  string pressed = "nopress";
  
  // Only one direction at a time, check in priority order
  if (x > 4000) {
    pressed = "left";
  } else if (x < 50) {
    pressed = "right";
  } else if (y > 4000) {
    pressed = "up";
  } else if (y < 50) {
    pressed = "down";
  }
  
  return pressed;
}

static Button ToButton(const std::string &value) {
  if (value == "up") return Button::Up;
  if (value == "down") return Button::Down;
  if (value == "left") return Button::Left;
  if (value == "right") return Button::Right;
  if (value == "a") return Button::A;
  if (value == "b") return Button::B;
  if (value == "c") return Button::C;
  if (value == "d") return Button::D;
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
  if (vibe.pattern == 1) {
    pattern = pattern1;
    length = 6;
  } else if (vibe.pattern == 2) {
    pattern = pattern2;
    length = 2;
  }

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

void buzzMotor() {
    alarmTriggered = true;
    currentScreen = Screen::Alert;
    alertMenuIndex = 0;
    StartVibrationPattern(alarmToneIndex, true);
}

void mqttCallback(char* topic, byte* payload, unsigned int length) {
  Serial.print("\n🚨 Received message on topic: ");
  Serial.println(topic);
  
  String message = "";
  for (unsigned int i = 0; i < length; i++) {
    message += (char)payload[i];
  }
  
  Serial.print("Message: ");
  Serial.println(message);
  
  if (String(topic) == "esp32/wake_alert") {
    Serial.println("\n=====================================");
    Serial.println("🚨 WAKE UP ALERT! 🚨");
    Serial.println("=====================================");
    buzzMotor();  // Activate vibration motor
  }
  else if (String(topic) == "esp32/control") {
    if (message == "STOP_SAMPLING") {
      stopSampling = true;
      Serial.println("🛑 Received STOP command - stopping data collection");
    }
  }
}

void setup() {
  Serial.begin(115200);
  delay(200);

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
  mqttClient.setCallback(mqttCallback);  // Set callback for incoming messages

  // Initialize I2C
  Wire.begin(SDA_PIN, SCL_PIN);


  // Initialize OLED display
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


  //initialize vibration motor
  pinMode(kVibePin, OUTPUT);
  digitalWrite(kVibePin, LOW);

  //initialize speaker
  ledcSetup(kSpeakerChannel, kSpeakerFreq, 8);
  ledcAttachPin(kSpeakerPin, kSpeakerChannel);
  ledcWriteTone(kSpeakerChannel, 0);

  //initialize joystick
  pinMode(joystickPinX, INPUT);
  pinMode(joyStickPinY, INPUT);
  
  // Configure ADC for joystick pins
  analogSetAttenuation(ADC_11db);  // Full 0-3.3V range
  analogReadResolution(12);         // 12-bit resolution (0-4095)


  //initialize max30102 heart rate sensor
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

  rtc.Begin();

  if (!rtc.GetIsRunning()) {
    rtc.SetIsRunning(true);
  }

  if (!rtc.IsDateTimeValid()) {
    // RTC lost confidence: set to compile time
    rtc.SetDateTime(RtcDateTime(__DATE__, __TIME__));
  }

  RtcDateTime now = rtc.GetDateTime();
  if (rtc.IsDateTimeValid()) {
    SetAlarmDefaultsFromNow(now);
  }

  
  Serial.println("RTC initialized. Current time:");
  PrintDateTime(now);
}

void loop() {
  static uint32_t lastVitalsMs = 0;
  static uint32_t lastDisplayMs = 0;
  static uint32_t lastTimeMs = 0;
  const uint32_t nowMs = millis();

  // Read ADC with multiple samples and validation
  int x_sum = 0;
  int y_sum = 0;
  const int samples = 8;
  
  for (int i = 0; i < samples; i++) {
    // Read X axis with settling
    analogRead(joystickPinX);  // Dummy read
    delayMicroseconds(250);
    x_sum += analogRead(joystickPinX);
    delayMicroseconds(100);
    
    // Read Y axis with settling  
    analogRead(joyStickPinY);  // Dummy read
    delayMicroseconds(250);
    y_sum += analogRead(joyStickPinY);
    delayMicroseconds(100);
  }
  
  int raw_x = x_sum / samples;
  int raw_y = y_sum / samples;
  
  // Filter out ADC crosstalk - when one axis is at extreme, other gets pulled up
  const int nearMax = 3800;
  const int crossTalkThreshold = 3000;  // If other axis also above this, likely crosstalk
  
  // If X is at max but Y is suspiciously high (crosstalk), correct Y
  if (raw_x > nearMax && raw_y > crossTalkThreshold && raw_y < nearMax) {
    raw_y = 2048;  // Reset Y to center
  }
  // If Y is at max but X is suspiciously high (crosstalk), correct X
  else if (raw_y > nearMax && raw_x > crossTalkThreshold && raw_x < nearMax) {
    raw_x = 2048;  // Reset X to center
  }
  // If both are at max, keep the more extreme one
  else if (raw_x > nearMax && raw_y > nearMax) {
    if (raw_x > raw_y) {
      raw_y = 2048;
    } else {
      raw_x = 2048;
    }
  }
  
  joystickReadingX = raw_x;
  joystickReadingY = raw_y;
  
  // Serial.print("X: ");
  // Serial.print(joystickReadingX);
  // Serial.print(" Y: ");
  // Serial.println(joystickReadingY);
  //delay(200);

  UpdateVibration(nowMs);
  Button button = ReadJoystickEvent(nowMs);

  RtcDateTime now = rtc.GetDateTime();
  const bool timeValid = rtc.IsDateTimeValid();

  if (timeValid && alarmEnabled && !alarmTriggered && now >= alarmTime) {
    alarmTriggered = true;
    currentScreen = Screen::Alert;
    alertMenuIndex = 0;
    StartVibrationPattern(alarmToneIndex, true);
  }

  switch (currentScreen) {
    case Screen::Dashboard:
      if (button == Button::Right) {
        currentScreen = Screen::MainMenu;
        mainMenuIndex = 0;
      }
      break;
    case Screen::MainMenu:
      if (button == Button::Up && mainMenuIndex > 0) {
        mainMenuIndex--;
      } else if (button == Button::Down && mainMenuIndex < 4) {
        mainMenuIndex++;
      } else if (button == Button::Right) {
        if (mainMenuIndex == 0) {
          alarmEnabled = !alarmEnabled;
          if (!alarmEnabled) {
            alarmTriggered = false;
            StopVibration();
          }
        } else if (mainMenuIndex == 1) {
          currentScreen = Screen::ViewAlarm;
        } else if (mainMenuIndex == 2) {
          if (timeValid) {
            SetAlarmDefaultsFromNow(now);
          }
          setAlarmFieldIndex = 0;
          currentScreen = Screen::SetAlarm;
        } else if (mainMenuIndex == 3) {
          setAlarmTypeIndex = alarmType;
          currentScreen = Screen::SetAlarmType;
        } else if (mainMenuIndex == 4) {
          systemMenuIndex = 0;
          currentScreen = Screen::SystemSettingsMenu;
        }
      } else if (button == Button::Left) {
        currentScreen = Screen::Dashboard;
      }
      break;
    case Screen::ViewAlarm:
      if (button == Button::Left) {
        currentScreen = Screen::MainMenu;
      }
      break;
    case Screen::SetAlarm:
      if (button == Button::Up) {
        if (setAlarmFieldIndex == 0) {
          alarmYear = (alarmYear < 2099) ? alarmYear + 1 : 2000;
        } else if (setAlarmFieldIndex == 1) {
          alarmMonth = (alarmMonth % 12) + 1;
        } else if (setAlarmFieldIndex == 2) {
          alarmDay = (alarmDay % 31) + 1;
        } else if (setAlarmFieldIndex == 3) {
          alarmHour = (alarmHour + 1) % 24;
        } else {
          alarmMinute = (alarmMinute + 1) % 60;
        }
      } else if (button == Button::Down) {
        if (setAlarmFieldIndex == 0) {
          alarmYear = (alarmYear > 2000) ? alarmYear - 1 : 2099;
        } else if (setAlarmFieldIndex == 1) {
          alarmMonth = (alarmMonth == 1) ? 12 : alarmMonth - 1;
        } else if (setAlarmFieldIndex == 2) {
          alarmDay = (alarmDay == 1) ? 31 : alarmDay - 1;
        } else if (setAlarmFieldIndex == 3) {
          alarmHour = (alarmHour == 0) ? 23 : alarmHour - 1;
        } else {
          alarmMinute = (alarmMinute == 0) ? 59 : alarmMinute - 1;
        }
      } else if (button == Button::Right) {
        if (setAlarmFieldIndex < 4) {
          setAlarmFieldIndex++;
        } else {
          SetAlarmFromFields();
          currentScreen = Screen::MainMenu;
        }
      } else if (button == Button::Left) {
        SetAlarmFromFields();
        currentScreen = Screen::MainMenu;
      }
      break;
    case Screen::SetAlarmType:
      if (button == Button::Up && setAlarmTypeIndex > 0) {
        setAlarmTypeIndex--;
        showAlarmTypeInfo = false;
      } else if (button == Button::Down && setAlarmTypeIndex < 2) {
        setAlarmTypeIndex++;
        showAlarmTypeInfo = false;
      } else if (button == Button::Right) {
        if (showAlarmTypeInfo) {
          alarmType = setAlarmTypeIndex;
          currentScreen = Screen::MainMenu;
        } else {
          showAlarmTypeInfo = true;
        }
      } else if (button == Button::Left) {
        if (showAlarmTypeInfo) {
          showAlarmTypeInfo = false;
        } else {
          currentScreen = Screen::MainMenu;
        }
        currentScreen = Screen::MainMenu;
      }
      break;
    case Screen::SmartAlarm:
      if (button == Button::Up && smartAlarmMenuIndex > 0) {
        smartAlarmMenuIndex--;
        showSmartAlarmInfo = false;
      } else if (button == Button::Down && smartAlarmMenuIndex < 1) {
        smartAlarmMenuIndex++;
        showSmartAlarmInfo = false;
      } else if (button == Button::Right) {
        if (showSmartAlarmInfo) {
          showSmartAlarmInfo = false;
        } else {
          showSmartAlarmInfo = true;
        }
      } else if (button == Button::Left) {
        if (showSmartAlarmInfo) {
          showSmartAlarmInfo = false;
        } else {
          currentScreen = Screen::MainMenu;
        }
      }
      break;
    case Screen::SmartAlarmInfo:
      if (button == Button::Left) {
        currentScreen = Screen::SmartAlarm;
      }
      break;
    case Screen::SystemSettingsMenu:
      if (button == Button::Up && systemMenuIndex > 0) {
        systemMenuIndex--;
      } else if (button == Button::Down && systemMenuIndex < 2) {
        systemMenuIndex++;
      } else if (button == Button::Right) {
        if (systemMenuIndex == 0) {
          currentScreen = Screen::SystemTone;
          lastTonePreview = 255;
        } else if (systemMenuIndex == 1) {
          currentScreen = Screen::AlarmOutput;
          alarmOutputIndex = alarmOutput;
        } else if (systemMenuIndex == 2) {
          currentScreen = Screen::TestAlarmOutput;
          testAlarmPlaying = false;
        }
      } else if (button == Button::Left) {
        currentScreen = Screen::MainMenu;
      }
      break;
    case Screen::AlarmOutput:
      if (button == Button::Up && alarmOutputIndex > 0) {
        alarmOutputIndex--;
      } else if (button == Button::Down && alarmOutputIndex < 2) {
        alarmOutputIndex++;
      } else if (button == Button::Right) {
        alarmOutput = alarmOutputIndex;
        currentScreen = Screen::SystemSettingsMenu;
      } else if (button == Button::Left) {
        currentScreen = Screen::SystemSettingsMenu;
      }
      break;
    case Screen::SystemTone:
      if (button == Button::Up && alarmToneIndex > 0) {
        alarmToneIndex--;
      } else if (button == Button::Down && alarmToneIndex < 2) {
        alarmToneIndex++;
      } else if (button == Button::Right || button == Button::Left) {
        currentScreen = Screen::SystemSettingsMenu;
      }
      if (alarmToneIndex != lastTonePreview) {
        lastTonePreview = alarmToneIndex;
        StartVibrationPattern(alarmToneIndex, false);
      }
      break;
    case Screen::TestAlarmOutput:
      if (button == Button::Right && !testAlarmPlaying) {
        testAlarmPlaying = true;
        testAlarmStartMs = nowMs;
        if (alarmOutput == 0 || alarmOutput == 2) {
          StartVibrationPattern(alarmToneIndex, false);
        }
        if (alarmOutput == 1 || alarmOutput == 2) {
          ledcWriteTone(kSpeakerChannel, kSpeakerFreq);
        }
      } else if (button == Button::Left) {
        testAlarmPlaying = false;
        StopVibration();
        ledcWriteTone(kSpeakerChannel, 0);
        currentScreen = Screen::SystemSettingsMenu;
      }
      if (testAlarmPlaying && (alarmOutput == 1 || alarmOutput == 2)) {
        if (nowMs - testAlarmStartMs < 500) {
          ledcWriteTone(kSpeakerChannel, kSpeakerFreq);
        } else {
          ledcWriteTone(kSpeakerChannel, 0);
          testAlarmPlaying = false;
        }
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
          alarmTriggered = false;
          StopVibration();
          currentScreen = Screen::Dashboard;
        } else {
          alarmTriggered = false;
          StopVibration();
          if (timeValid && now >= alarmTime) {
            alarmTime = RtcDateTime(alarmTime.Unix32Time() + 86400);
          }
          currentScreen = Screen::Dashboard;
        }
      }
      break;
  }
  
  if (!mqttClient.connected()) {
    while (!mqttClient.connected()) {
      String clientId = "esp32-max30102-" + String((uint32_t)ESP.getEfuseMac(), HEX);
      if (mqttClient.connect(clientId.c_str())) {
        Serial.println("MQTT connected");
        // Subscribe to wake alert topic
        mqttClient.subscribe("esp32/wake_alert");
        mqttClient.subscribe("esp32/control");
        Serial.println("🔔 Subscribed to wake alerts and control messages");
      } else {
        Serial.print("MQTT connect failed, rc=");
        Serial.print(mqttClient.state());
        Serial.println(" retrying in 2s");
        delay(2000);
      }
    }
  }
  mqttClient.loop();

  // Check if sampling should stop (wake alert triggered)
  if (stopSampling) {
    delay(100);  // Just keep MQTT connection alive
    return;      // Don't collect or send data
  }

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
      display.print(mainMenuIndex == 4 ? "> " : "  ");
      display.println("System Settings");
    } else if (currentScreen == Screen::ViewAlarm) {
      display.setTextSize(1);
      display.setCursor(0, 0);
      display.println("Current Alarm");
      display.setCursor(0, 16);
      char alarmStr[12];
      snprintf(alarmStr, sizeof(alarmStr), "%02u:%02u", 
               alarmTime.Hour(), alarmTime.Minute());
      DrawCenteredText(alarmStr, 25, 2);
      const char *typeStr;
      if (alarmType == 0) {
        typeStr = "Normal Alarm";
      } else if (alarmType == 1) {
        typeStr = "Max Sleep Mode";
      } else {
        typeStr = "Smart Sleep Mode";
      }
      DrawCenteredText(typeStr, 45, 1);
    } else if (currentScreen == Screen::SetAlarm) {
      display.setTextSize(1);
      display.setCursor(0, 0);
      display.println("Set Alarm Time");
      display.setCursor(0, 16);
      char dateStr[12];
      snprintf(dateStr, sizeof(dateStr), "%04u-%02u-%02u",
               alarmYear, alarmMonth, alarmDay);
      display.println(dateStr);
      display.setCursor(0, 26);
      char timeStr[10];
      snprintf(timeStr, sizeof(timeStr), "%02u:%02u", alarmHour, alarmMinute);
      display.println(timeStr);
      display.setCursor(0, 36);
      display.print("Now: ");
      char nowStr[10];
      snprintf(nowStr, sizeof(nowStr), "%02u:%02u",
               now.Hour(), now.Minute());
      display.println(nowStr);
      bool blinkOn = ((nowMs / 400) % 2) == 0;
      if (blinkOn) {
        const int charW = 6;
        const int charH = 8;
        int x = 0;
        int y = 0;
        int w = 0;
        int h = charH + 2;
        if (setAlarmFieldIndex == 0) {
          x = 0; y = 15; w = 4 * charW;
        } else if (setAlarmFieldIndex == 1) {
          x = 5 * charW; y = 15; w = 2 * charW;
        } else if (setAlarmFieldIndex == 2) {
          x = 8 * charW; y = 15; w = 2 * charW;
        } else if (setAlarmFieldIndex == 3) {
          x = 0; y = 25; w = 2 * charW;
        } else {
          x = 3 * charW; y = 25; w = 2 * charW;
        }
        display.drawRect(x - 1, y - 1, w + 2, h, SSD1306_WHITE);
      }
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
      
      if (showAlarmTypeInfo) {
        display.setCursor(0, 45);
        if (setAlarmTypeIndex == 0) {
          display.println("Sounds at set");
          display.println("time only");
        } else if (setAlarmTypeIndex == 1) {
          display.println("Wakes before deep");
          display.println("sleep (< 1 cycle)");
        } else {
          display.println("Wakes before deep");
          display.println("sleep or timer end");
        }
      }
    } else if (currentScreen == Screen::SmartAlarm) {
      display.setTextSize(1);
      display.setCursor(0, 0);
      display.println("Smart Alarm");
      display.setCursor(0, 16);
      display.print(smartAlarmMenuIndex == 0 ? "> " : "  ");
      display.println("Max Sleep");
      display.print(smartAlarmMenuIndex == 1 ? "> " : "  ");
      display.println("Smart Sleep");
      
      if (showSmartAlarmInfo) {
        display.setCursor(0, 35);
        if (smartAlarmMenuIndex == 0) {
          display.println("Wakes before deep");
          display.println("sleep (< 1 cycle)");
        } else {
          display.println("Wakes before deep");
          display.println("sleep or timer end");
        }
      }
    } else if (currentScreen == Screen::SystemSettingsMenu) {
      display.setTextSize(1);
      display.setCursor(0, 0);
      display.println("System Settings");
      display.setCursor(0, 16);
      display.print(systemMenuIndex == 0 ? "> " : "  ");
      display.println("Select Tone");
      display.print(systemMenuIndex == 1 ? "> " : "  ");
      display.println("Alarm Output");
      display.print(systemMenuIndex == 2 ? "> " : "  ");
      display.println("Test Output");
    } else if (currentScreen == Screen::AlarmOutput) {
      display.setTextSize(1);
      display.setCursor(0, 0);
      display.println("Alarm Output");
      display.setCursor(0, 16);
      display.print(alarmOutputIndex == 0 ? "> " : "  ");
      display.println("Vibration");
      display.print(alarmOutputIndex == 1 ? "> " : "  ");
      display.println("Speaker");
      display.print(alarmOutputIndex == 2 ? "> " : "  ");
      display.println("Both");
    } else if (currentScreen == Screen::TestAlarmOutput) {
      display.setTextSize(1);
      DrawCenteredText("Test Alarm Output", 0, 1);
      const char *outputStr = "Testing: Vibration";
      if (alarmOutput == 1) {
        outputStr = "Testing: Speaker";
      } else if (alarmOutput == 2) {
        outputStr = "Testing: Both";
      }
      DrawCenteredText(outputStr, 20, 1);
      if (testAlarmPlaying) {
        DrawCenteredText("Playing...", 35, 1);
      } else {
        DrawCenteredText("Press Right to Test", 35, 1);
      }
    } else if (currentScreen == Screen::SystemTone) {
      display.setTextSize(1);
      display.setCursor(0, 0);
      display.println("Select Tone");
      display.setCursor(0, 16);
      display.print(alarmToneIndex == 0 ? "> " : "  ");
      display.println("Tone 1");
      display.print(alarmToneIndex == 1 ? "> " : "  ");
      display.println("Tone 2");
      display.print(alarmToneIndex == 2 ? "> " : "  ");
      display.println("Tone 3");
    } else if (currentScreen == Screen::Alert) {
      DrawCenteredText("ALARM!", 0, 2);
      display.setTextSize(1);
      display.setCursor(0, 34);
      display.print(alertMenuIndex == 0 ? "> " : "  ");
      display.println("Snooze");
      display.print(alertMenuIndex == 1 ? "> " : "  ");
      display.println("Dismiss");
    }

    display.display();
  }

  // 
  // Print time every 5 seconds
  if (nowMs - lastTimeMs >= 5000) {
    lastTimeMs = nowMs;

    if (!timeValid) {
      Serial.println("RTC invalid time");
      return;
    }

    PrintDateTime(now);

    if (!alarmTriggered && now >= alarmTime) {
      alarmTriggered = true;
      Serial.println("ALARM: time reached");
    }
  }
}

