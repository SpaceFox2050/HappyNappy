#include <Arduino.h>
#include <Wire.h>
#include <MAX30105.h>
#include <heartRate.h>
#include <ThreeWire.h>
#include <RtcDS1302.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <string>
#include <unordered_map>

using std::unordered_map;
using std::string;

#define SDA_PIN 21
#define SCL_PIN 22

// OLED display configuration
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
#define SCREEN_ADDRESS 0x3C

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

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
const int buttonPadPin = 12;
const int readingTolerance = 60;

// Vibration motor
static const uint8_t kVibePin = 25;
static const uint16_t kSnoozeMinutes = 5;

// Menu state
enum class Screen {
  Dashboard,
  MainMenu,
  SetAlarm,
  SmartAlarm,
  SystemSettingsMenu,
  SystemTone,
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
uint8_t alarmToneIndex = 0;
uint8_t setAlarmFieldIndex = 0;
bool alarmEnabled = true;
uint16_t alarmYear = 2026;
uint8_t alarmMonth = 1;
uint8_t alarmDay = 1;
uint8_t alarmHour = kAlarmHour;
uint8_t alarmMinute = kAlarmMinute;

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

//reading the buttonpad press
std::string buttonpadReading(int reading){
  string pressed = "nopress";
  if(abs(reading-511) < readingTolerance){
    pressed = "up";
  }else if(abs(reading-1807) < readingTolerance){
    pressed = "down";
  }else if(abs(reading-1168) < readingTolerance){
    pressed = "left";
  }else if(abs(reading-2440) < readingTolerance){
    pressed = "right";
  }
  //comment out for now cuz prob dont need these 

  // else if(abs(reading-566) < readingTolerance){
  //   pressed = "a";
  // }else if(abs(reading-793) < readingTolerance){
  //   pressed = "b";
  // }else if(abs(reading-678) < readingTolerance){
  //   pressed = "c";
  // }else if(abs(reading - 907) < readingTolerance){
  //   pressed = "d";
  // }

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
  const uint16_t pattern2[] = {800, 400};

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

void setup() {
  Serial.begin(115200);
  delay(200);

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

  //initialize button pad
  pinMode(buttonPadPin, INPUT);
  pinMode(kVibePin, OUTPUT);
  digitalWrite(kVibePin, LOW);

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

  UpdateVibration(nowMs);
  Button button = ReadButtonEvent(nowMs);

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
      } else if (button == Button::Down && mainMenuIndex < 3) {
        mainMenuIndex++;
      } else if (button == Button::Right) {
        if (mainMenuIndex == 0) {
          alarmEnabled = !alarmEnabled;
          if (!alarmEnabled) {
            alarmTriggered = false;
            StopVibration();
          }
        } else if (mainMenuIndex == 1) {
          if (timeValid) {
            SetAlarmDefaultsFromNow(now);
          }
          setAlarmFieldIndex = 0;
          currentScreen = Screen::SetAlarm;
        } else if (mainMenuIndex == 2) {
          currentScreen = Screen::SmartAlarm;
        } else if (mainMenuIndex == 3) {
          systemMenuIndex = 0;
          currentScreen = Screen::SystemSettingsMenu;
        }
      } else if (button == Button::Left) {
        currentScreen = Screen::Dashboard;
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
        currentScreen = Screen::MainMenu;
      }
      break;
    case Screen::SmartAlarm:
      if (button == Button::Left) {
        currentScreen = Screen::MainMenu;
      }
      break;
    case Screen::SystemSettingsMenu:
      if (button == Button::Up && systemMenuIndex > 0) {
        systemMenuIndex--;
      } else if (button == Button::Down && systemMenuIndex < 1) {
        systemMenuIndex++;
      } else if (button == Button::Right) {
        if (systemMenuIndex == 0) {
          currentScreen = Screen::SystemTone;
          lastTonePreview = 255;
        }
      } else if (button == Button::Left) {
        currentScreen = Screen::MainMenu;
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
  
  // Read sensor data
  long irValue = particleSensor.getIR();

  // Check if finger is detected
  if (irValue > kFingerThreshold) {
    if (checkForBeat(irValue)) {
      long delta = millis() - lastBeat;
      lastBeat = millis();

      int beatsPerMinute = 60 / (delta / 1000.0);

      if (beatsPerMinute < 255 && beatsPerMinute > 20) {
        rates[rateSpot++] = (byte)beatsPerMinute;
        rateSpot %= RATE_SIZE;

        int beatAvg = 0;
        for (byte x = 0; x < RATE_SIZE; x++) {
          beatAvg += rates[x];
        }
        beatAvg /= RATE_SIZE;
        heartRate = beatAvg;
        validHeartRate = (heartRate > 20 && heartRate < 200) ? 1 : 0;
        
        // Simplified SpO2 estimation (for display only, not medical-grade)
        // Based on ratio of red/IR - this is a rough approximation
        float ratio = (float)particleSensor.getRed() / (float)irValue;
        if (ratio > 0.4 && ratio < 2.0) {
          spo2 = (int32_t)(110 - 25 * ratio);
          if (spo2 > 100) spo2 = 100;
          if (spo2 < 80) spo2 = 80;
          validSpo2 = 1;
        } else {
          validSpo2 = 0;
        }
      }
    }
  } else {
    // No finger detected, reset
    validHeartRate = 0;
    validSpo2 = 0;
    rateSpot = 0;
    for (byte x = 0; x < RATE_SIZE; x++) {
      rates[x] = 0;
    }
  }

  // Print heart rate and SpO2 every second
  if (nowMs - lastVitalsMs >= 1000) {
    lastVitalsMs = nowMs;

    Serial.print("HR: ");
    if (validHeartRate) {
      Serial.print(heartRate);
      Serial.print(" bpm");
    } else {
      Serial.print("na");
    }
    
    Serial.print(" | SpO2: ");
    if (validSpo2) {
      Serial.print(spo2);
      Serial.print(" %");
    } else {
      Serial.print("na");
    }
    
    Serial.print(" | Button: ");
    int currentButtonReading = analogRead(buttonPadPin);
    Serial.print(currentButtonReading);
    Serial.print(" (");
    Serial.print(buttonpadReading(currentButtonReading).c_str());
    Serial.print(")");
    
    Serial.println();
  }

  uint32_t displayIntervalMs = (currentScreen == Screen::Dashboard) ? 1000 : 200;
  if (nowMs - lastDisplayMs >= displayIntervalMs) {
    lastDisplayMs = nowMs;
    display.clearDisplay();

    if (currentScreen == Screen::Dashboard) {
      if (timeValid) {
        display.setTextSize(2);
        display.setCursor(10, 10);
        char timeStr[10];
        snprintf(timeStr, sizeof(timeStr), "%02u:%02u:%02u",
                 now.Hour(), now.Minute(), now.Second());
        display.println(timeStr);

        display.setTextSize(1);
        display.setCursor(10, 35);
        char dateStr[12];
        snprintf(dateStr, sizeof(dateStr), "%04u-%02u-%02u",
                 now.Year(), now.Month(), now.Day());
        display.println(dateStr);
      }

      display.setTextSize(1);
      display.setCursor(0, 50);
      display.print("HR:");
      if (validHeartRate) {
        display.print(heartRate);
      } else {
        display.print("--");
      }
      display.print(" SpO2:");
      if (validSpo2) {
        display.print(spo2);
      } else {
        display.print("--");
      }
    } else if (currentScreen == Screen::MainMenu) {
      display.setTextSize(1);
      display.setCursor(0, 0);
      display.println("Main Menu");
      display.setCursor(0, 16);
      display.print(mainMenuIndex == 0 ? "> " : "  ");
      display.print("Alarm: ");
      display.println(alarmEnabled ? "On" : "Off");
      display.print(mainMenuIndex == 1 ? "> " : "  ");
      display.println("Set Alarm");
      display.print(mainMenuIndex == 2 ? "> " : "  ");
      display.println("Set Smart Alarm");
      display.print(mainMenuIndex == 3 ? "> " : "  ");
      display.println("System Settings");
    } else if (currentScreen == Screen::SetAlarm) {
      display.setTextSize(1);
      display.setCursor(0, 0);
      display.println("Set Alarm");
      display.setCursor(0, 16);
      char dateStr[12];
      snprintf(dateStr, sizeof(dateStr), "%04u-%02u-%02u",
               alarmYear, alarmMonth, alarmDay);
      display.println(dateStr);
      display.setCursor(0, 26);
      char timeStr[10];
      snprintf(timeStr, sizeof(timeStr), "%02u:%02u", alarmHour, alarmMinute);
      display.println(timeStr);
      bool blinkOn = ((nowMs / 400) % 2) == 0;
      if (blinkOn) {
        const int charW = 6;
        const int charH = 8;
        int x = 0;
        int y = 0;
        int w = 0;
        int h = charH + 2;
        if (setAlarmFieldIndex == 0) {
          x = 0;
          y = 15;
          w = 4 * charW;
        } else if (setAlarmFieldIndex == 1) {
          x = 5 * charW;
          y = 15;
          w = 2 * charW;
        } else if (setAlarmFieldIndex == 2) {
          x = 8 * charW;
          y = 15;
          w = 2 * charW;
        } else if (setAlarmFieldIndex == 3) {
          x = 0;
          y = 25;
          w = 2 * charW;
        } else {
          x = 3 * charW;
          y = 25;
          w = 2 * charW;
        }
        display.drawRect(x - 1, y - 1, w + 2, h, SSD1306_WHITE);
      }
    } else if (currentScreen == Screen::SmartAlarm) {
      display.setTextSize(1);
      DrawCenteredText("Smart Alarm", 0, 1);
      DrawCenteredText("(placeholder)", 20, 1);
    } else if (currentScreen == Screen::SystemSettingsMenu) {
      display.setTextSize(1);
      display.setCursor(0, 0);
      display.println("System Settings");
      display.setCursor(0, 16);
      display.print(systemMenuIndex == 0 ? "> " : "  ");
      display.println("Select Tone");
      display.print(systemMenuIndex == 1 ? "> " : "  ");
      display.println("Placeholder");
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

