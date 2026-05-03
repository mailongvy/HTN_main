// Phiên bản rút gọn: giữ nguyên logic và chức năng, chỉ làm gọn trình bày để dễ đọc.

#include <Arduino.h>
#include <SPI.h>
#include <math.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ILI9341.h>
#include <Adafruit_NeoPixel.h>
#include <DHT.h>
#include <ESP32Servo.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/event_groups.h"
#include "freertos/timers.h"
#include "freertos/portable.h"

/* =========================
   Tùy chọn
   ========================= */
#define ENABLE_DEBUG_TASKS      1
#define ENABLE_STACK_DETAIL     0

#define STATE_MUTEX_TIMEOUT_MS  5
#define TFT_MUTEX_TIMEOUT_MS    10

/* =========================
   Chân cảm biến và ngõ ra
   ========================= */
#define FLAME_DO_PIN        27
#define FLAME_AO_PIN        34
#define MQ2_AO_PIN          32
#define DHT_PIN             4
#define DHT_TYPE            DHT11

#define BUZZER_PIN          25
#define PUMP_PIN            26

#define MANUAL_BTN_PIN      13
#define RESET_BTN_PIN       14

/* =========================
   WS2812 RGB
   ========================= */
#define RGB_WS2812_PIN      2
#define RGB_LED_COUNT       8

/* =========================
   Servo
   ========================= */
#define SERVO_PIN           15
#define SERVO_MIN_ANGLE     2
#define SERVO_MAX_ANGLE     178
#define SERVO_STEP_DEG      2
#define SERVO_STEP_DELAY_MS 35

/* =========================
   TFT ILI9341 SPI
   ========================= */
#define TFT_SCK             18
#define TFT_MISO            19
#define TFT_MOSI            23
#define TFT_RST             17
#define TFT_DC              16
#define TFT_BLK             33
#define TFT_CS              -1

/* =========================
   Logic và ngưỡng
   ========================= */
#define FLAME_ACTIVE_LEVEL  LOW
#define BUZZER_ACTIVE_LEVEL HIGH
#define PUMP_ACTIVE_LEVEL   LOW

#define TEMP_THRESHOLD_C      50.0f
#define MQ2_THRESHOLD         1000
#define HUMI_LOW_THRESHOLD    30.0f
#define FLAME_AO_THRESHOLD    2500

/* =========================
   Chu kỳ task
   ========================= */
#define SENSOR_PERIOD_MS      150
#define DHT_PERIOD_MS         2000
#define RGB_PERIOD_MS         30
#define TFT_PERIOD_MS         150
#define SERIAL_PERIOD_MS      2000
#define HEALTH_PERIOD_MS      5000

/* =========================
   Firebase / WiFi config
   ========================= */
#define WIFI_SSID             "vtbpro"
#define WIFI_PASSWORD         "11111111"
#define FIREBASE_HOST         "testhtn-23965-default-rtdb.firebaseio.com"
#define FIREBASE_AUTH         "P4hyzooFsfo0CsvvehNNZtzM40qiApR6St9RqzTt"
#define FIREBASE_PATH         "/sensor_logs/latest"
#define FIREBASE_UPLOAD_EVERY 5
#define FIREBASE_COMMAND_PATH "/commands"
#define COMMAND_POLL_MS       500

/* =========================
   Event bit
   ========================= */
#define FIRE_DETECTED_BIT     (1 << 0)
#define GAS_DETECTED_BIT      (1 << 1)
#define TEMP_DETECTED_BIT     (1 << 2)
#define HUMI_LOW_BIT          (1 << 3)
#define FULL_ALARM_BIT        (1 << 4)
#define WARNING_ALARM_BIT     (1 << 5)
#define MANUAL_ALARM_BIT      (1 << 6)
#define RESET_HOLD_BIT        (1 << 7)

enum SystemMode
{
  MODE_SAFE = 0,
  MODE_WARNING,
  MODE_ALARM,
  MODE_MANUAL,
  MODE_RESET
};

/* =========================
   Dữ liệu cảm biến
   ========================= */
typedef struct
{
  int flameDO;
  int flameAO;
  int mq2AO;
  float tempC;
  float humi;
  bool dhtOk;
} SensorFrame_t;

/* =========================
   Trạng thái hệ thống
   ========================= */
typedef struct
{
  int flameDO;
  int flameAO;
  int mq2AO;
  float tempC;
  float humi;
  bool dhtOk;

  bool fire;
  bool gas;
  bool tempHigh;
  bool humiLow;

  bool manual;
  bool reset;
  bool buzzer;
  bool rgb;
  bool pump;

  int servoAngleValue;
  bool servoHold;
  int servoDirection;
} SystemState_t;

/* =========================
   RTOS objects
   ========================= */
EventGroupHandle_t systemEventGroup = NULL;
QueueHandle_t qSensor = NULL;
SemaphoreHandle_t mutexTFT = NULL;
SemaphoreHandle_t mutexState = NULL;
SemaphoreHandle_t semUiUpdate = NULL;
TimerHandle_t rgbBlinkTimer = NULL;
SemaphoreHandle_t mutexFirebaseHttp = NULL;

/* =========================
   Task handles
   ========================= */
TaskHandle_t hTaskReadSensors     = NULL;
TaskHandle_t hTaskProcessSensors  = NULL;
TaskHandle_t hTaskManualButton    = NULL;
TaskHandle_t hTaskResetButton     = NULL;
TaskHandle_t hTaskServoScan       = NULL;
TaskHandle_t hTaskAlarmControl    = NULL;
TaskHandle_t hTaskRgbStatus       = NULL;
TaskHandle_t hTaskDisplayTFT      = NULL;
TaskHandle_t hTaskSerialMonitor   = NULL;
TaskHandle_t hTaskHealthMonitor   = NULL;
TaskHandle_t hTaskFirebaseCommand = NULL;

/* =========================
   Hardware objects
   ========================= */
DHT dht(DHT_PIN, DHT_TYPE);
Servo flameServo;
Adafruit_ILI9341 tft(TFT_CS, TFT_DC, TFT_RST);
Adafruit_NeoPixel rgbStrip(RGB_LED_COUNT, RGB_WS2812_PIN, NEO_GRB + NEO_KHZ800);

/* =========================
   Shared state
   ========================= */
SystemState_t gState = {
  .flameDO = 1,
  .flameAO = 0,
  .mq2AO = 0,
  .tempC = 0.0f,
  .humi = 0.0f,
  .dhtOk = false,
  .fire = false,
  .gas = false,
  .tempHigh = false,
  .humiLow = false,
  .manual = false,
  .reset = false,
  .buzzer = false,
  .rgb = false,
  .pump = false,
  .servoAngleValue = SERVO_MIN_ANGLE,
  .servoHold = false,
  .servoDirection = 1
};

volatile bool rgbBlinkState = true;
volatile int  rgbModeApplied = -1;
volatile uint32_t serialPublishCounter = 0;
String lastProcessedCmdId = "";
volatile bool cmdPumpOverrideEnabled = false;
volatile bool cmdPumpOverrideValue = false;
volatile bool cmdBuzzerOverrideEnabled = false;
volatile bool cmdBuzzerOverrideValue = false;

/* =========================
   Cache TFT
   ========================= */
int   lastFlameState = -1;
int   lastFlameAO = -1;
int   lastGasAO = -1;
int   lastSystemMode = -1;
int   lastBuzzer = -1;
int   lastLed = -1;
int   lastPump = -1;
int   lastManualBtn = -1;
int   lastResetBtn = -1;
float lastTempC = -9999.0f;
float lastHumidity = -9999.0f;

/* =========================
   Màu TFT
   ========================= */
const uint16_t COLOR_BG        = 0x0841;
const uint16_t COLOR_HEADER    = 0x13A6;
const uint16_t COLOR_CARD      = 0x10A2;
const uint16_t COLOR_CARD2     = 0x18E3;
const uint16_t COLOR_BORDER    = 0x31C7;
const uint16_t COLOR_TEXT      = ILI9341_WHITE;
const uint16_t COLOR_LABEL     = 0x7D7C;
const uint16_t COLOR_SAFE_BG   = 0x05E0;
const uint16_t COLOR_WARN_BG   = 0xFD20;
const uint16_t COLOR_ALARM_BG  = 0xF940;
const uint16_t COLOR_MANUAL_BG = ILI9341_RED;
const uint16_t COLOR_RESET_BG  = ILI9341_BLUE;
const uint16_t COLOR_OFF_BG    = 0x5AEB;
const uint16_t COLOR_VALUE     = ILI9341_WHITE;

/* =========================================================
   Helper
   ========================================================= */
inline void lockState()
{
  if (mutexState) xSemaphoreTake(mutexState, portMAX_DELAY);
}

inline void unlockState()
{
  if (mutexState) xSemaphoreGive(mutexState);
}

inline bool lockStateTimed(TickType_t ticks)
{
  if (!mutexState) return false;
  return xSemaphoreTake(mutexState, ticks) == pdTRUE;
}

inline void requestUiUpdate()
{
  if (semUiUpdate) xSemaphoreGive(semUiUpdate);
}

bool copyState(SystemState_t *dst)
{
  if (!dst) return false;

  if (lockStateTimed(pdMS_TO_TICKS(STATE_MUTEX_TIMEOUT_MS))) {
    *dst = gState;
    unlockState();
    return true;
  }
  return false;
}

inline void setEventBit(EventBits_t bit, bool on)
{
  if (on) xEventGroupSetBits(systemEventGroup, bit);
  else    xEventGroupClearBits(systemEventGroup, bit);
}

SystemMode getSystemMode()
{
  EventBits_t bits = xEventGroupGetBits(systemEventGroup);

  if (bits & RESET_HOLD_BIT)   return MODE_RESET;
  if (bits & MANUAL_ALARM_BIT) return MODE_MANUAL;
  if (bits & FULL_ALARM_BIT)   return MODE_ALARM;
  if (bits & WARNING_ALARM_BIT) return MODE_WARNING;
  return MODE_SAFE;
}

String getModeString()
{
  switch (getSystemMode()) {
    case MODE_SAFE:    return "SAFE";
    case MODE_WARNING: return "WARNING";
    case MODE_ALARM:   return "ALARM";
    case MODE_MANUAL:  return "MANUAL";
    case MODE_RESET:   return "RESET";
    default:           return "SAFE";
  }
}

void updateAlarmBits()
{
  SystemState_t s;
  if (!copyState(&s)) return;

  bool fullAlarm = s.fire || s.gas || s.tempHigh;
  bool warning   = s.humiLow;

  setEventBit(FULL_ALARM_BIT, fullAlarm);
  setEventBit(WARNING_ALARM_BIT, warning);
}

void setBuzzer(bool on)
{
  digitalWrite(BUZZER_PIN, on ? BUZZER_ACTIVE_LEVEL : !BUZZER_ACTIVE_LEVEL);

  lockState();
  gState.buzzer = on;
  unlockState();

  requestUiUpdate();
}

void setPump(bool on)
{
  digitalWrite(PUMP_PIN, on ? PUMP_ACTIVE_LEVEL : !PUMP_ACTIVE_LEVEL);

  lockState();
  gState.pump = on;
  unlockState();

  requestUiUpdate();
}

void clearAllAlarmStates()
{
  lockState();
  gState.fire = false;
  gState.gas = false;
  gState.tempHigh = false;
  gState.humiLow = false;
  gState.manual = false;
  gState.buzzer = false;
  gState.pump = false;
  gState.rgb = false;
  unlockState();

  xEventGroupClearBits(
    systemEventGroup,
    FIRE_DETECTED_BIT |
    GAS_DETECTED_BIT  |
    TEMP_DETECTED_BIT |
    HUMI_LOW_BIT      |
    FULL_ALARM_BIT    |
    WARNING_ALARM_BIT |
    MANUAL_ALARM_BIT
  );

  digitalWrite(BUZZER_PIN, !BUZZER_ACTIVE_LEVEL);
  digitalWrite(PUMP_PIN, !PUMP_ACTIVE_LEVEL);
}

bool ensureWiFiConnected(uint32_t timeoutMs)
{
  if (WiFi.status() == WL_CONNECTED) {
    return true;
  }

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  uint32_t startMs = millis();
  while (WiFi.status() != WL_CONNECTED && (millis() - startMs) < timeoutMs) {
    vTaskDelay(pdMS_TO_TICKS(300));
  }

  return WiFi.status() == WL_CONNECTED;
}

bool uploadToFirebaseLatest()
{
  if (!ensureWiFiConnected(5000)) {
    Serial.println("[Firebase] WiFi not connected.");
    return false;
  }

  SystemState_t s;
  if (!copyState(&s)) {
    Serial.println("[Firebase] State copy failed.");
    return false;
  }

  EventBits_t bits = xEventGroupGetBits(systemEventGroup);
  bool fullAlarmNow = (bits & FULL_ALARM_BIT) != 0;
  bool warningNow   = (bits & WARNING_ALARM_BIT) != 0;
  bool manualNow    = (bits & MANUAL_ALARM_BIT) != 0;
  bool resetNow     = (bits & RESET_HOLD_BIT) != 0;

  if (!mutexFirebaseHttp) {
    Serial.println("[Firebase] HTTP mutex missing.");
    return false;
  }

  if (xSemaphoreTake(mutexFirebaseHttp, pdMS_TO_TICKS(2000)) != pdTRUE) {
    Serial.println("[Firebase] HTTP mutex timeout.");
    return false;
  }

  HTTPClient http;
  WiFiClientSecure client;
  String url = String("https://") + FIREBASE_HOST + FIREBASE_PATH + ".json?auth=" + FIREBASE_AUTH;

  client.setInsecure();
  if (!http.begin(client, url)) {
    Serial.println("[Firebase] HTTP begin failed.");
    xSemaphoreGive(mutexFirebaseHttp);
    return false;
  }

  http.setTimeout(5000);
  http.addHeader("Content-Type", "application/json");

  unsigned long nowMs = millis();
  String payload = "{";
  payload += "\"mode\":\"" + getModeString() + "\",";
  payload += "\"updatedAt\":" + String(nowMs) + ",";
  payload += "\"millis\":" + String(nowMs) + ",";
  payload += "\"flameDO\":" + String(s.flameDO) + ",";
  payload += "\"flameAO\":" + String(s.flameAO) + ",";
  payload += "\"mq2AO\":" + String(s.mq2AO) + ",";
  payload += "\"tempC\":" + String(s.tempC, 1) + ",";
  payload += "\"humi\":" + String(s.humi, 1) + ",";
  payload += "\"dhtOk\":" + String(s.dhtOk ? "true" : "false") + ",";
  payload += "\"fire\":" + String(s.fire ? "true" : "false") + ",";
  payload += "\"gas\":" + String(s.gas ? "true" : "false") + ",";
  payload += "\"tempHigh\":" + String(s.tempHigh ? "true" : "false") + ",";
  payload += "\"humiLow\":" + String(s.humiLow ? "true" : "false") + ",";
  payload += "\"manual\":" + String(manualNow ? "true" : "false") + ",";
  payload += "\"reset\":" + String(resetNow ? "true" : "false") + ",";
  payload += "\"manualBtn\":" + String(manualNow ? "true" : "false") + ",";
  payload += "\"resetBtn\":" + String(resetNow ? "true" : "false") + ",";
  payload += "\"fullAlarm\":" + String(fullAlarmNow ? "true" : "false") + ",";
  payload += "\"warning\":" + String(warningNow ? "true" : "false") + ",";
  payload += "\"buzzer\":" + String(s.buzzer ? "true" : "false") + ",";
  payload += "\"rgb\":" + String(s.rgb ? "true" : "false") + ",";
  payload += "\"pump\":" + String(s.pump ? "true" : "false") + ",";
  payload += "\"servoAngle\":" + String(s.servoAngleValue);
  payload += "}";

  int httpCode = http.PUT(payload);
  bool ok = (httpCode >= 200 && httpCode < 300);
  if (!ok) {
    Serial.print("[Firebase] Upload fail, HTTP=");
    Serial.println(httpCode);
  }

  http.end();
  xSemaphoreGive(mutexFirebaseHttp);
  return ok;
}

bool jsonTryGetBool(const String &json, const char *key, bool *outValue)
{
  if (!outValue) return false;

  String truePattern  = String("\"") + key + "\":true";
  String falsePattern = String("\"") + key + "\":false";

  if (json.indexOf(truePattern) >= 0) {
    *outValue = true;
    return true;
  }
  if (json.indexOf(falsePattern) >= 0) {
    *outValue = false;
    return true;
  }
  return false;
}

String jsonGetString(const String &json, const char *key)
{
  String pattern = String("\"") + key + "\":\"";
  int start = json.indexOf(pattern);
  if (start < 0) return "";

  start += pattern.length();
  int end = json.indexOf('"', start);
  if (end < 0) return "";

  return json.substring(start, end);
}

void applyFirebaseCommand(const String &payload)
{
  String cmdId = jsonGetString(payload, "cmdId");
  if (cmdId.length() == 0 || cmdId == lastProcessedCmdId) {
    return;
  }

  bool v = false;

  if (jsonTryGetBool(payload, "manualAlarm", &v)) {
    lockState();
    gState.manual = v;
    unlockState();
    setEventBit(MANUAL_ALARM_BIT, v);

    // Khi bật manual từ web, bỏ override riêng của pump/buzzer
    // để hệ thống quay về đúng logic MODE_MANUAL.
    if (v) {
      cmdPumpOverrideEnabled = false;
      cmdBuzzerOverrideEnabled = false;
    }
  }

  if (jsonTryGetBool(payload, "resetAlarm", &v) && v && hTaskResetButton) {
    xTaskNotifyGive(hTaskResetButton);
  }

  if (jsonTryGetBool(payload, "pump", &v)) {
    cmdPumpOverrideEnabled = true;
    cmdPumpOverrideValue = v;
  }

  if (jsonTryGetBool(payload, "buzzer", &v)) {
    cmdBuzzerOverrideEnabled = true;
    cmdBuzzerOverrideValue = v;
  }

  lastProcessedCmdId = cmdId;
  requestUiUpdate();
}

void performResetBlocking(TickType_t holdTicks)
{
  lockState();
  gState.reset = true;
  gState.servoHold = false;
  gState.servoAngleValue = SERVO_MIN_ANGLE;
  unlockState();

  xEventGroupSetBits(systemEventGroup, RESET_HOLD_BIT);
  requestUiUpdate();

  clearAllAlarmStates();
  xQueueReset(qSensor);
  flameServo.write(SERVO_MIN_ANGLE);

  vTaskDelay(holdTicks);

  lockState();
  gState.reset = false;
  unlockState();

  xEventGroupClearBits(systemEventGroup, RESET_HOLD_BIT);
  requestUiUpdate();
}

/* =========================================================
   RGB
   ========================================================= */
void setAllRgb(uint8_t r, uint8_t g, uint8_t b)
{
  for (int i = 0; i < RGB_LED_COUNT; i++) {
    rgbStrip.setPixelColor(i, rgbStrip.Color(r, g, b));
  }
  rgbStrip.show();
}

void clearRgb()
{
  rgbStrip.clear();
  rgbStrip.show();
}

void rgbBlinkTimerCallback(TimerHandle_t xTimer)
{
  (void)xTimer;
  rgbBlinkState = !rgbBlinkState;
}

void updateRgbStatus()
{
  SystemMode mode = getSystemMode();

  if ((int)mode != rgbModeApplied) {
    rgbModeApplied = (int)mode;
    rgbBlinkState = true;

    if (mode == MODE_MANUAL) {
      xTimerChangePeriod(rgbBlinkTimer, pdMS_TO_TICKS(120), 0);
    } else if (mode == MODE_WARNING) {
      xTimerChangePeriod(rgbBlinkTimer, pdMS_TO_TICKS(250), 0);
    } else {
      xTimerChangePeriod(rgbBlinkTimer, pdMS_TO_TICKS(300), 0);
    }
  }

  bool rgbOn = false;

  switch (mode)
  {
    case MODE_SAFE:
      setAllRgb(0, 120, 0);
      rgbOn = true;
      break;

    case MODE_WARNING:
      if (rgbBlinkState) {
        setAllRgb(255, 140, 0);
        rgbOn = true;
      } else {
        clearRgb();
        rgbOn = false;
      }
      break;

    case MODE_ALARM:
      if (rgbBlinkState) {
        setAllRgb(180, 0, 0);
        rgbOn = true;
      } else {
        clearRgb();
        rgbOn = false;
      }
      break;

    case MODE_MANUAL:
      if (rgbBlinkState) {
        setAllRgb(130, 0, 150);
        rgbOn = true;
      } else {
        clearRgb();
        rgbOn = false;
      }
      break;

    case MODE_RESET:
      setAllRgb(0, 0, 180);
      rgbOn = true;
      break;
  }

  lockState();
  gState.rgb = rgbOn;
  unlockState();
}

/* =========================================================
   TFT
   ========================================================= */
void drawPanel(int x, int y, int w, int h, const char *title)
{
  tft.fillRoundRect(x, y, w, h, 10, COLOR_CARD);
  tft.drawRoundRect(x, y, w, h, 10, COLOR_BORDER);

  tft.setTextColor(COLOR_LABEL);
  tft.setTextSize(1);
  tft.setCursor(x + 8, y + 6);
  tft.print(title);
}

void clearPanelValue(int x, int y, int w, int h)
{
  tft.fillRect(x, y, w, h, COLOR_CARD);
}

void drawOutputBox(int x, int y, int w, int h, const char *title, bool active, uint16_t activeColor)
{
  tft.fillRoundRect(x, y, w, h, 10, COLOR_CARD);
  tft.drawRoundRect(x, y, w, h, 10, COLOR_BORDER);

  tft.setTextColor(COLOR_LABEL);
  tft.setTextSize(1);
  tft.setCursor(x + 6, y + 5);
  tft.print(title);

  uint16_t boxColor = active ? activeColor : COLOR_OFF_BG;
  tft.fillRoundRect(x + 6, y + 18, w - 12, h - 24, 10, boxColor);

  tft.setTextColor(COLOR_TEXT);
  tft.setTextSize(2);

  const char *txt = active ? "ON" : "OFF";
  int16_t x1, y1;
  uint16_t tw, th;
  tft.getTextBounds((char*)txt, 0, 0, &x1, &y1, &tw, &th);
  tft.setCursor(x + (w - tw) / 2, y + 24);
  tft.print(txt);
}

void drawBigBadge(int mode)
{
  uint16_t badgeColor;
  const char *text;

  if (mode == MODE_RESET) {
    badgeColor = COLOR_RESET_BG;
    text = "RESET";
  } else if (mode == MODE_MANUAL) {
    badgeColor = COLOR_MANUAL_BG;
    text = "MANUAL";
  } else if (mode == MODE_ALARM) {
    badgeColor = COLOR_ALARM_BG;
    text = "ALARM";
  } else if (mode == MODE_WARNING) {
    badgeColor = COLOR_WARN_BG;
    text = "WARNING";
  } else {
    badgeColor = COLOR_SAFE_BG;
    text = "SAFE";
  }

  tft.fillRoundRect(10, 40, 300, 26, 12, badgeColor);
  tft.setTextColor(COLOR_TEXT);
  tft.setTextSize(2);

  int16_t x1, y1;
  uint16_t tw, th;
  tft.getTextBounds((char*)text, 0, 0, &x1, &y1, &tw, &th);
  tft.setCursor(10 + (300 - tw) / 2, 45);
  tft.print(text);
}

void drawStaticUI()
{
  tft.setRotation(1);
  tft.fillScreen(COLOR_BG);

  tft.fillRoundRect(10, 8, 300, 24, 10, COLOR_HEADER);
  tft.setTextColor(COLOR_TEXT);
  tft.setTextSize(2);
  tft.setCursor(76, 13);
  tft.print("SMART FIRE ALARM");

  tft.fillRoundRect(8, 38, 304, 30, 12, COLOR_CARD2);
  tft.drawRoundRect(8, 38, 304, 30, 12, COLOR_BORDER);

  drawPanel(8,   76, 148, 44, "FLAME");
  drawPanel(164, 76, 148, 44, "MQ2 GAS");
  drawPanel(8,  126, 148, 44, "TEMPERATURE");
  drawPanel(164,126, 148, 44, "HUMIDITY");

  drawOutputBox(8,   178, 58, 54, "BUZZ", false, COLOR_ALARM_BG);
  drawOutputBox(70,  178, 58, 54, "RGB",  false, COLOR_SAFE_BG);
  drawOutputBox(132, 178, 58, 54, "PUMP", false, COLOR_ALARM_BG);
  drawOutputBox(194, 178, 58, 54, "MAN",  false, COLOR_MANUAL_BG);
  drawOutputBox(256, 178, 58, 54, "RST",  false, COLOR_RESET_BG);
}

void updateTFT()
{
  SystemState_t s;
  if (!copyState(&s)) return;

  int systemMode = (int)getSystemMode();

  if (systemMode != lastSystemMode) {
    drawBigBadge(systemMode);
    lastSystemMode = systemMode;
  }

  if ((int)s.fire != lastFlameState || s.flameAO != lastFlameAO) {
    clearPanelValue(16, 92, 132, 20);

    tft.setTextSize(2);
    tft.setTextColor(s.fire ? COLOR_ALARM_BG : COLOR_SAFE_BG);
    tft.setCursor(16, 92);
    tft.print(s.fire ? "FIRE" : "SAFE");

    tft.setTextSize(1);
    tft.setTextColor(COLOR_TEXT);
    tft.setCursor(88, 98);
    tft.print("AO:");
    tft.print(s.flameAO);

    lastFlameState = s.fire;
    lastFlameAO = s.flameAO;
  }

  if (s.mq2AO != lastGasAO) {
    clearPanelValue(172, 92, 132, 20);

    tft.setTextSize(2);
    tft.setTextColor(s.gas ? COLOR_ALARM_BG : COLOR_VALUE);
    tft.setCursor(172, 92);
    tft.print(s.mq2AO);

    tft.setTextSize(1);
    tft.setTextColor(s.gas ? COLOR_ALARM_BG : COLOR_LABEL);
    tft.setCursor(248, 98);
    tft.print(s.gas ? "GAS!" : "NORMAL");

    lastGasAO = s.mq2AO;
  }

  if (fabs(s.tempC - lastTempC) >= 0.1f || !s.dhtOk) {
    clearPanelValue(16, 142, 132, 20);

    tft.setTextSize(2);
    if (s.dhtOk) {
      tft.setTextColor(s.tempHigh ? COLOR_ALARM_BG : COLOR_VALUE);
      tft.setCursor(16, 142);
      tft.print(s.tempC, 1);
      tft.print(" C");
    } else {
      tft.setTextColor(COLOR_LABEL);
      tft.setCursor(16, 142);
      tft.print("N/A");
    }

    lastTempC = s.tempC;
  }

  if (fabs(s.humi - lastHumidity) >= 0.1f || !s.dhtOk) {
    clearPanelValue(172, 142, 132, 20);

    tft.setTextSize(2);
    if (s.dhtOk) {
      tft.setTextColor(s.humiLow ? COLOR_ALARM_BG : COLOR_VALUE);
      tft.setCursor(172, 142);
      tft.print(s.humi, 1);
      tft.print(" %");

      if (s.humiLow) {
        tft.setTextSize(1);
        tft.setCursor(256, 148);
        tft.print("LOW!");
      }
    } else {
      tft.setTextColor(COLOR_LABEL);
      tft.setCursor(172, 142);
      tft.print("N/A");
    }

    lastHumidity = s.humi;
  }

  if ((int)s.buzzer != lastBuzzer) {
    drawOutputBox(8, 178, 58, 54, "BUZZ", s.buzzer, COLOR_ALARM_BG);
    lastBuzzer = s.buzzer;
  }

  if ((int)s.rgb != lastLed) {
    drawOutputBox(70, 178, 58, 54, "RGB", s.rgb, COLOR_SAFE_BG);
    lastLed = s.rgb;
  }

  if ((int)s.pump != lastPump) {
    drawOutputBox(132, 178, 58, 54, "PUMP", s.pump, COLOR_ALARM_BG);
    lastPump = s.pump;
  }

  if ((int)s.manual != lastManualBtn) {
    drawOutputBox(194, 178, 58, 54, "MAN", s.manual, COLOR_MANUAL_BG);
    lastManualBtn = s.manual;
  }

  if ((int)s.reset != lastResetBtn) {
    drawOutputBox(256, 178, 58, 54, "RST", s.reset, COLOR_RESET_BG);
    lastResetBtn = s.reset;
  }
}

/* =========================================================
   ISR nút nhấn
   ========================================================= */
void IRAM_ATTR isrManualButton()
{
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  if (hTaskManualButton) vTaskNotifyGiveFromISR(hTaskManualButton, &xHigherPriorityTaskWoken);
  if (xHigherPriorityTaskWoken == pdTRUE) portYIELD_FROM_ISR();
}

void IRAM_ATTR isrResetButton()
{
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  if (hTaskResetButton) vTaskNotifyGiveFromISR(hTaskResetButton, &xHigherPriorityTaskWoken);
  if (xHigherPriorityTaskWoken == pdTRUE) portYIELD_FROM_ISR();
}

/* =========================================================
   RTOS object
   ========================================================= */
bool createRtosObjects()
{
  qSensor = xQueueCreate(1, sizeof(SensorFrame_t));
  mutexTFT = xSemaphoreCreateMutex();
  mutexState = xSemaphoreCreateMutex();
  semUiUpdate = xSemaphoreCreateBinary();
  systemEventGroup = xEventGroupCreate();
  rgbBlinkTimer = xTimerCreate("RgbBlinkTimer", pdMS_TO_TICKS(300), pdTRUE, NULL, rgbBlinkTimerCallback);
  return qSensor && mutexTFT && mutexState && semUiUpdate && systemEventGroup && rgbBlinkTimer;
}

/* =========================================================
   TASK 1
   ========================================================= */
void TaskReadSensors(void *pvParameters)
{
  (void)pvParameters;
  SensorFrame_t frame;
  float lastTemp = 0.0f;
  float lastHumi = 0.0f;
  bool lastDhtOk = false;

  TickType_t lastWake = xTaskGetTickCount();
  TickType_t lastDhtTick = lastWake;

  while (1)
  {
    frame.flameDO = digitalRead(FLAME_DO_PIN);
    frame.flameAO = analogRead(FLAME_AO_PIN);
    frame.mq2AO   = analogRead(MQ2_AO_PIN);

    TickType_t nowTick = xTaskGetTickCount();
    if ((nowTick - lastDhtTick) >= pdMS_TO_TICKS(DHT_PERIOD_MS)) {
      float h = dht.readHumidity();
      float t = dht.readTemperature();

      if (!isnan(h) && !isnan(t)) {
        lastHumi = h;
        lastTemp = t;
        lastDhtOk = true;
      } else {
        lastDhtOk = false;
      }

      lastDhtTick = nowTick;
    }

    frame.tempC = lastTemp;
    frame.humi  = lastHumi;
    frame.dhtOk = lastDhtOk;

    xQueueOverwrite(qSensor, &frame);
    vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(SENSOR_PERIOD_MS));
  }
}

/* =========================================================
   TASK 2
   ========================================================= */
void TaskProcessSensors(void *pvParameters)
{
  (void)pvParameters;
  SensorFrame_t frame;

  while (1)
  {
    if (xQueueReceive(qSensor, &frame, portMAX_DELAY) == pdPASS)
    {
      if (xEventGroupGetBits(systemEventGroup) & RESET_HOLD_BIT) continue;

      bool fireNow = (frame.flameDO == FLAME_ACTIVE_LEVEL) || (frame.flameAO <= FLAME_AO_THRESHOLD);
      bool gasNow  = (frame.mq2AO >= MQ2_THRESHOLD);
      bool tempNow = frame.dhtOk && (frame.tempC >= TEMP_THRESHOLD_C);
      bool humNow  = frame.dhtOk && (frame.humi < HUMI_LOW_THRESHOLD);

      lockState();
      gState.flameDO   = frame.flameDO;
      gState.flameAO   = frame.flameAO;
      gState.mq2AO     = frame.mq2AO;
      gState.tempC     = frame.tempC;
      gState.humi      = frame.humi;
      gState.dhtOk     = frame.dhtOk;
      gState.fire      = fireNow;
      gState.gas       = gasNow;
      gState.tempHigh  = tempNow;
      gState.humiLow   = humNow;
      unlockState();

      setEventBit(FIRE_DETECTED_BIT, fireNow);
      setEventBit(GAS_DETECTED_BIT, gasNow);
      setEventBit(TEMP_DETECTED_BIT, tempNow);
      setEventBit(HUMI_LOW_BIT, humNow);

      updateAlarmBits();
      requestUiUpdate();
    }
  }
}

/* =========================================================
   TASK 3
   ========================================================= */
void TaskManualButton(void *pvParameters)
{
  (void)pvParameters;
  TickType_t lastPressTick = 0;

  while (1)
  {
    if (ulTaskNotifyTake(pdTRUE, portMAX_DELAY) > 0)
    {
      TickType_t nowTick = xTaskGetTickCount();
      if ((nowTick - lastPressTick) < pdMS_TO_TICKS(200)) continue;
      lastPressTick = nowTick;

      lockState();
      gState.manual = !gState.manual;
      bool manualNow = gState.manual;
      unlockState();

      setEventBit(MANUAL_ALARM_BIT, manualNow);
      requestUiUpdate();
    }
  }
}

/* =========================================================
   TASK 4
   ========================================================= */
void TaskResetButton(void *pvParameters)
{
  (void)pvParameters;
  TickType_t lastPressTick = 0;

  while (1)
  {
    if (ulTaskNotifyTake(pdTRUE, portMAX_DELAY) > 0)
    {
      TickType_t nowTick = xTaskGetTickCount();
      if ((nowTick - lastPressTick) < pdMS_TO_TICKS(200)) continue;
      lastPressTick = nowTick;
      performResetBlocking(pdMS_TO_TICKS(5000));
    }
  }
}

/* =========================================================
   TASK 5
   ========================================================= */
void TaskServoScan(void *pvParameters)
{
  (void)pvParameters;
  const TickType_t delayTicks = pdMS_TO_TICKS(SERVO_STEP_DELAY_MS);

  flameServo.write(SERVO_MIN_ANGLE);

  lockState();
  gState.servoAngleValue = SERVO_MIN_ANGLE;
  gState.servoDirection = 1;
  gState.servoHold = false;
  unlockState();

  while (1)
  {
    bool resetNow = (xEventGroupGetBits(systemEventGroup) & RESET_HOLD_BIT) != 0;
    bool fireNow  = (xEventGroupGetBits(systemEventGroup) & FIRE_DETECTED_BIT) != 0;

    if (resetNow) {
      lockState();
      gState.servoAngleValue = SERVO_MIN_ANGLE;
      gState.servoHold = false;
      unlockState();

      flameServo.write(SERVO_MIN_ANGLE);
      vTaskDelay(pdMS_TO_TICKS(100));
      continue;
    }

    if (fireNow) {
      lockState();
      gState.servoHold = true;
      int holdAngle = gState.servoAngleValue;
      unlockState();

      flameServo.write(holdAngle);
      vTaskDelay(pdMS_TO_TICKS(80));
      continue;
    }

    int angle, dir;
    lockState();
    angle = gState.servoAngleValue;
    dir   = gState.servoDirection;
    unlockState();

    angle += dir * SERVO_STEP_DEG;
    if (angle >= SERVO_MAX_ANGLE) {
      angle = SERVO_MAX_ANGLE;
      dir = -1;
    } else if (angle <= SERVO_MIN_ANGLE) {
      angle = SERVO_MIN_ANGLE;
      dir = 1;
    }

    lockState();
    gState.servoAngleValue = angle;
    gState.servoDirection = dir;
    gState.servoHold = false;
    unlockState();

    flameServo.write(angle);
    vTaskDelay(delayTicks);
  }
}

/* =========================================================
   TASK 6
   ========================================================= */
void TaskAlarmControl(void *pvParameters)
{
  (void)pvParameters;

  while (1)
  {
    SystemMode mode = getSystemMode();

    // Chỉ cho phép override từ web khi hệ thống đang SAFE.
    // Khi đã vào WARNING / ALARM / MANUAL / RESET thì ưu tiên logic hệ thống.
    if (mode == MODE_SAFE) {
      if (cmdPumpOverrideEnabled) {
        setPump(cmdPumpOverrideValue);
      } else {
        setPump(false);
      }

      if (cmdBuzzerOverrideEnabled) {
        setBuzzer(cmdBuzzerOverrideValue);
      } else {
        setBuzzer(false);
      }

      vTaskDelay(pdMS_TO_TICKS(100));
      continue;
    }

    if (mode == MODE_RESET) {
      setBuzzer(false);
      setPump(false);
      vTaskDelay(pdMS_TO_TICKS(100));
    }
    else if (mode == MODE_MANUAL) {
      setBuzzer(true);
      setPump(true);
      vTaskDelay(pdMS_TO_TICKS(100));
    }
    else if (mode == MODE_ALARM) {
      setPump(true);
      setBuzzer(true);
      vTaskDelay(pdMS_TO_TICKS(250));
      setBuzzer(false);
      vTaskDelay(pdMS_TO_TICKS(150));
    }
    else if (mode == MODE_WARNING) {
      setPump(false);
      setBuzzer(true);
      vTaskDelay(pdMS_TO_TICKS(250));
      setBuzzer(false);
      vTaskDelay(pdMS_TO_TICKS(150));
    }
  }
}

/* =========================================================
   TASK 7
   ========================================================= */
void TaskRgbStatus(void *pvParameters)
{
  (void)pvParameters;
  TickType_t lastWake = xTaskGetTickCount();

  while (1)
  {
    updateRgbStatus();
    vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(RGB_PERIOD_MS));
  }
}

/* =========================================================
   TASK 8
   ========================================================= */
void TaskDisplayTFT(void *pvParameters)
{
  (void)pvParameters;

  while (1)
  {
    if (semUiUpdate) {
      xSemaphoreTake(semUiUpdate, pdMS_TO_TICKS(TFT_PERIOD_MS));
    } else {
      vTaskDelay(pdMS_TO_TICKS(TFT_PERIOD_MS));
    }

    if (xSemaphoreTake(mutexTFT, pdMS_TO_TICKS(TFT_MUTEX_TIMEOUT_MS)) == pdTRUE) {
      updateTFT();
      xSemaphoreGive(mutexTFT);
    }
  }
}

/* =========================================================
   TASK 9
   ========================================================= */
void TaskSerialMonitor(void *pvParameters)
{
  (void)pvParameters;
  TickType_t lastWake = xTaskGetTickCount();

  while (1)
  {
    SystemState_t s;
    if (copyState(&s)) {
      Serial.print("Mode=");    Serial.print(getModeString());
      Serial.print(" Fire=");   Serial.print(s.fire);
      Serial.print(" Gas=");    Serial.print(s.gas);
      Serial.print(" Temp=");   Serial.print(s.tempC, 1);
      Serial.print(" Humi=");   Serial.print(s.humi, 1);
      Serial.print(" MQ2=");    Serial.print(s.mq2AO);
      Serial.print(" Buzzer="); Serial.print(s.buzzer);
      Serial.print(" Pump=");   Serial.print(s.pump);
      Serial.print(" RGB=");    Serial.print(s.rgb);
      Serial.print(" Manual="); Serial.print(s.manual);
      Serial.print(" Reset=");  Serial.print(s.reset);
      Serial.print(" Servo=");  Serial.println(s.servoAngleValue);

      serialPublishCounter++;
      if (serialPublishCounter >= FIREBASE_UPLOAD_EVERY) {
        (void)uploadToFirebaseLatest();
        serialPublishCounter = 0;
      }
    }

    vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(SERIAL_PERIOD_MS));
  }
}

/* =========================================================
   TASK 10
   ========================================================= */
void TaskHealthMonitor(void *pvParameters)
{
  (void)pvParameters;
  TickType_t lastWake = xTaskGetTickCount();

  while (1)
  {
    Serial.println("===== RTOS HEALTH =====");
    Serial.print("FreeHeap    = "); Serial.println((uint32_t)xPortGetFreeHeapSize());
    Serial.print("MinEverHeap = "); Serial.println((uint32_t)xPortGetMinimumEverFreeHeapSize());
    Serial.print("NumTasks    = "); Serial.println((uint32_t)uxTaskGetNumberOfTasks());
    if (qSensor) {
      Serial.print("QueueFree   = "); Serial.println((uint32_t)uxQueueSpacesAvailable(qSensor));
    }
    Serial.println("=======================");
    vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(HEALTH_PERIOD_MS));
  }
}

/* =========================================================
   TASK 11
   ========================================================= */
void TaskFirebaseCommand(void *pvParameters)
{
  (void)pvParameters;

  while (1)
  {
    if (!ensureWiFiConnected(3000)) {
      vTaskDelay(pdMS_TO_TICKS(COMMAND_POLL_MS));
      continue;
    }

    if (mutexFirebaseHttp && xSemaphoreTake(mutexFirebaseHttp, pdMS_TO_TICKS(2000)) == pdTRUE) {
      HTTPClient http;
      WiFiClientSecure client;
      String url = String("https://") + FIREBASE_HOST + FIREBASE_COMMAND_PATH + ".json?auth=" + FIREBASE_AUTH;

      client.setInsecure();
      if (http.begin(client, url)) {
        http.setTimeout(4000);
        int httpCode = http.GET();

        if (httpCode >= 200 && httpCode < 300) {
          String payload = http.getString();
          if (payload.length() > 0 && payload != "null") {
            applyFirebaseCommand(payload);
          }
        } else {
          Serial.print("[Firebase] Command GET fail, HTTP=");
          Serial.println(httpCode);
        }

        http.end();
      }

      xSemaphoreGive(mutexFirebaseHttp);
    }

    (void)uploadToFirebaseLatest();
    vTaskDelay(pdMS_TO_TICKS(COMMAND_POLL_MS));
  }
}

/* =========================================================
   setup
   ========================================================= */
void setup()
{
  Serial.begin(115200);

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  pinMode(FLAME_DO_PIN, INPUT);
  pinMode(FLAME_AO_PIN, INPUT);
  pinMode(MQ2_AO_PIN, INPUT);

  pinMode(MANUAL_BTN_PIN, INPUT_PULLUP);
  pinMode(RESET_BTN_PIN, INPUT_PULLUP);

  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(PUMP_PIN, OUTPUT);

  analogReadResolution(12);
  dht.begin();

  if (!createRtosObjects()) {
    while (1) delay(1000);
  }

  if (!mutexFirebaseHttp) {
    mutexFirebaseHttp = xSemaphoreCreateMutex();
  }

  setBuzzer(false);
  setPump(false);

  flameServo.setPeriodHertz(50);
  flameServo.attach(SERVO_PIN, 500, 2500);
  flameServo.write(SERVO_MIN_ANGLE);

  rgbStrip.begin();
  rgbStrip.setBrightness(40);
  clearRgb();
  xTimerStart(rgbBlinkTimer, 0);

  pinMode(TFT_BLK, OUTPUT);
  digitalWrite(TFT_BLK, HIGH);

  SPI.begin(TFT_SCK, TFT_MISO, TFT_MOSI);
  tft.begin();

  if (xSemaphoreTake(mutexTFT, pdMS_TO_TICKS(TFT_MUTEX_TIMEOUT_MS)) == pdTRUE) {
    drawStaticUI();
    updateTFT();
    xSemaphoreGive(mutexTFT);
  }

  requestUiUpdate();

  xTaskCreatePinnedToCore(TaskReadSensors,     "ReadSensors",  4096, NULL, 2, &hTaskReadSensors,     1);
  xTaskCreatePinnedToCore(TaskProcessSensors,  "ProcessSensors", 4096, NULL, 2, &hTaskProcessSensors, 1);
  xTaskCreatePinnedToCore(TaskManualButton,    "ManualBtn",    2048, NULL, 3, &hTaskManualButton,    1);
  xTaskCreatePinnedToCore(TaskResetButton,     "ResetBtn",     2048, NULL, 3, &hTaskResetButton,     1);
  xTaskCreatePinnedToCore(TaskServoScan,       "ServoScan",    3072, NULL, 1, &hTaskServoScan,       0);
  xTaskCreatePinnedToCore(TaskAlarmControl,    "AlarmCtrl",    3072, NULL, 2, &hTaskAlarmControl,    1);
  xTaskCreatePinnedToCore(TaskRgbStatus,       "RgbStatus",    2048, NULL, 1, &hTaskRgbStatus,       0);
  xTaskCreatePinnedToCore(TaskDisplayTFT,      "DisplayTFT",   4096, NULL, 1, &hTaskDisplayTFT,      0);
  xTaskCreatePinnedToCore(TaskFirebaseCommand, "FirebaseCmd",  8192, NULL, 1, &hTaskFirebaseCommand, 0);

#if ENABLE_DEBUG_TASKS
  xTaskCreatePinnedToCore(TaskSerialMonitor,   "SerialMon",    8192, NULL, 1, &hTaskSerialMonitor,   0);
  xTaskCreatePinnedToCore(TaskHealthMonitor,   "HealthMon",    3072, NULL, 1, &hTaskHealthMonitor,   0);
#endif

  attachInterrupt(digitalPinToInterrupt(MANUAL_BTN_PIN), isrManualButton, FALLING);
  attachInterrupt(digitalPinToInterrupt(RESET_BTN_PIN), isrResetButton, FALLING);

  Serial.println("Smart Fire Alarm RTOS start.");
}

/* =========================================================
   loop
   ========================================================= */
void loop()
{
  vTaskDelay(pdMS_TO_TICKS(1000));
}
