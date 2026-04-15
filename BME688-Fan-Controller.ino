#include <WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <ArduinoJson.h>
#include <Preferences.h>
#include <bsec2.h>
#include <math.h>
#include <time.h>
#include "config.h"

#ifndef TIME_ZONE
#define TIME_ZONE "CST-8"
#endif

#define SDA_PIN        8
#define SCL_PIN        9
#define FAN_PWM_PIN    5
#define FAN_TACH_PIN   4

// ===== MQTT Topics: sensor publish =====
const char* TOPIC_TEMP       = "modbus-mqtt/sensor/bme680_temperature";
const char* TOPIC_HUM        = "modbus-mqtt/sensor/bme680_humidity";
const char* TOPIC_PRES       = "modbus-mqtt/sensor/bme680_pressure";
const char* TOPIC_GAS        = "modbus-mqtt/sensor/bme680_gas";
const char* TOPIC_IAQ        = "modbus-mqtt/sensor/bme680_iaq";
const char* TOPIC_JSON       = "modbus-mqtt/sensor/esp32_telemetry";
const char* TOPIC_STATUS     = "modbus-mqtt/sensor/bme680_status";
const char* TOPIC_IAQ_ACC    = "modbus-mqtt/sensor/bme680_iaq_accuracy";
const char* TOPIC_STATIC_IAQ = "modbus-mqtt/sensor/bme680_static_iaq";
const char* TOPIC_CO2_EQ     = "modbus-mqtt/sensor/bme680_co2_eq";
const char* TOPIC_BVOC_EQ    = "modbus-mqtt/sensor/bme680_bvoc_eq";
const char* TOPIC_FAN_SPEED  = "modbus-mqtt/sensor/fan_speed_percent";
const char* TOPIC_FAN_RPM    = "modbus-mqtt/sensor/fan_rpm";
const char* TOPIC_FAN_MODE_STATE = "modbus-mqtt/sensor/fan_mode";

// ===== MQTT Topics: control subscribe =====
const char* TOPIC_FAN_MODE_CTRL = "modbus-mqtt/control/fan_mode";
const char* TOPIC_FAN_SET_CTRL  = "modbus-mqtt/control/fan_set_percent";

// ===== PWM =====
const int pwmFreq = 25000;      // 4线PWM风扇推荐 25kHz
const int pwmResolution = 8;    // 0~255

// ===== BSEC =====
#define SAMPLE_RATE BSEC_SAMPLE_RATE_LP
#define ARRAY_LEN(x) (sizeof(x) / sizeof((x)[0]))

// ===== Timing =====
static const uint32_t PUBLISH_INTERVAL_MS = 5000;
static const uint32_t RPM_CALC_INTERVAL_MS = 2000;

WiFiClient espClient;
PubSubClient mqtt(espClient);
Bsec2 envSensor;
Preferences prefs;

uint32_t lastPublishMs = 0;
uint32_t lastStateSaveMs = 0;
uint32_t lastRpmCalcMs = 0;
int lastAccSaved = -1;

uint8_t bsecState[BSEC_MAX_STATE_BLOB_SIZE];

volatile uint32_t tachCount = 0;
uint32_t lastTachCount = 0;
float fanRPM = 0.0f;

bool fanAutoMode = true;
int fanManualPercent = 50;
int g_fanPercent = 30;

struct SensorData {
  float temp = NAN;
  float hum = NAN;
  float pres_hpa = NAN;
  float gas_kohm = NAN;

  float iaq = NAN;
  int   iaq_acc = -1;
  float static_iaq = NAN;
  float co2_eq = NAN;
  float bvoc_eq = NAN;

  uint32_t ts_ms = 0;
} g_data;

volatile bool g_hasNew = false;

// ===== 工具函数 =====
static void publishFloat(const char* topic, float value, uint8_t decimals = 2) {
  if (isnan(value)) return;
  char buf[32];
  dtostrf(value, 0, decimals, buf);
  mqtt.publish(topic, buf, true);
}

static void publishInt(const char* topic, int value) {
  char buf[16];
  snprintf(buf, sizeof(buf), "%d", value);
  mqtt.publish(topic, buf, true);
}

static void publishText(const char* topic, const char* value) {
  mqtt.publish(topic, value, true);
}

static void errHalt() {
  while (1) {
    delay(1000);
  }
}

static void checkBsecStatus(Bsec2 bsec) {
  if (bsec.status < BSEC_OK) {
    Serial.print("BSEC error code: ");
    Serial.println(bsec.status);
    errHalt();
  } else if (bsec.status > BSEC_OK) {
    Serial.print("BSEC warning code: ");
    Serial.println(bsec.status);
  }

  if (bsec.sensor.status < BME68X_OK) {
    Serial.print("BME68X error code: ");
    Serial.println(bsec.sensor.status);
    errHalt();
  } else if (bsec.sensor.status > BME68X_OK) {
    Serial.print("BME68X warning code: ");
    Serial.println(bsec.sensor.status);
  }
}

// ===== PWM风扇控制 =====
void setFanSpeedPercent(int percent) {
  percent = constrain(percent, 0, 100);
  g_fanPercent = percent;

  int duty = map(percent, 0, 100, 0, 255);

  // 2N3904 开漏下拉，逻辑反相
  duty = 255 - duty;

  ledcWrite(FAN_PWM_PIN, duty);

  Serial.print("Fan speed -> ");
  Serial.print(percent);
  Serial.print("%, duty=");
  Serial.println(duty);
}

void controlFanByTemperature(float tempC) {
  if (isnan(tempC)) return;

  if (tempC >= 50.0f) {
    setFanSpeedPercent(100);
  } else if (tempC >= 45.0f) {
    setFanSpeedPercent(80);
  } else if (tempC >= 40.0f) {
    setFanSpeedPercent(60);
  } else if (tempC >= 35.0f) {
    setFanSpeedPercent(50);
  } else if (tempC >= 30.0f) {
    setFanSpeedPercent(40);
  } else if (tempC < 30.0f) {
    setFanSpeedPercent(25);   
  }
}

// ===== 风扇测速 =====
void IRAM_ATTR tachISR() {
  tachCount++;
}

void updateFanRPM() {
  uint32_t now = millis();
  if (now - lastRpmCalcMs < RPM_CALC_INTERVAL_MS) {
    return;
  }

  noInterrupts();
  uint32_t currentCount = tachCount;
  interrupts();

  uint32_t deltaCount = currentCount - lastTachCount;
  uint32_t deltaMs = now - lastRpmCalcMs;

  // 常见4线风扇：每转2个脉冲
  fanRPM = (deltaCount / 2.0f) * (60000.0f / deltaMs);

  lastTachCount = currentCount;
  lastRpmCalcMs = now;

  Serial.print("Fan RPM -> ");
  Serial.println(fanRPM);
}

// ===== BSEC状态保存/恢复 =====
static bool loadBsecState() {
  prefs.begin("bsec2", true);
  size_t n = prefs.getBytesLength("state");
  if (n != sizeof(bsecState)) {
    prefs.end();
    Serial.printf("BSEC state not found (len=%u)", (unsigned)n);
    return false;
  }
  prefs.getBytes("state", bsecState, sizeof(bsecState));
  prefs.end();

  bool ok = envSensor.setState(bsecState);
  Serial.printf("BSEC setState => %s", ok ? "OK" : "FAIL");
  return ok;
}

static void saveBsecStateIfNeeded(int iaqAcc) {
  const uint32_t intervalMs = 6UL * 60UL * 60UL * 1000UL;
  uint32_t now = millis();

  bool accUp = (iaqAcc >= 0 && iaqAcc > lastAccSaved);
  bool timeDue = (now - lastStateSaveMs) >= intervalMs;

  if (!accUp && !timeDue) return;

  bool ok = envSensor.getState(bsecState);
  if (!ok) {
    Serial.println("BSEC getState => FAIL");
    return;
  }

  prefs.begin("bsec2", false);
  prefs.putBytes("state", bsecState, sizeof(bsecState));
  prefs.end();

  lastStateSaveMs = now;
  if (accUp) lastAccSaved = iaqAcc;

  Serial.printf("BSEC state saved (acc=%d)", iaqAcc);
}

// ===== BSEC回调 =====
void newDataCallback(const bme68xData data, const bsecOutputs outputs, Bsec2 bsec) {
  (void)data; (void)bsec;
  if (!outputs.nOutputs) return;

  g_data.ts_ms = millis();

  for (uint8_t i = 0; i < outputs.nOutputs; i++) {
    const bsecData o = outputs.output[i];
    switch (o.sensor_id) {
      case BSEC_OUTPUT_RAW_TEMPERATURE: g_data.temp = o.signal; break;
      case BSEC_OUTPUT_RAW_HUMIDITY:    g_data.hum = o.signal; break;
      case BSEC_OUTPUT_RAW_PRESSURE:    g_data.pres_hpa = o.signal; break; // 如果显示成 101000，请改成 o.signal / 100.0f
      case BSEC_OUTPUT_RAW_GAS:         g_data.gas_kohm = o.signal / 1000.0f; break;
      case BSEC_OUTPUT_IAQ:             g_data.iaq = o.signal; g_data.iaq_acc = (int)o.accuracy; break;
      case BSEC_OUTPUT_STATIC_IAQ:      g_data.static_iaq = o.signal; break;
      case BSEC_OUTPUT_CO2_EQUIVALENT:  g_data.co2_eq = o.signal; break;
      case BSEC_OUTPUT_BREATH_VOC_EQUIVALENT: g_data.bvoc_eq = o.signal; break;
      default: break;
    }
  }

  g_hasNew = true;
}

// ===== WiFi =====
void connectWiFi() {
  Serial.print("Connecting WiFi: ");
  Serial.println(WIFI_SSID);

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWD);

  // 我的这块ESP32-C3超迷你板的天线设计存在缺陷，个别情况下连不上wifi，故这里降低一下限号强度
  WiFi.setTxPower(WIFI_POWER_8_5dBm);

  int retry = 0;
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
    retry++;

    if (retry > 40) {
      Serial.println("\nWiFi FAILED!");
      Serial.print("Status = ");
      Serial.println(WiFi.status());
      return;
    }
  }

  Serial.println();
  Serial.print("WiFi connected, IP = ");
  Serial.println(WiFi.localIP());
}

// ===== MQTT回调 =====
void mqttCallback(char* topic, byte* payload, unsigned int length) {
  String msg;
  msg.reserve(length);
  for (unsigned int i = 0; i < length; i++) {
    msg += (char)payload[i];
  }

  Serial.print("MQTT recv: ");
  Serial.print(topic);
  Serial.print(" = ");
  Serial.println(msg);

  if (String(topic) == TOPIC_FAN_MODE_CTRL) {
    msg.trim();
    msg.toLowerCase();

    if (msg == "auto") {
      fanAutoMode = true;
      publishText(TOPIC_FAN_MODE_STATE, "auto");
    } else if (msg == "manual") {
      fanAutoMode = false;
      publishText(TOPIC_FAN_MODE_STATE, "manual");
    }
  }

  if (String(topic) == TOPIC_FAN_SET_CTRL) {
    int p = msg.toInt();
    p = constrain(p, 0, 100);
    fanManualPercent = p;

    if (!fanAutoMode) {
      setFanSpeedPercent(fanManualPercent);
    }
  }
}

// ===== MQTT =====
void connectMQTT() {
  mqtt.setServer(MQTT_HOST, MQTT_PORT);
  mqtt.setCallback(mqttCallback);
  // 增加缓冲区以容纳较大的 JSON 负载
  mqtt.setBufferSize(512);

  while (!mqtt.connected()) {
    String clientId = "esp32c3-bme688-" + String((uint32_t)ESP.getEfuseMac(), HEX);

    Serial.print("Connecting MQTT... ");
    bool ok = mqtt.connect(
      clientId.c_str(),
      MQTT_USER,
      MQTT_PASSWD,
      TOPIC_STATUS,
      1,
      true,
      "offline"
    );

    if (ok) {
      Serial.println("OK");
      mqtt.publish(TOPIC_STATUS, "online", true);
      mqtt.subscribe(TOPIC_FAN_MODE_CTRL);
      mqtt.subscribe(TOPIC_FAN_SET_CTRL);
      publishText(TOPIC_FAN_MODE_STATE, fanAutoMode ? "auto" : "manual");
    } else {
      Serial.print("failed, rc=");
      Serial.print(mqtt.state());
      Serial.println(", retry in 2s");
      delay(2000);
    }
  }
}

// ===== BSEC初始化 =====
bool initBSEC() {
  Wire.begin(SDA_PIN, SCL_PIN);

  bsecSensor sensorList[] = {
    BSEC_OUTPUT_IAQ,
    BSEC_OUTPUT_STATIC_IAQ,
    BSEC_OUTPUT_CO2_EQUIVALENT,
    BSEC_OUTPUT_BREATH_VOC_EQUIVALENT,
    BSEC_OUTPUT_RAW_TEMPERATURE,
    BSEC_OUTPUT_RAW_HUMIDITY,
    BSEC_OUTPUT_RAW_PRESSURE,
    BSEC_OUTPUT_RAW_GAS
  };

  if (!envSensor.begin(BME68X_I2C_ADDR_LOW, Wire)) {
    checkBsecStatus(envSensor);
    return false;
  }
  loadBsecState();
  if (!envSensor.updateSubscription(sensorList, ARRAY_LEN(sensorList), SAMPLE_RATE)) {
    checkBsecStatus(envSensor);
  }
  envSensor.attachCallback(newDataCallback);
  return true;
}

// ===== 发布数据 =====
void publishTelemetry() {
  publishText(TOPIC_FAN_MODE_STATE, fanAutoMode ? "auto" : "manual");

  char timeStr[24] = {0};
  struct tm timeinfo;
  if (getLocalTime(&timeinfo)) {
    strftime(timeStr, sizeof(timeStr), "%Y-%m-%d %H:%M:%S", &timeinfo);
  } else {
    snprintf(timeStr, sizeof(timeStr), "1970-01-01 00:00:00");
  }
  
  // 打印调试信息到串口，不受 sensor 数据是否就绪的限制
  Serial.print("Current Time: "); Serial.println(timeStr);

  if (g_data.ts_ms == 0 || isnan(g_data.temp) || isnan(g_data.hum)) {
    Serial.println("Sensor data not ready, skipping telemetry publish.");
    return;
  }

  // 发布具体主题，方便 Node-RED 单独订阅
  publishFloat(TOPIC_TEMP, g_data.temp);
  publishFloat(TOPIC_HUM, g_data.hum);
  publishFloat(TOPIC_PRES, g_data.pres_hpa);
  publishFloat(TOPIC_GAS, g_data.gas_kohm);
  publishFloat(TOPIC_IAQ, g_data.iaq);
  publishInt(TOPIC_IAQ_ACC, g_data.iaq_acc);
  publishFloat(TOPIC_STATIC_IAQ, g_data.static_iaq);
  publishFloat(TOPIC_CO2_EQ, g_data.co2_eq);
  publishFloat(TOPIC_BVOC_EQ, g_data.bvoc_eq);
  publishInt(TOPIC_FAN_SPEED, g_fanPercent);
  publishFloat(TOPIC_FAN_RPM, fanRPM);

  JsonDocument doc;
  doc["timestamp"] = timeStr;
  doc["ts_ms"] = g_data.ts_ms;
  doc["temp"] = g_data.temp;
  doc["hum"] = g_data.hum;
  doc["pres"] = g_data.pres_hpa;
  doc["gas_kohm"] = g_data.gas_kohm;
  doc["iaq"] = g_data.iaq;
  doc["iaq_acc"] = g_data.iaq_acc;
  doc["static_iaq"] = g_data.static_iaq;
  doc["co2_eq"] = g_data.co2_eq;
  doc["bvoc_eq"] = g_data.bvoc_eq;
  doc["fan_percent"] = g_fanPercent;
  doc["fan_rpm"] = fanRPM;
  doc["fan_mode"] = fanAutoMode ? "auto" : "manual";

  char payload[384];
  size_t n = serializeJson(doc, payload, sizeof(payload));
  bool pubOk = mqtt.publish(TOPIC_JSON, payload, true);

  Serial.println("----- BSEC2 + FAN -----");
  Serial.print("MQTT JSON Publish: "); Serial.println(pubOk ? "SUCCESS" : "FAILED (Check Buffer Size)");
  Serial.print("JSON Length: "); Serial.println(n);
  Serial.print("Temp: "); Serial.println(g_data.temp);
  Serial.print("Hum : "); Serial.println(g_data.hum);
  Serial.print("IAQ : "); Serial.print(g_data.iaq);
  Serial.print(" (acc="); Serial.print(g_data.iaq_acc); Serial.println(")");
  Serial.print("Fan : "); Serial.print(g_fanPercent); Serial.println("%");
  Serial.print("RPM : "); Serial.println(fanRPM);
}

void setup() {
  Serial.begin(115200);
  delay(2000);

  Serial.println();
  Serial.println("=== ESP32-C3 BME688 BSEC2 PWM FAN MQTT ===");

  // PWM初始化
  if (!ledcAttach(FAN_PWM_PIN, pwmFreq, pwmResolution)) {
    Serial.println("PWM attach failed!");
    while (1) delay(1000);
  }
  Serial.println("PWM attach OK");

  // Tach输入初始化
  pinMode(FAN_TACH_PIN, INPUT_PULLUP);
  attachInterrupt(FAN_TACH_PIN, tachISR, FALLING);
  lastRpmCalcMs = millis();

  // 上电先低速
  setFanSpeedPercent(30);

  if (!initBSEC()) {
    Serial.println("BSEC2 init failed!");
    while (1) delay(1000);
  }
  Serial.println("BSEC2 init OK");

  connectWiFi();
  configTzTime(TIME_ZONE, "pool.ntp.org", "time.nist.gov");
  connectMQTT();
}

void loop() {
  if (WiFi.status() != WL_CONNECTED) {
    connectWiFi();
  }

  if (!mqtt.connected()) {
    connectMQTT();
  }

  mqtt.loop();

  if (!envSensor.run()) {
    checkBsecStatus(envSensor);
  }

  updateFanRPM();

  if (g_hasNew) {
    g_hasNew = false;

    if (fanAutoMode) {
      controlFanByTemperature(g_data.temp);
    } else {
      setFanSpeedPercent(fanManualPercent);
    }

    saveBsecStateIfNeeded(g_data.iaq_acc);
  }

  if (millis() - lastPublishMs >= PUBLISH_INTERVAL_MS) {
    lastPublishMs = millis();
    publishTelemetry();
  }
}