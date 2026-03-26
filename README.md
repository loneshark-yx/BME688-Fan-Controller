# BME688 智能风扇控制器 (ESP32-C3)

本项目基于 ESP32-C3-SuperMini 开发板，结合 Bosch BME688 传感器，实现了一个智能环境监控与风扇自动控制系统。它能够实时监测环境温度、湿度、气压、空气质量（IAQ）等指标，并通过 MQTT 将数据上传，同时根据温度自动调节风扇转速，或通过 MQTT 进行远程手动控制。

## 功能特性

- **环境监测**：集成 Bosch BSEC2 库，提供温度、湿度、气压、气体电阻、IAQ（空气质量指数）、CO2 等效值和 bVOC 等效值。
- **智能风扇控制**：
  - **自动模式**：根据传感器读取的温度，通过预设算法自动调节风扇 PWM 占空比。
  - **手动模式**：通过 MQTT 远程设置风扇转速百分比（0-100%）。
- **转速监测**：实时捕获风扇 TACH 信号，计算并反馈风扇实时 RPM。
- **数据同步**：通过 MQTT 协议发布传感器数据和风扇状态，支持接收控制命令。
- **状态持久化**：自动保存 BSEC 算法状态到 ESP32 Flash 中，确保断电重启后空气质量校准信息不丢失。

## 硬件清单

- **开发板**：ESP32-C3-SuperMini
- **传感器**：Bosch BME688 (使用 I2C 接口)
- **风扇**：12V 4线 PWM 风扇
- **驱动元件**：2N3904 NPN 三极管、1kΩ 电阻、10kΩ 电阻
- **电源**：12V 直流电源（用于风扇）及 5V 或 3.3V（用于 ESP32）

## 硬件接线

### 1. BME688 传感器 (I2C 模式)

| BME688 引脚 | ESP32-C3 引脚 | 说明 |
| :--- | :--- | :--- |
| **VIN** | 3V3 | 电源正极 |
| **GND** | GND | 电源地 |
| **SDI** | GPIO8 | I2C SDA |
| **SCK** | GPIO9 | I2C SCL |
| **CS** | 3V3 | 拉高以选择 I2C 模式 |
| **SDO** | GND | 接地使 I2C 地址为 0x76 |

### 2. 风扇与驱动电路 (2N3904)

**注意：ESP32 的 GND 必须与 12V 电源负极共地。**

#### 风扇连接：
- **红线**：接 12V 电源正极
- **黑线**：接 12V 电源负极 (GND)
- **黄线 (Tach)**：接 ESP32 **GPIO4**，同时通过 **10kΩ 电阻上拉至 3.3V**
- **蓝线 (PWM)**：接 2N3904 的**集电极 (C)**

#### 2N3904 三极管：
- **基极 (B)**：通过 **1kΩ 电阻** 接 ESP32 **GPIO5**
- **集电极 (C)**：接风扇蓝线 (PWM)
- **发射极 (E)**：接 GND

## 软件配置

在代码中，您可能需要根据实际环境修改以下配置：

- `WIFI_SSID` / `WIFI_PASSWD`: WiFi 名称和密码。
- `MQTT_HOST` / `MQTT_PORT`: MQTT 服务器地址和端口。
- `MQTT_USER` / `MQTT_PASSWD`: MQTT 认证信息。

## MQTT 话题说明

### 发布 (Telemetry)
- `modbus-mqtt/sensor/bme680_temperature`: 温度 (°C)
- `modbus-mqtt/sensor/bme680_humidity`: 湿度 (%)
- `modbus-mqtt/sensor/bme680_iaq`: 空气质量指数 (0-500)
- `modbus-mqtt/sensor/fan_rpm`: 风扇实时转速
- `modbus-mqtt/sensor/fan_speed_percent`: 当前风扇功率百分比
- `modbus-mqtt/sensor/esp32_telemetry`: 完整的 JSON 格式数据

### 订阅 (Control)
- `modbus-mqtt/control/fan_mode`: 设置模式 (`auto` 或 `manual`)
- `modbus-mqtt/control/fan_set_percent`: 手动模式下设置转速 (0-100)

## 依赖库

- [BSEC2 Software Library](https://github.com/boschsensortec/Bosch-BSEC2-Library)
- [PubSubClient](https://github.com/knolleary/pubsubclient)
- [ArduinoJson](https://arduinojson.org/)

## 许可证

本项目遵循 MIT 许可证。
