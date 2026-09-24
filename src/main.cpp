#include <Arduino.h>
#include <ArduinoJson.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "PT1000_Sensor/PT1000_Sensor.h"
#include "HX711_Sensor/HX711_Sensor.h"
#include "AccelStepper_Motor/AccelStepper_Motor.h"
#include "ESPNow_SerialBridge/ESPNow_SerialBridge.h"

static constexpr double HX711_DEFAULT_CALIB_A = -2.082522834641083e-13;
static constexpr double HX711_DEFAULT_CALIB_B = -7.341127721143217e-04;
static constexpr double HX711_DEFAULT_CALIB_C = -161.24014013101178;

static constexpr double HX711_DEFAULT_CALIB_D = -4.275183570420112e-12;
static constexpr double HX711_DEFAULT_CALIB_E = -4.642817410247923e-04;
static constexpr double HX711_DEFAULT_CALIB_F = -27.60143162547832;

// ==================== 硬件定义 ====================
// 侧面称重 = F压；竖直称重 = F支
HX711_Sensor weightSensor1(35, 36, HX711_DEFAULT_CALIB_A, HX711_DEFAULT_CALIB_B, HX711_DEFAULT_CALIB_C);
HX711_Sensor weightSensor2(1, 2, HX711_DEFAULT_CALIB_D, HX711_DEFAULT_CALIB_E, HX711_DEFAULT_CALIB_F);

AccelStepper_Motor motor1(6, 4, 5, 1);     // M1 往复
AccelStepper_Motor motor2(40, 41, 42, 1); // M2 运动控制
AccelStepper_Motor motor3(17, 15, 16, 1); // M3 竖直升降
AccelStepper_Motor motor4(10, 11, 12, 1); // M4 控制磁吸装置上下
#define MAGNET_PIN 8

ESPNow_SerialBridge espNow(peerMac, WIFI_CHANNEL, WIFI_MODE);
PT1000_Sensor pt1000;

// ==================== 常量与参数 ====================
static const unsigned long REPORT_MS = 200;
static const unsigned long TEMP_READ_MS = 200; // PT1000 温度传感器采样间隔
static const float G_TO_N = 0.00980665f;
static const unsigned long DEFAULT_REPORT_MS = 1000;
static const long M1_TRIP_STEPS = 160000;
static const unsigned long M1_PAUSE_MS = 500;

// ==================== 全局状态与互斥锁 ====================
SemaphoreHandle_t dataMutex = NULL;

unsigned long lastReportMs = 0;
unsigned long lastTempReadMs = 0;
unsigned long reportIntervalMs = DEFAULT_REPORT_MS;

float currentTemp = 0.0f;

int motor2TargetSpeed = 0;
int motor2Dir = 1;
bool motor2Running = false;

int motor3TargetSpeed = 0;
int motor3Dir = 1;
bool motor3Running = false;

int motor4TargetSpeed = 2000; // M4 默认恒速速度
int motor4Dir = 1;
bool motor4Running = false;

bool motor1Active = false;
bool motor1GoingOut = true;
unsigned long motor1PauseUntil = 0;

float lastLateralN = 0.0f;
float lastForwardN = 0.0f;

// ==================== 电磁铁开关控制 ====================

void magnetOn()
{
    digitalWrite(MAGNET_PIN, HIGH);
}

void magnetOff()
{
    digitalWrite(MAGNET_PIN, LOW);
}

// ==================== 辅助与通信函数 ====================

/** 对上位机：仅输出单行 JSON（Web Serial 按行解析） */
void emitToHost(const String &json)
{
    Serial.println(json);
}

void emitError(const char *error, int exp = 0)
{
    JsonDocument doc;
    doc["ok"] = false;
    doc["cmd"] = "error";
    doc["error"] = error;
    if (exp != 0)
    {
        doc["exp"] = exp;
    }
    String out;
    serializeJson(doc, out);
    emitToHost(out);
}

bool looksLikeJsonObject(const String &s)
{
    for (unsigned int i = 0; i < s.length(); i++)
    {
        const char c = s[i];
        if (c == ' ' || c == '\t' || c == '\r')
        {
            continue;
        }
        return c == '{';
    }
    return false;
}

String buildReportJson()
{
    JsonDocument doc;
    doc["exp"] = 2;

    if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE)
    {
        doc["temp"] = currentTemp;
        doc["motor2_speed"] = motor2Running ? (motor2TargetSpeed * motor2Dir) : motor2TargetSpeed;
        doc["motor3_speed"] = motor3Running ? (motor3TargetSpeed * motor3Dir) : motor3TargetSpeed;
        doc["lateral_pressure"] = lastLateralN;
        doc["forward_pressure"] = lastForwardN;
        xSemaphoreGive(dataMutex);
    }

    String out;
    serializeJson(doc, out);
    return out;
}

// ==================== 执行机构与传感器更新 ====================

void updateTemperature()
{
    const unsigned long now = millis();
    if (now - lastTempReadMs < TEMP_READ_MS)
    {
        return;
    }
    lastTempReadMs = now;

    float readVal = pt1000.readTemperature();

    if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE)
    {
        currentTemp = readVal;
        xSemaphoreGive(dataMutex);
    }
}

void updateSensors()
{
    const float w1g = weightSensor1.update();
    const float w2g = weightSensor2.update();

    if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE)
    {
        lastLateralN = w1g * G_TO_N;
        lastForwardN = w2g * G_TO_N;
        xSemaphoreGive(dataMutex);
    }
}

void applyMotor2Speed()
{
    if (motor2Running)
    {
        motor2.setSpeed((float)(motor2TargetSpeed * motor2Dir));
    }
}

void applyMotor3Speed()
{
    if (motor3Running)
    {
        motor3.setSpeed((float)(motor3TargetSpeed * motor3Dir));
    }
}

void applyMotor4Speed()
{
    if (motor4Running)
    {
        motor4.setSpeed((float)(motor4TargetSpeed * motor4Dir));
    }
}

void startMotor1Once()
{
    motor1Active = true;
    motor1GoingOut = true;
    motor1PauseUntil = 0;
    motor1.move(M1_TRIP_STEPS);
}

void serviceMotors()
{
    if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE)
    {
        if (motor2Running)
        {
            motor2.runSpeed();
        }
        if (motor3Running)
        {
            motor3.runSpeed();
        }
        if (motor4Running)
        {
            motor4.runSpeed();
        }

        if (motor1Active)
        {
            if (motor1PauseUntil != 0)
            {
                if (millis() < motor1PauseUntil)
                {
                    xSemaphoreGive(dataMutex);
                    return;
                }
                motor1PauseUntil = 0;

                if (motor1GoingOut)
                {
                    motor1GoingOut = false;
                    motor1.move(-M1_TRIP_STEPS);
                }
                else
                {
                    motor1Active = false;
                }
                xSemaphoreGive(dataMutex);
                return;
            }

            motor1.run();
            if (motor1.distanceToGo() == 0)
            {
                motor1PauseUntil = millis() + M1_PAUSE_MS;
            }
        }
        xSemaphoreGive(dataMutex);
    }
}

// ==================== 指令解析与串口通信 ====================

void handleCommand(const String &line)
{
    if (line == "t")
    {
        if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE)
        {
            weightSensor1.manualTare();
            weightSensor2.manualTare();
            xSemaphoreGive(dataMutex);
        }
        emitToHost("{\"ok\":true,\"cmd\":\"tare\",\"action\":\"done\"}");
        return;
    }

    JsonDocument doc;
    if (deserializeJson(doc, line))
    {
        return;
    }

    const int exp = doc["exp"] | 0;

    // 实验一：透传给斜面板
    if (exp == 1)
    {
        if (!espNow.send(line.c_str()))
        {
            emitError("espnow_forward_failed", 1);
        }
        return;
    }

    if (exp != 2)
    {
        return;
    }

    const char *cmd = doc["cmd"] | "";
    const char *action = doc["action"] | "";
    const int speed = doc["speed"] | 0;

    if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE)
    {
        if (strcmp(cmd, "motor1") == 0)
        {
            if (strcmp(action, "start") == 0)
            {
                startMotor1Once();
            }
        }
        else if (strcmp(cmd, "motor2") == 0)
        {
            if (strcmp(action, "set_speed") == 0)
            {
                motor2TargetSpeed = speed;
                applyMotor2Speed();
            }
            else if (strcmp(action, "forward") == 0)
            {
                motor2Dir = 1;
                motor2Running = true;
                applyMotor2Speed();
            }
            else if (strcmp(action, "back") == 0)
            {
                motor2Dir = -1;
                motor2Running = true;
                applyMotor2Speed();
            }
            else if (strcmp(action, "stop") == 0)
            {
                motor2Running = false;
                motor2.stop();
            }
        }
        else if (strcmp(cmd, "motor3") == 0)
        {
            if (strcmp(action, "set_speed") == 0)
            {
                motor3TargetSpeed = speed;
                applyMotor3Speed();
            }
            else if (strcmp(action, "forward") == 0)
            {
                motor3Dir = -1;
                motor3Running = true;
                applyMotor3Speed();
            }
            else if (strcmp(action, "down") == 0)
            {
                motor3Dir = 1;
                motor3Running = true;
                applyMotor3Speed();
            }
            else if (strcmp(action, "stop") == 0)
            {
                motor3Running = false;
                motor3.stop();
            }
        }
        else if (strcmp(cmd, "magnet") == 0)
        {
            // 磁吸装置升降（M4 控制）
            if (strcmp(action, "up") == 0)
            {
                motor4Dir = -1;
                motor4Running = true;
                applyMotor4Speed();
            }
            else if (strcmp(action, "down") == 0)
            {
                motor4Dir = 1;
                motor4Running = true;
                applyMotor4Speed();
            }
            else if (strcmp(action, "stop") == 0)
            {
                motor4Running = false;
                motor4.stop();
            }
            // 电磁铁通断电控制
            else if (strcmp(action, "on") == 0)
            {
                magnetOn();
            }
            else if (strcmp(action, "off") == 0)
            {
                magnetOff();
            }
        }
        else if (strcmp(cmd, "report") == 0)
        {
            if (strcmp(action, "set_interval") == 0)
            {
                const int intervalMs = doc["interval_ms"] | doc["interval"] | 0;
                if (intervalMs >= 100)
                {
                    reportIntervalMs = (unsigned long)intervalMs;
                    lastReportMs = millis();
                }
                else
                {
                    emitError("invalid_interval", 2);
                }
            }
        }
        xSemaphoreGive(dataMutex);
    }
}

void handleSerial()
{
    while (Serial.available() > 0)
    {
        String line = Serial.readStringUntil('\n');
        line.trim();
        if (line.length() == 0)
        {
            continue;
        }
        handleCommand(line);
    }
}

/** 斜面经 ESP-NOW 上报：仅透传 JSON 对象给上位机 */
void forwardEspNowToSerial()
{
    while (espNow.pendingCount() > 0)
    {
        String data = espNow.getReceivedData();
        data.trim();
        if (data.length() == 0)
        {
            break;
        }
        if (!looksLikeJsonObject(data))
        {
            continue;
        }
        emitToHost(data);
    }
}

void maybeReport()
{
    const unsigned long now = millis();
    if (now - lastReportMs < reportIntervalMs)
    {
        return;
    }
    lastReportMs = now;
    emitToHost(buildReportJson());
}

// ==================== FreeRTOS 任务函数 ====================

void taskMotors(void *pvParameters)
{
    (void)pvParameters;
    for (;;)
    {
        serviceMotors();
        vTaskDelay(pdMS_TO_TICKS(1)); // 给出CPU时间片，防止看门狗超时
    }
}

void taskSensors(void *pvParameters)
{
    (void)pvParameters;
    for (;;)
    {
        updateSensors();
        updateTemperature();
        vTaskDelay(pdMS_TO_TICKS(10)); // 传感器采样轮询周期
    }
}

// ==================== 初始化与主循环 ====================

void setup()
{
    Serial.begin(115200);
    Serial.setTimeout(20);

    // 创建互斥锁
    dataMutex = xSemaphoreCreateMutex();

    pinMode(MAGNET_PIN, OUTPUT);
    magnetOff();

    pt1000.begin();

    weightSensor1.begin(0.2f, 0.15f);
    weightSensor2.begin(0.2f, 0.15f);
    weightSensor1.manualTare();
    weightSensor2.manualTare();

    motor1.begin(20000, 10000);
    motor2.begin(4000, 1000);
    motor3.begin(4000, 1000);
    motor4.begin(4000, 1000);

    if (!espNow.begin())
    {
        emitError("espnow_init_failed");
    }
    else
    {
        JsonDocument boot;
        boot["ok"] = true;
        boot["cmd"] = "hub";
        boot["action"] = "ready";
        boot["mac"] = espNow.getLocalMac();
        String out;
        serializeJson(boot, out);
        emitToHost(out);
    }

    // 创建 FreeRTOS 任务
    xTaskCreatePinnedToCore(taskMotors, "TaskMotors", 4096, NULL, 5, NULL, 1);
    xTaskCreatePinnedToCore(taskSensors, "TaskSensors", 4096, NULL, 2, NULL, 0);
}

void loop()
{
    forwardEspNowToSerial();
    handleSerial();
    maybeReport();
    vTaskDelay(pdMS_TO_TICKS(1));
}