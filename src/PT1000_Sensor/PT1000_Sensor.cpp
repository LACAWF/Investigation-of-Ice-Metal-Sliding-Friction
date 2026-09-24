#include "PT1000_Sensor.h"

// 默认构造函数（使用定义的默认引脚）
PT1000_Sensor::PT1000_Sensor()
  : max31865(DEFAULT_CS_PIN, DEFAULT_MOSI_PIN, DEFAULT_MISO_PIN, DEFAULT_SCK_PIN)
{
    tempValue = 0.0f;
    faultCode = 0;
}

// 自定义引脚构造函数
PT1000_Sensor::PT1000_Sensor(int8_t cs, int8_t mosi, int8_t miso, int8_t sck)
  : max31865(cs, mosi, miso, sck)
{
    tempValue = 0.0f;
    faultCode = 0;
}

// 初始化传感器（三线制模式）
void PT1000_Sensor::begin() {
    max31865.begin(MAX31865_3WIRE);
}

// 读取温度 + 自动更新故障码
float PT1000_Sensor::readTemperature() {
    tempValue = max31865.temperature(RTD_NOMINAL, R_REF);
    faultCode = max31865.readFault();
    return tempValue;
}

// 获取当前故障码
uint8_t PT1000_Sensor::getFault() {
    return faultCode;
}

// 判断传感器是否正常工作
bool PT1000_Sensor::isNormal() {
    return (faultCode == 0);
}