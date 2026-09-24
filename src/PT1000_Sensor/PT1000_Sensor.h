#ifndef PT1000_SENSOR_H
#define PT1000_SENSOR_H

#include <Arduino.h>
#include <Adafruit_MAX31865.h>

// 默认引脚定义（可在调用时修改）
#define DEFAULT_CS_PIN    19
#define DEFAULT_MOSI_PIN  20
#define DEFAULT_MISO_PIN  21
#define DEFAULT_SCK_PIN   47

// PT1000 标准参数
#define RTD_NOMINAL 1000.0f    // PT1000 0℃电阻
#define R_REF       4300.0f    // 模块参考电阻 430Ω

class PT1000_Sensor {
private:
    Adafruit_MAX31865 max31865;  // 驱动对象
    float tempValue;             // 温度缓存
    uint8_t faultCode;           // 故障码

public:
    // 构造函数：使用默认引脚 / 自定义引脚
    PT1000_Sensor();
    PT1000_Sensor(int8_t cs, int8_t mosi, int8_t miso, int8_t sck);

    // 初始化函数（三线制）
    void begin();

    // 读取温度（自动更新故障码）
    float readTemperature();

    // 获取故障码
    uint8_t getFault();

    // 判断是否正常
    bool isNormal();
};

#endif