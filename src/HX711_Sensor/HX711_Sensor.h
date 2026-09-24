#ifndef HX711_SENSOR_H
#define HX711_SENSOR_H

#include <Arduino.h>
#include <HX711.h>

class HX711_Sensor
{
public:
    /**
     * @brief 构造函数
     * @param HX711_DOUT_PIN HX711数据引脚
     * @param HX711_SCK_PIN HX711时钟引脚
     * @param HX711_CALIB_A 二次项系数
     * @param HX711_CALIB_B 一次项系数
     * @param HX711_CALIB_C 常数项
     */
    HX711_Sensor(
        uint8_t HX711_DOUT_PIN ,
        uint8_t HX711_SCK_PIN ,
        double HX711_CALIB_A ,
        double HX711_CALIB_B ,
        double HX711_CALIB_C );

    /**
     * @brief 初始化传感器
     * @param deadband 死区范围 (g)
     * @param emaAlpha 一阶低通（EMA）的平滑系数，决定“丝滑程度 vs 响应速度”(0-1)
     * emaAlpha 越小：曲线更平滑，但响应更慢，emaAlpha 越大：响应更快，但曲线更“抖
     */
    void begin(float deadband = 0.01, float emaAlpha = 0.15f);

    /**
     * @brief 读取并更新重量，返回用于显示的净重（负数置 0，小于死区置 0）
     * @return 显示用重量 (g)
     */
    float update();

    /**
     * @brief 获取当前计算出的净重（原始数据，不经过死区滤波）
     * @return 重量 (g)
     */
    float getNetWeight();

    /**
     * @brief 手动触发去皮
     */
    void manualTare();

    /**
     * @brief 旧版简易读取方式，等效原生 scale.get_units(1)
     * @return 单次采样重量
     */
    float simpleRead();

    /**
     * @brief 简易模式去皮（对应原生 scale.tare()）
     */
    void simpleTare(float scaleFactor = 420.0f);

private:
    // 15 点截尾均值：丢掉最小 3 个 + 最大 3 个，取中间 9 个平均
    static constexpr int SAMPLE_COUNT = 15;
    static constexpr int TRIM_COUNT = 3;
    static constexpr int MIDDLE_COUNT = SAMPLE_COUNT - 2 * TRIM_COUNT; // 9
    // 兼容旧版简易单采样模式（对应你原来 scale.get_units(1) 逻辑）
    float _simpleRawWeight = 0.0f;

    // 引脚
    uint8_t _doutPin;
    uint8_t _sckPin;

    // 二次多项式标定系数
    double _a;
    double _b;
    double _c;

    // 配置参数
    float _deadband;

    // 状态变量
    float _tareOffset;
    bool _isTared;
    long _currentAdc;
    float _lastNetWeight;

    // 截尾均值滑动缓冲区（环形队列）
    long _adcBuf[SAMPLE_COUNT];
    uint8_t _bufPos;
    uint8_t _bufCount;

    // 一阶低通（EMA）
    float _emaAlpha;
    float _emaWeight;
    bool _emaInitialized;

    // HX711 驱动对象
    HX711 _scale;

    // 阻塞式填充缓冲区：用于初始化/手动去皮（不用于实时 update）
    void fillAdcBufferBlocking();

    // 从当前环形缓冲区计算截尾均值（返回滤波后的 ADC）
    long computeTrimmedMeanADCFromBuffer() const;

    /**
     * @brief 内部函数：使用二次公式计算重量
     * @param rawAdc 原始ADC值
     * @return 计算出的毛重
     */
    float calculateWeight(long rawAdc);
};

#endif // HX711_SENSOR_H
