#include "HX711_Sensor.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <algorithm>

// 构造函数
HX711_Sensor::HX711_Sensor(uint8_t doutPin, uint8_t sckPin, double a, double b, double c)
    : _doutPin(doutPin), _sckPin(sckPin), _a(a), _b(b), _c(c),
      _tareOffset(0.0), _isTared(false), _currentAdc(0), _lastNetWeight(0.0),
      _bufPos(0), _bufCount(0),
      _emaAlpha(0.15f), _emaWeight(0.0f), _emaInitialized(false)
{
}

// 初始化：无等待，直接准备
void HX711_Sensor::begin(float deadband, float emaAlpha)
{
    _deadband = deadband;
    _emaAlpha = emaAlpha;
    _scale.begin(_doutPin, _sckPin);
    _scale.set_gain(128);

    // 初始化滑动缓冲区：阻塞式快速填满（只发生在 setup/setup 早期）
    fillAdcBufferBlocking();

    // 以当前“截尾均值滤波后”的 ADC 作为去皮基准
    _currentAdc = computeTrimmedMeanADCFromBuffer();
    float initWeight = calculateWeight(_currentAdc);
    _tareOffset = initWeight;

    // 去皮后初始净重应为 0，EMA 直接从 0 开始
    _emaWeight = 0.0f;
    _emaInitialized = true;
    _isTared = true;
}

// 主循环
float HX711_Sensor::update()
{
    // 非阻塞采样：只要当前已经 ready，就尽快读入缓冲区
    // 这样不会像“15 点批处理”那样卡住很久。
    const int MAX_READS_PER_CALL = 3; // 每次最多读 3 个新样本
    int reads = 0;
    while (reads < MAX_READS_PER_CALL && _scale.is_ready())
    {
        long adc = _scale.read(); // is_ready() 为真时，read() 内部等待会立刻返回
        _adcBuf[_bufPos] = adc;
        _bufPos = (_bufPos + 1) % SAMPLE_COUNT;
        if (_bufCount < SAMPLE_COUNT)
            _bufCount++;
        reads++;
    }

    // 缓冲区还没满时，无法稳定输出“15点截尾均值”
    if (_bufCount < SAMPLE_COUNT)
    {
        return _emaInitialized ? _emaWeight : 0.0f;
    }

    // 1) 底层：15 点截尾均值（丢 3 个最小 + 3 个最大，只均值中间 9 个）
    _currentAdc = computeTrimmedMeanADCFromBuffer();

    // 2) 二次标定 -> 净重
    float grossWeight = calculateWeight(_currentAdc);
    _lastNetWeight = grossWeight - _tareOffset;

    // 强制：负数归 0 + 死区
    float displayWeight = _lastNetWeight;
    if (displayWeight < 0)
        displayWeight = 0.0f;
    if (displayWeight < _deadband)
        displayWeight = 0.0f;

    // 3) 顶层：一阶低通（EMA）
    if (!_emaInitialized)
    {
        _emaWeight = displayWeight;
        _emaInitialized = true;
    }
    else
    {
        _emaWeight = (1.0f - _emaAlpha) * _emaWeight + _emaAlpha * displayWeight;
    }

    return _emaWeight;
}

// 获取净重
float HX711_Sensor::getNetWeight()
{
    return _lastNetWeight;
}

// 手动去皮
void HX711_Sensor::manualTare()
{
    // 为了保证手动去皮也用“15点截尾均值”，这里阻塞式确保缓冲区满
    if (_bufCount < SAMPLE_COUNT)
    {
        fillAdcBufferBlocking();
    }

    _currentAdc = computeTrimmedMeanADCFromBuffer();
    float currentGross = calculateWeight(_currentAdc);
    _tareOffset = currentGross;

    // 去皮后 EMA 从 0 开始
    _emaWeight = 0.0f;
    _emaInitialized = true;
}

// 阻塞式填充缓冲区（用于初始化/手动去皮）
void HX711_Sensor::fillAdcBufferBlocking()
{
    _bufPos = 0;
    _bufCount = 0;

    for (int i = 0; i < SAMPLE_COUNT; i++)
    {
        while (!_scale.is_ready())
        {
            vTaskDelay(1); // 让出 CPU，避免死等导致看门狗风险
        }
        long adc = _scale.read();
        _adcBuf[_bufPos] = adc;
        _bufPos = (_bufPos + 1) % SAMPLE_COUNT;
        _bufCount++;
    }
}

// 从当前环形缓冲区计算截尾均值 ADC（丢两端极值）
long HX711_Sensor::computeTrimmedMeanADCFromBuffer() const
{
    long tmp[SAMPLE_COUNT];
    for (int i = 0; i < SAMPLE_COUNT; i++)
    {
        tmp[i] = _adcBuf[i];
    }

    std::sort(tmp, tmp + SAMPLE_COUNT);

    long sum = 0;
    for (int i = TRIM_COUNT; i < SAMPLE_COUNT - TRIM_COUNT; i++)
    {
        sum += tmp[i];
    }
    return sum / MIDDLE_COUNT;
}

// 二次公式计算重量
float HX711_Sensor::calculateWeight(long rawAdc)
{
    double adc = (double)rawAdc;
    double weight = _a * adc * adc + _b * adc + _c;
    return (float)weight;
}

// 简易模式去皮，兼容原生scale.tare逻辑
void HX711_Sensor::simpleTare(float scaleFactor)
{
    _scale.set_scale(scaleFactor);
    _scale.tare();
}

// 单次采样读取重量，等效原生 get_units(1)
float HX711_Sensor::simpleRead()
{
    if (_scale.is_ready())
    {
        _simpleRawWeight = _scale.get_units(10);
    }
    return _simpleRawWeight;
}
