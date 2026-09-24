#ifndef ACCELSTEPPER_MOTOR_H
#define ACCELSTEPPER_MOTOR_H
#include <Arduino.h>
#include <AccelStepper.h>
class AccelStepper_Motor {
public:
    // microstep : 细分。1=全步进, 2=半步, 4, 8, 16, 32...（软件倍率）
    AccelStepper_Motor(uint8_t enablePin, uint8_t stepPin, uint8_t dirPin, uint16_t microstep = 1);

    void begin(float maxSpeed = 1000, float accel = 500); // 初始化参数、设置速度/加速度

    void setMaxSpeed(float speed); // 设置最大速度（全步/秒）
    void setAcceleration(float accel); // 设置加速度（全步/秒²）

    // 阻塞式：一直运行到目标位置
    void moveToBlocking(long steps); // 绝对位置（全步）
    void moveBlocking(long steps); // 相对移动（全步）

    // 非阻塞式：设定目标后，需在 loop 里反复调用 run()
    void moveTo(long steps); // 绝对目标（全步）
    void move(long steps); // 相对目标（全步）
    bool run(); // 运行一步，返回电机是否仍在运动

    // 往复运动（阻塞）：steps=单程距离（全步），pauseMs=两端停留毫秒
    void moveReciprocation(long steps, unsigned long pauseMs = 1000);
    // 获取信息
    float getSpeed(); // 当前速度（全步/秒），带方向正负
    float getMaxSpeed(); // 最大速度（全步/秒）
    long getCurrentPosition(); // 当前位置（全步）
    long distanceToGo(); // 距目标剩余（全步）
    bool isRunning(); // 是否正在运动

    // 强制设置当前位置（全步单位，可用于回零后清零坐标）
    void setCurrentPosition(long pos);

    // 恒速运行模式（持续正反转，需循环调用 runSpeed）
    void setSpeed(float speed); // 设置恒速速度（全步/秒），正负代表方向
    bool runSpeed(); // 恒速模式刷新，必须放在loop里循环调用
    void stop(); // 按加速度平滑减速停止
    
    void enable();  // 使能驱动器（EN拉低）
    void disable(); // 禁用驱动器（EN拉高）
private:
    AccelStepper _stepper;
    uint16_t _microstep;
    uint8_t _enablePin;
    uint8_t _stepPin;
    uint8_t _dirPin;
};
#endif
