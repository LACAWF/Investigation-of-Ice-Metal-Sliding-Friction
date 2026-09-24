#include "AccelStepper_Motor.h"
AccelStepper_Motor::AccelStepper_Motor(uint8_t enablePin, uint8_t stepPin, uint8_t dirPin, uint16_t microstep)
    : _stepper(AccelStepper::DRIVER, stepPin, dirPin),
      _microstep(microstep == 0 ? 1 : microstep),
      _enablePin(enablePin),
      _stepPin(stepPin),
      _dirPin(dirPin)
{
}
void AccelStepper_Motor::begin(float maxSpeed, float accel)
{
    // ESP32 全局构造阶段 pinMode 可能失效，这里在 setup 阶段重新初始化所有引脚
    pinMode(_enablePin, OUTPUT);
    pinMode(_stepPin, OUTPUT);
    pinMode(_dirPin, OUTPUT);
    digitalWrite(_enablePin, LOW);
    digitalWrite(_stepPin, LOW);
    digitalWrite(_dirPin, LOW);
    enable();
    setMaxSpeed(maxSpeed);
    setAcceleration(accel);
    setCurrentPosition(0);
}
void AccelStepper_Motor::enable()
{
    digitalWrite(_enablePin, LOW);
}
void AccelStepper_Motor::disable()
{
    digitalWrite(_enablePin, HIGH);
}
void AccelStepper_Motor::setMaxSpeed(float speed)
{
    _stepper.setMaxSpeed(speed * _microstep);
}
void AccelStepper_Motor::setAcceleration(float accel)
{
    _stepper.setAcceleration(accel * _microstep);
}
void AccelStepper_Motor::moveTo(long steps)
{
    _stepper.moveTo(steps * (long)_microstep);
}
void AccelStepper_Motor::move(long steps)
{
    _stepper.move(steps * (long)_microstep);
}
bool AccelStepper_Motor::run()
{
    return _stepper.run();
}
void AccelStepper_Motor::moveToBlocking(long steps)
{
    moveTo(steps);
    while (_stepper.distanceToGo() != 0)
    {
        _stepper.run();
    }
}
void AccelStepper_Motor::moveBlocking(long steps)
{
    move(steps);
    while (_stepper.distanceToGo() != 0)
    {
        _stepper.run();
    }
}
void AccelStepper_Motor::moveReciprocation(long steps, unsigned long pauseMs)
{
    move(steps);
    while (_stepper.distanceToGo() != 0)
    {
        _stepper.run();
    }
    delay(pauseMs);
    move(-steps);
    while (_stepper.distanceToGo() != 0)
    {
        _stepper.run();
    }
    delay(pauseMs);
}
float AccelStepper_Motor::getSpeed()
{
    return _stepper.speed() / (float)_microstep;
}
float AccelStepper_Motor::getMaxSpeed()
{
    return _stepper.maxSpeed() / (float)_microstep;
}
long AccelStepper_Motor::getCurrentPosition()
{
    return _stepper.currentPosition() / (long)_microstep;
}
long AccelStepper_Motor::distanceToGo()
{
    return _stepper.distanceToGo() / (long)_microstep;
}
bool AccelStepper_Motor::isRunning()
{
    return _stepper.isRunning();
}
void AccelStepper_Motor::setCurrentPosition(long pos)
{
    _stepper.setCurrentPosition(pos * (long)_microstep);
}
void AccelStepper_Motor::setSpeed(float speed)
{
    _stepper.setSpeed(speed * _microstep);
}
bool AccelStepper_Motor::runSpeed()
{
    return _stepper.runSpeed();
}
void AccelStepper_Motor::stop()
{
    _stepper.stop();
}
