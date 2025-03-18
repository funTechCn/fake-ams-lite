#include "Servo.h"

Servo::Servo() : pin(-1), state(0){
}

void Servo::setServoPosition(int position) {
    // 控制舵机转动到指定角度
    if (position > 30) position = 30; // 限制最大角度
    if (position < 0) position = 0;   // 限制最小角度
    
    pwm_set_gpio_level(this->pin, (position * 1000) / 30); // 假设0-30度对应的PWM信号
}

int Servo::getState(){
    return state;
}
void Servo::initIO(int pin){
    this->pin = pin;
    pinMode(pin, OUTPUT);
}