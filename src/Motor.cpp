#include "Motor.h"

#define DEFAULT_PWM_DUTY_CYCLE 255  // 默认PWM占空比（0-255）

Motor::Motor(int pin1, int pin2) {
    this->pin1 = pin1;
    this->pin2 = pin2;
    pinMode(pin1, OUTPUT);
    pinMode(pin2, OUTPUT);
}

void Motor::forward() {
    // analogWrite(pin1, DEFAULT_PWM_DUTY_CYCLE);
    digitalWrite(pin1, HIGH);
    digitalWrite(pin2, LOW);
    isStop = false;
    state = "前进";
}

void Motor::backforward() {
    digitalWrite(pin1, LOW);
    digitalWrite(pin2, HIGH);
    // analogWrite(pin2, DEFAULT_PWM_DUTY_CYCLE);
    isStop = false;
    state = "后退";
}

void Motor::stop() {
    digitalWrite(pin1, LOW);
    digitalWrite(pin2, LOW);
    isStop = true;
    state = "停止";
}

bool Motor::getStopState() {
    return isStop;
}

String Motor::getState() {
    return state;
}