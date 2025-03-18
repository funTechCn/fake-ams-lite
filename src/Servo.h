#include <Arduino.h>
#pragma once

class Servo{
private:
    int pin;
    int state;
public:
    Servo();
    void setServoPosition(int position);
    int getState();
    void initIO(int pin);
};
