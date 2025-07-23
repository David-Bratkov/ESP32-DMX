#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

#include "cie1931.h"

#define MAX_POWER 765

enum COLORS{
    RED,
    GREEN,
    BLUE,
    WHITE
};

class PwmControl
{

private:
    Adafruit_PWMServoDriver *pwm;
    uint16_t frequency;

public:
    PwmControl(uint8_t address = 0x40, float frequency = 500.0);
    ~PwmControl();
    void begin();
    void SetColor(uint8_t channel, uint8_t red, uint8_t green, uint8_t blue, uint8_t white);
};

class ColorData{
    public:
        ColorData(COLORS channel, uint16_t value);
        COLORS channel;
        uint16_t value;
};