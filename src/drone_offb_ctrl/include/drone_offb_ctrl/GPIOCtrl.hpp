#pragma once
#include "JetsonGPIO.h"
#include <stdint.h>

class GPIOController
{
private:
    uint8_t key[4];
    uint8_t code[6];
    uint8_t led[3];
    uint8_t beep;
    uint8_t laser;

public:
    GPIOController();
    GPIOController(uint8_t k1, uint8_t k2, uint8_t k3, uint8_t k4, uint8_t code1, uint8_t code2, uint8_t code3, uint8_t code4, uint8_t code5, uint8_t code6, uint8_t led1, uint8_t led2, uint8_t led3, uint8_t beep, uint8_t laser);
    ~GPIOController();

    bool ReadKey(int ind);
    bool ReadCode(int ind);
    void WriteLED(int ind, bool output);
    void WriteBeep(bool output);
    void WriteLaser(bool output);
};
GPIOController::GPIOController()
{
    GPIO::cleanup();
    GPIO::setmode(GPIO::BCM);
}

GPIOController::GPIOController(uint8_t k1, uint8_t k2, uint8_t k3, uint8_t k4, uint8_t code1, uint8_t code2, uint8_t code3, uint8_t code4, uint8_t code5, uint8_t code6, uint8_t led1, uint8_t led2, uint8_t led3, uint8_t beep_, uint8_t laser_)
{
    GPIO::cleanup();
    GPIO::setmode(GPIO::BCM);

    if (k1)
    {
        this->key[0] = k1;
        GPIO::setup(k1, GPIO::IN);
    }
    if (k2)
    {
        this->key[1] = k2;
        GPIO::setup(k2, GPIO::IN);
    }
    if (k3)
    {
        this->key[2] = k3;
        GPIO::setup(k3, GPIO::IN);
    }
    if (k4)
    {
        this->key[3] = k4;
        GPIO::setup(k4, GPIO::IN);
    }

    if (code1)
    {
        this->code[0] = code1;
        GPIO::setup(code1, GPIO::IN);
    }
    if (code2)
    {
        this->code[1] = code2;
        GPIO::setup(code2, GPIO::IN);
    }
    if (code3)
    {
        this->code[2] = code3;
        GPIO::setup(code3, GPIO::IN);
    }
    if (code4)
    {
        this->code[3] = code4;
        GPIO::setup(code4, GPIO::IN);
    }
    if (code5)
    {
        this->code[4] = code5;
        GPIO::setup(code5, GPIO::IN);
    }
    if (code6)
    {
        this->code[5] = code6;
        GPIO::setup(code6, GPIO::IN);
    }

    if (led1)
    {
        this->led[0] = led1;
        GPIO::setup(led1, GPIO::OUT, GPIO::LOW);
    }
    if (led2)
    {
        this->led[1] = led2;
        GPIO::setup(led2, GPIO::OUT, GPIO::LOW);
    }
    if (led3)
    {
        this->led[2] = led3;
        GPIO::setup(led3, GPIO::OUT, GPIO::LOW);
    }

    if (beep_)
    {
        this->beep = beep_;
        GPIO::setup(beep_, GPIO::OUT, GPIO::LOW);
    }
    if (laser_)
    {
        this->laser = laser_;
        GPIO::setup(laser_, GPIO::OUT, GPIO::HIGH);
    }
}

GPIOController::~GPIOController()
{
    GPIO::cleanup();
}

bool GPIOController::ReadKey(int ind)
{
    return GPIO::input(this->key[ind - 1]);
}
bool GPIOController::ReadCode(int ind)
{
    return GPIO::input(this->code[ind - 1]);
}
void GPIOController::WriteLED(int ind, bool output)
{
    GPIO::output(this->led[ind - 1], output);
}
void GPIOController::WriteBeep(bool output)
{
    GPIO::output(this->beep, output);
}
void GPIOController::WriteLaser(bool output)
{
    GPIO::output(this->laser, !output);
}
