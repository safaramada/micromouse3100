#include "../include/Encoder.hpp"

namespace mtrn3100 {

Encoder* Encoder::leftEncoder = nullptr;
Encoder* Encoder::rightEncoder = nullptr;

Encoder::Encoder(uint8_t enc1, uint8_t enc2) : encoder1_pin(enc1), encoder2_pin(enc2)
{
    pinMode(encoder1_pin, INPUT_PULLUP);
    pinMode(encoder2_pin, INPUT_PULLUP);

    // First object becomes left encoder
    if (leftEncoder == nullptr)
    {
        leftEncoder = this;

        attachInterrupt(
            digitalPinToInterrupt(encoder1_pin),
            leftISR,
            CHANGE
        );
    }
    else
    {
        rightEncoder = this;

        attachInterrupt(
            digitalPinToInterrupt(encoder1_pin),
            rightISR,
            CHANGE
        );
    }
}

void Encoder::readEncoder()
{
    bool A = digitalRead(encoder1_pin);
    bool B = digitalRead(encoder2_pin);

    if (A == B)
    {
        count++;
        direction = 1;
    }
    else
    {
        count--;
        direction = -1;
    }
}

void Encoder::leftISR()
{
    if (leftEncoder)
        leftEncoder->readEncoder();
}

void Encoder::rightISR()
{
    if (rightEncoder)
        rightEncoder->readEncoder();
}

long Encoder::getCount()
{
    noInterrupts();
    long c = count;
    interrupts();
    return c;
}

void Encoder::reset()
{
    noInterrupts();
    count = 0;
    interrupts();
}

float Encoder::getRotation()
{
    return (2.0f * PI * getCount()) / counts_per_revolution;
}
}