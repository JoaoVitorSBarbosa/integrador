#ifndef TESTS_H
#define TESTS_H

#include <Arduino.h>
#include "Motor.h"
#include "Encoder.h"
#include "IMU.h"

class Tests {
public:
    Tests();
    ~Tests();

    template<typename T>
    void singleTest(T& obj);

private:
    void motor(Motor& obj);
    void encoder(Encoder& obj);
    void imu(IMU& obj);
};

template<> void Tests::singleTest<Motor>(Motor& obj);
template<> void Tests::singleTest<Encoder>(Encoder& obj);
template<> void Tests::singleTest<IMU>(IMU& obj);

#endif
