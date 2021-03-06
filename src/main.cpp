/*
    ChibiOS - Copyright (C) 2006..2018 Giovanni Di Sirio

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/

/**
 * @brief This file is based on the MAPLE MINI example from ChibiOS
 *
 * @file main.cpp
 * @author Alex Au
 * @date 2018-09-11
 */


#include "hal.h"
#include "chstd.hpp"
#include "dbus.h"

extern "C" {
#include "canBusProcess.h"
}

#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wmissing-noreturn"

template<class T>
constexpr T warp(T val, T min, T max) noexcept { return (val > max ? max : (val < min ? min : val)); }

namespace {
    //These are the parameters of PID controller
    constexpr float kp = 1.2f;     //Proportional
    constexpr float ki = 60.0f;     //Integration
    constexpr float kd = 0.05f;     //Derivative
    constexpr float dt = 2.f / 1000.f;
    constexpr float error_min = -1000000.f, error_max = 1000000.f;
}

class pid_controller {
public:
    int16_t step(const int16_t setPoint, const int16_t current) noexcept {
        auto error = setPoint - current;
        auto derivative = static_cast<float>(error - _last) / dt;
        _integration = warp((_integration + static_cast<float>(error) * dt) * 0.985f, error_min, error_max);
        _last = error;
        return warp<int16_t>(static_cast<int16_t>(kp * error + ki * _integration + kd * derivative), -10000, 10000);
    }

private:
    int16_t _last = 0;
    float _integration = 0.f;
};

int main() {
    RC_init();
    can_processInit();
    for(;;) {
        uartStartSend(&UARTD1, 14, "Hello World!\n");
        chstd::this_thread::sleep_for(std::chrono::hours(1));
    }
}

#pragma clang diagnostic pop