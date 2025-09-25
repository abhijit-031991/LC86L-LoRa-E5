#ifndef STM32STOP1_H
#define STM32STOP1_H

#include "Arduino.h"
#include "stm32wlxx_hal.h"

class STM32Stop1 {
public:
    STM32Stop1();

    // Optional user-defined callback to reinitialize peripherals
    void begin(void (*afterWakeCallback)() = nullptr);

    // Enter STOP1 mode (wait for EXTI or RTC interrupt)
    void sleep();

private:
    void (*_afterWakeCallback)();
};

#endif
