#include "STM32Stop1.h"

extern "C" void SystemClock_Config(void);  // Reused from STM32 core

STM32Stop1::STM32Stop1() {
    _afterWakeCallback = nullptr;
}

void STM32Stop1::begin(void (*afterWakeCallback)()) {
    _afterWakeCallback = afterWakeCallback;
}

void STM32Stop1::sleep() {
    // Optional: Suspend system tick to save more power
    HAL_SuspendTick();

    // Clear any previous wakeup flags
    __HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);

    // Enter STOP1 mode (wait for interrupt)
    HAL_PWREx_EnterSTOP1Mode(PWR_STOPENTRY_WFI);

    // Woken up

    // Resume system tick
    HAL_ResumeTick();

    // Restore system clocks
    SystemClock_Config();

    // Call user-provided reinit callback if defined
    if (_afterWakeCallback) {
        _afterWakeCallback();
    }
}
