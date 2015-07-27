/**
 * @file gpio.cpp
 * @brief GPIO handling code.
 * Uses wiringPi to control the GPIO pins on the RPi.
 */

#include "common.h"
#include "gpio.h"

#include <wiringPi.h>

using namespace picopter;

static bool g_gpio_initted = false;
static std::mutex g_mutex;

/**
 * Initialises the GPIO pins, if necessary.
 */
void gpio::Init() {
    std::lock_guard<std::mutex> lock(g_mutex);
    
    if (!g_gpio_initted) {
        wiringPiSetup();
        pinMode(gpio::BUZZER_PIN, OUTPUT);
        
        g_gpio_initted = true;
    }
}

/**
 * Turns the buzzer on or off.
 * Assumes that gpio::init has already been called.
 * Probably not thread-safe.
 * @param value Indicates whether the buzzer should be on (true) or off (false).
 */
void gpio::SetBuzzer(bool value) {
    digitalWrite(gpio::BUZZER_PIN, value);
}
