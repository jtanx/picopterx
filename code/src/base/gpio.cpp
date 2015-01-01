/**
 * @file gpio.cpp
 * @brief GPIO handling code.
 * Uses wiringPi to control the GPIO pins on the rPi.
 * PWM (through the GPIO pins) is done through ServoBlaster.
 */

#include "picopter.h"

#include <wiringPi.h>

using namespace picopter;

static bool g_gpio_initted = false;
static std::mutex g_mutex;

/**
 * Initialises the GPIO pins, if necessary.
 */
void gpio::init() {
    std::lock_guard<std::mutex> lock(g_mutex);
    
    if (!g_gpio_initted) {
        wiringPiSetup();
		pinMode(gpio::MODE_PIN, INPUT);
		pinMode(gpio::BUZZER_PIN, OUTPUT);
		
        g_gpio_initted = true;
    }
}

/**
 * Determines if autonomous mode has been enabled by the user.
 * Assumes that gpio::init has already been called.
 * Probably not thread-safe.
 * @return true iff the user has enabled the switch for autonomous mode
 */
bool gpio::isAutoMode() {
    return digitalRead(gpio::MODE_PIN);
}

/**
 * Turns the buzzer on or off.
 * Assumes that gpio::init has already been called.
 * Probably not thread-safe.
 * @param value Indicates whether the buzzer should be on (true) or off (false).
 */
void gpio::setBuzzer(bool value) {
    digitalWrite(gpio::BUZZER_PIN, value);
}
