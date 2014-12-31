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

/**
 * Initialises the GPIO pins, if necessary.
 */
void gpio::init() {
    if (!g_gpio_initted) {
        wiringPiSetup();
		pinMode(gpio::MODE_PIN, INPUT);
		pinMode(gpio::BUZZER_PIN, OUTPUT);
		
        g_gpio_initted = true;
    }
}

/**
 * Determines if autonomous mode has been enabled by the user.
 * @return true iff the user has enabled the switch for autonomous mode
 */
bool gpio::isAutoMode() {
    return digitalRead(gpio::MODE_PIN);
}

/**
 * Turns the buzzer on or off.
 * @param value Indicates whether the buzzer should be on (true) or off (false).
 */
void gpio::setBuzzer(bool value) {
    digitalWrite(gpio::BUZZER_PIN, value);
}
