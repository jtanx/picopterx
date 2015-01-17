/**
 * @file wiringPi.h
 * @brief Simple stub file to get 'wiringPi' when not compiling on the RPi
 */
#ifndef _WIRINGPI_STUB_H
#define _WIRINGPI_STUB_H

#define wiringPiSetup()
#define pinMode(pin, mode)
#define digitalRead(pin) 1
#define digitalWrite(pin, value)

#define HIGH 1
#define LOW 0

//Unfortunately the minimum accuracy on Windows is ~2ms, which means that frequencies above 500Hz are not testable on Windows.
#define delayMicroseconds(x) (std::this_thread::sleep_for(std::chrono::microseconds(x)))

#endif //_WIRINGPI_STUB_H