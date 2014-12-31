/**
 * @file wiringPi.h
 * @brief Simple stub file to get 'wiringPi' when not compiling on the rPi
 */
#ifndef _WIRINGPI_STUB_H
#define _WIRINGPI_STUB_H

#define wiringPiSetup()
#define pinMode(pin, mode)
#define digitalRead(pin) 1
#define digitalWrite(pin, value)

#endif //_WIRINGPI_STUB_H