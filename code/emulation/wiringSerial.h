/**
 * @file wiringSerial.h
 * @brief Simple stub file to get 'wiringSerial' when not compiling on the RPi
 */
#ifndef _WIRINGSERIAL_STUB_H
#define _WIRINGSERIAL_STUB_H

#define serialOpen(A,B) 0
#define serialDataAvail(fd) 0
#define serialGetchar(fd) fd
#define serialClose(fd)

#endif //_WIRINGSERIAL_STUB_H