/**
 * @file wiringPiI2C.h
 * @brief Simple stub file to get 'wiringPiI2C' when not compiling on the RPi
 */
#ifndef _WIRINGPII2C_STUB_H
#define _WIRINGPII2C_STUB_H

#define wiringPiI2CSetup(devid) 0
#define wiringPiI2CRead(fd) 0
#define wiringPiI2CWrite(fd, data) 0
#define wiringPiI2CWriteReg8(fd, reg, data) 0
#define wiringPiI2CWriteReg16(fd, reg, data) 0
#define wiringPiI2CReadReg8(fd, reg) 0
#define wiringPiI2CReadReg16(fd, reg) 0

#endif //_WIRINGPII2C_STUB_H