/**
 * @file flightboard-private.h
 * @brief Contains calibration data and pin mappings for the FlightBoard class.
 * Note: The unit 'step' is ServoBlaster specific. Usually 1 step is 10us.
 */

#ifndef _FLIGHTBOARD_PRIVATE_H
#define _FLIGHTBOARD_PRIVATE_H

/** 
 * Linearly scales from one range to another range.
 * @param x The input value (assumed that xl < x < xh)
 * @param xl The lower bound on the input scale
 * @param xh The upper bound on the input scale
 * @param yl The lower bound on the output scale
 * @param yh The upper bound on the output scale
 */
#define LINEAR_SCALE(x, xl, xh, yl, yh) \
        ((yl) + (((yh) - (yl)) * ((x) - (xl))) / ((xh) - (xl)))
/** Helper to scale a speed (-100 to 100) value **/
#define SPEED_SCALE(speed, yl, yh) LINEAR_SCALE(speed, -100, 100, yl, yh)
#define INV_SPEED_SCALE(val, yl, yh) LINEAR_SCALE(val, yl, yh, -100, 100)

//Calibrated on October 17 2014
/** The ServoBlaster channel for the aileron **/
#define AILERON_CHANNEL 0
/** The actual (physical) pin number for the aileron **/
#define AILERON_PIN_PHYSICAL 11
/** The lower bound for the pulse width (in steps) of the aileron **/
#define AILERON_LOW 111
/** The upper bound for the pulse width (in steps) of the aileron **/
#define AILERON_HIGH 193
/** Scales the aileron speed to the corresponding PWM range **/
#define AILERON_SCALE(x) SPEED_SCALE(x, AILERON_LOW, AILERON_HIGH)
#define INV_AILERON_SCALE(x) INV_SPEED_SCALE(x, AILERON_LOW, AILERON_HIGH)

//Calibrated on October 17 2014
/** The ServoBlaster channel for the elevator **/
#define ELEVATOR_CHANNEL 1
/** The actual (physical) pin number for the elevator **/
#define ELEVATOR_PIN_PHYSICAL 12
/** The lower bound for the pulse width (in steps) of the elevator **/
#define ELEVATOR_LOW 111
/** The upper bound for the pulse width (in steps) of the elevator **/
#define ELEVATOR_HIGH 195
/** Scales the elevator speed to the corresponding PWM range **/
#define ELEVATOR_SCALE(x) SPEED_SCALE(x, ELEVATOR_LOW, ELEVATOR_HIGH)
#define INV_ELEVATOR_SCALE(x) INV_SPEED_SCALE(x, ELEVATOR_LOW, ELEVATOR_HIGH)

//Calibrated on October 17 2014
/** The ServoBlaster channel for the rudder **/
#define RUDDER_CHANNEL 2
/** The actual (physical) pin number for the rudder **/
#define RUDDER_PIN_PHYSICAL 15
/** The lower bound for the pulse width (in steps) of the rudder **/
#define RUDDER_LOW 110
/** The upper bound for the pulse width (in steps) of the rudder **/
#define RUDDER_HIGH 194
/** Scales the rudder speed to the corresponding PWM range **/
#define RUDDER_SCALE(x) SPEED_SCALE(x, RUDDER_LOW, RUDDER_HIGH)
#define INV_RUDDER_SCALE(x) INV_SPEED_SCALE(x, RUDDER_LOW, RUDDER_HIGH)

//Calibrated on October 5 2014
/** The ServoBlaster channel for the gimbal **/
#define GIMBAL_CHANNEL 3
/** The actual (physical) pin number for the gimbal **/
#define GIMBAL_PIN_PHYSICAL 16
/** The lower bound for the pulse width (in steps) of the gimbal **/
#define GIMBAL_LOW 95
/** The upper bound for the pulse width (in steps) of the gimbal **/
#define GIMBAL_HIGH 210
/** Scales the gimbal angle (0 - 90) to the corresponding PWM range **/
#define GIMBAL_SCALE(x) LINEAR_SCALE(x, 0, 90, GIMBAL_LOW, GIMBAL_HIGH)
#define INV_GIMBAL_SCALE(x) LINEAR_SCALE(x, GIMBAL_LOW, GIMBAL_HIGH, 0, 90)

#endif //_FLIGHTBOARD_PRIVATE_H