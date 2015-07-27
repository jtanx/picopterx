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


#endif //_FLIGHTBOARD_PRIVATE_H
