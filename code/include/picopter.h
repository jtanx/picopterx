/**
 * @file picopter.h
 * @brief The main include file
 */

#ifndef _PICOPTERX_PICOPTER_H
#define _PICOPTERX_PICOPTER_H

//Base includes
#include "config.h"
#include "common.h"
#include "log.h"
#include "opts.h"

//Modules
#include "gpio.h"
#include "buzzer.h"
#include "flightboard.h"
#include "gps_feed.h"
#include "imu_feed.h"

namespace picopter {
    template <typename T>
    inline T clamp(const T& n, const T& lower, const T& upper) {
        return std::max(lower, std::min(n, upper));
    }
}


#endif // _PICOPTERX_PICOPTER_H