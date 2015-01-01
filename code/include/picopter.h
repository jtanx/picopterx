/**
 * @file picopter.h
 * @brief The main include file
 */

#ifndef _PICOPTER_H
#define _PICOPTER_H

//Base includes
#include "config.h"
#include "common.h"
#include "log.h"

//Modules
#include "gpio.h"
#include "buzzer.h" 

namespace picopter {
    template <typename T>
    inline T clamp(const T& n, const T& lower, const T& upper) {
        return std::max(lower, std::min(n, upper));
    }
}


#endif // _PICOPTER_H