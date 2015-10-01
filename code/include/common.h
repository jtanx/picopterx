/**
 * @file common.h
 * @brief Commonly included headers
 */

#ifndef _PICOPTERX_COMMON_H
#define _PICOPTERX_COMMON_H

#include "config.h"
#include "log.h"
#include "datalog.h"

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <string>
#include <vector>
#include <deque>
#include <map>
#include <utility>
#include <stdint.h>

#include "opts.h"

//C++11 threading stuff
#include <thread>
#include <future>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <functional>

namespace picopter {
    template <typename T>
    inline T clamp(const T& n, const T& lower, const T& upper) {
        return std::max(lower, std::min(n, upper));
    }
    
    std::string GenerateFilename(const char *folder, const char *name, const char *ext);
}

#endif // _PICOPTERX_COMMON_H