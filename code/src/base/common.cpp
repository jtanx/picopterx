/**
 * @file common.cpp
 * @brief Common utility functions that don't belong anywhere specific.
 */

#include "common.h"
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>

using namespace picopter;

/**
 * Retrieves the current timestamp.
 * Is thread-safe and re-entrant on all POSIX platforms.
 * @return The current timestamp, in local time.
 */
static struct tm GetTimestamp() {
    time_t now = time(NULL);
    struct tm ts;
#ifndef _WIN32
    localtime_r(&now, &ts);
#else
    struct tm *tsp = localtime(&now);
    ts = *tsp;
#endif
    return ts;
}

/**
 * Attempt to generate a unique filename.
 * Generates filenames in the form of:
 * folder/name-YYYY-MM-DD-NNN.ext
 * @param [in] folder The folder for this file.
 * @param [in] name The name of this file.
 * @param [in] ext The extension of this file, e.g. '.txt'.
 * @return The generated filename.
 */
std::string picopter::GenerateFilename(const char *folder, const char *name, const char *ext) {
    struct stat buffer; 
    struct tm ts = GetTimestamp();
    char buf[20];
    snprintf(buf, 20, "-%04d-%02d-%02d-",
                 ts.tm_year+1900, ts.tm_mon+1, ts.tm_mday);
    
    std::string n1(folder), n2;
    n1 += "/"; n1 += name; n1 += buf;
    n2 = n1 + "001"  + ext;
    for (int i = 2; (stat(n2.c_str(), &buffer) == 0) && i < 998; i++) {
        snprintf(buf, 20, "%03d", i);
        n2 = n1 + buf + ext;
    }
    return n2;
}