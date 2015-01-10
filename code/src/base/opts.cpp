/**
 * @file opts.cpp
 * @brief Options and persistent configurations handler
 */

#include "picopter.h"

using picopter::Options;

Options::Options(const char *file) {

}

/**
 * Constructs an options class with no initial file.
 * This is the same as calling Options::Options(NULL).
 */
Options::Options() : Options(NULL) {}