/**
 * @file log.cpp
 * @brief Implement logging and error handling functions
 */

#include "common.h"
#include "log.h"

#include <cstdarg>
#include <unistd.h>

using namespace std;

static const char * unspecified_funct = "???";

/**
 * Returns the function name and class from a __PRETTY_FUNCTION__ string.
 * @param pfn The result from __PRETTY_FUNCTION__
 */
static inline const std::string prettyName(const std::string &pfn)
{
    size_t start = pfn.find(" "), end = pfn.find("(");
    if (start == std::string::npos || end == std::string::npos) {
        return pfn;
    }
    return pfn.substr(start + 1, end - start - 1);
}


/**
 * Print a message to stderr and log it via syslog. 
 * The message must be less than BUFSIZ characters long, or it will be truncated.
 * @param level Specify how severe the message is.
                If level is higher (less urgent) than the program's verbosity
                (see options.h) no message will be printed.
 * @param funct String indicating the function name from which this function
                was called.	If this is NULL, Log will show the unspecified_funct
                string instead.
 * @param file Source file containing the function
 * @param line Line in the source file at which Log is called
 * @param fmt A format string
 * @param ... Arguments to be printed according to the format string
 */
void LogEx(int level, const char * funct, const char * file, int line, ...)
{
    //Todo: consider setlogmask(3) to filter messages
    const char *fmt;
    char buffer[BUFSIZ];
    va_list va;

    // Don't print the message unless we need to
    //if (level > g_options.verbosity)
    //	return;

    va_start(va, line);
    fmt = va_arg(va, const char*);
    
    if (fmt == NULL) // sanity check
        Fatal("Format string is NULL");

    vsnprintf(buffer, BUFSIZ, fmt, va);
    va_end(va);

    if (funct == NULL)
        funct = unspecified_funct;
    else
        funct = prettyName(funct).c_str();

    // Make a human readable severity string
    const char *severity;
    switch (level)
    {
        case LOG_ERR:
            severity = "ERROR";
            break;
        case LOG_WARNING:
            severity = "WARNING";
            break;
        case LOG_NOTICE:
            severity = "NOTICE";
            break;
        case LOG_INFO:
            severity = "INFO";
            break;
        default:
            severity = "DEBUG";
            break;
    }

#if USE_SYSLOG
    syslog(level, "%s: %s (%s:%d) - %s", severity, funct, file, line, buffer);
#else
    fprintf(stderr, "%s: %s (%s:%d) - %s\n", severity, funct, file, line, buffer);
#endif
}

/**
 * Handle a Fatal error in the program by printing a message and exiting the program
 * CALLING THIS FUNCTION WILL CAUSE THE PROGAM TO EXIT
 * @param funct - Name of the calling function
 * @param file - Name of the source file containing the calling function
 * @param line - Line in the source file at which Fatal is called
 * @param fmt - A format string
 * @param ... - Arguments to be printed according to the format string
 */
void FatalEx(const char * funct, const char * file, int line, ...)
{
    const char *fmt;
    char buffer[BUFSIZ];
    va_list va;
    va_start(va, line);
    fmt = va_arg(va, const char*);
    
    if (fmt == NULL)
    {
        // Fatal error in the Fatal function.
        // (This really shouldn't happen unless someone does something insanely stupid)
        Fatal("Format string is NULL");
        return; // Should never get here
    }

    vsnprintf(buffer, BUFSIZ, fmt,va);
    va_end(va);

    if (funct == NULL)
        funct = unspecified_funct;

#if USE_SYSLOG
    syslog(LOG_CRIT, "FATAL: %s (%s:%d) - %s", funct, file, line, buffer);
#else
    fprintf(stderr, "FATAL: %s (%s:%d) - %s\n", funct, file, line, buffer);
#endif

    exit(EXIT_FAILURE);
}