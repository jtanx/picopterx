/**
 * @file log.cpp
 * @brief Implement logging and error handling functions
 */

#include "common.h"
#include "log.h"

#include <cstdarg>
#include <unistd.h>

static const char * unspecified_funct = "???";

/**
 * Initialises the logger. Should be called at the start of a program.
 */
void LogInit()
{
#ifdef USE_SYSLOG
    openlog("picopter", LOG_PID | LOG_PERROR, LOG_USER);
#endif
    Log(LOG_NOTICE, "Data log files will be stored by default to: %s",
        PICOPTER_LOG_LOCATION);
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

    std::string fn;
    if (funct == NULL)
        fn = unspecified_funct;
    else { //Get the function/method name only
        const char *p1 = strchr(funct, ' '), *p2 = strchr(funct, '(');
        if (p2) {
            if (p1 && p2-p1-1 > 0) { //Got space; must be function/method
                fn = std::string(p1+1, p2-p1-1);
            } else if (p2-funct > 0) { //No space; must be ctor or lambda func.
                fn = std::string(funct, p2-funct);
            } else {
                fn = std::string(funct);
            }
        } else {
            fn = unspecified_funct;
            std::cout << funct << std::endl;
        }
    }

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

#ifdef USE_SYSLOG
    syslog(level, "%s: %s (%s:%d) - %s", severity, fn.c_str(), file, line, buffer);
#else
    fprintf(stderr, "%s: %s (%s:%d) - %s\n", severity, fn.c_str(), file, line, buffer);
    fflush(stderr);
#endif
}

/**
 * Print a message to stderr and log it via syslog. 
 * The message must be less than BUFSIZ characters long, or it will be truncated.
 * This is a simple version that does not print the line number/file from which
 * the call was made.
 * @param level Specify how severe the message is.
                If level is higher (less urgent) than the program's verbosity
                (see options.h) no message will be printed.
 * @param fmt A format string
 * @param ... Arguments to be printed according to the format string
 */
void LogSimple(int level, const char *fmt, ...)
{
    char buffer[BUFSIZ];
    va_list va;

    // Don't print the message unless we need to
    //if (level > g_options.verbosity)
    //	return;

    if (fmt == NULL) // sanity check
        Fatal("Format string is NULL");

    va_start(va, fmt);
    vsnprintf(buffer, BUFSIZ, fmt, va);
    va_end(va);

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

#ifdef USE_SYSLOG
    syslog(level, "%s: %s", severity, buffer);
#else
    fprintf(stderr, "%s: %s\n", severity, buffer);
    fflush(stderr);
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

#ifdef USE_SYSLOG
    syslog(LOG_CRIT, "FATAL: %s (%s:%d) - %s", funct, file, line, buffer);
#else
    fprintf(stderr, "FATAL: %s (%s:%d) - %s\n", funct, file, line, buffer);
    fflush(stderr);
#endif

    exit(EXIT_FAILURE);
}
