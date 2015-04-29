/**
 * @file log.h
 * @brief Declaration of functions for printing log messages and/or terminating 
 *        program after a fatal error
 */

#ifndef _PICOPTERX_LOG_H
#define _PICOPTERX_LOG_H

#include "config.h"

/* To get around a 'pedantic' C99 rule that you must have at least 1 
   variadic arg, combine fmt into that. Note the use of __FILENAME__ instead
   of __FILE__. This is custom defined by the makefile (see CMakeLists.txt),
   which removes the absolute path from the file name. Credit:
   http://stackoverflow.com/questions/8487986/file-macro-shows-full-path
*/
#define Log(level, ...) LogEx(level, __PRETTY_FUNCTION__, __FILENAME__, __LINE__, __VA_ARGS__)
#define Fatal(...) FatalEx(__PRETTY_FUNCTION__, __FILENAME__, __LINE__, __VA_ARGS__)

#ifdef USE_SYSLOG
#include <syslog.h>
#else
//Replicate the syslog log levels
/** An enum to make the severity of log messages human readable in code **/
enum {LOG_ERR=0, LOG_WARNING=1, LOG_NOTICE=2, LOG_INFO=3, LOG_DEBUG=4};
#endif

extern void LogInit();
extern void LogSimple(int level, const char * fmt, ...);
extern void LogEx(int level, const char * funct, const char * file, int line, ...);
extern void FatalEx(const char * funct, const char * file, int line, ...);  

#endif // _PICOPTERX_LOG_H
