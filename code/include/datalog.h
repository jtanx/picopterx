/**
 * @file datalog.h
 * @brief Declaration of functions for data logging
 */

#ifndef _PICOPTERX_DATALOG_H
#define _PICOPTERX_DATALOG_H

#include "config.h"
#include <string>
#include <cstdio>

namespace picopter {
    /**
     * Class to log data in a flexible manner.
     */
    class DataLog {
        public:
            DataLog(const char *name, bool log_startstop=true, const char *location=PICOPTER_LOG_LOCATION);
            virtual ~DataLog();
            
            std::string GetSerial();
            void Write(size_t sz, const char *buf);
            void Write(const char *fmt, ...);
            void PlainWrite(const char *fmt, ...);
        private:
            FILE *m_fp;
            bool m_log_startstop;
            std::string m_serial;
    };
}

#endif // _PICOPTERX_DATALOG_H