/**
 * @file datalog.h
 * @brief Declaration of functions for data logging
 */

#ifndef _PICOPTERX_DATALOG_H
#define _PICOPTERX_DATALOG_H

#include "config.h"
#include <cstdio>

namespace picopter {
    /**
     * Class to log data in a flexible manner.
     */
    class DataLog {
        public:
            DataLog(const char *name, bool log_startstop=true, const char *location=PICOPTER_LOG_LOCATION);
            virtual ~DataLog();
            
            void Write(size_t sz, const char *buf);
            void Write(const char *fmt, ...);
            void PlainWrite(const char *fmt, ...);
        private:
            FILE *m_fp;
            bool m_log_startstop;
    };
}

#endif // _PICOPTERX_DATALOG_H