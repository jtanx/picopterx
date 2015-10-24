/**
 * @file datalog.cpp
 * @brief Implement data logging functions
 */

#include "common.h"
#include "datalog.h"
#include <ctime>
#include <cstdarg>

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
 * Creates a log file for logging *data*.
 * The filename will be of the form 'file-*timestamp*.txt'.
 * The *timestamp* is that from FileTimestamp. If the file exists, it will be
 * overwritten.
 * @param file The name of the file, excluding any extension.
 * @param log_startstop Whether or not to log the start and stop times of the log.
 *                      Defaults to true.
 * @param location The folder where the file should be stored. Defaults to 
 *                 PICOPTER_LOG_LOCATION, which is set in config.h. This should
 *                 be the home folder of the user.
 */
DataLog::DataLog(const char *file, bool log_startstop, const char *location)
: m_log_startstop(log_startstop)
{
    std::string path = GenerateFilename(location, file, ".txt");
    m_fp = fopen(path.c_str(), "w+");
    if (!m_fp) {
        Log(LOG_WARNING, "Could not open log for writing, falling back to stderr: %s", file);
        m_fp = stderr;
    }
    
    size_t off = strlen(file) + strlen(location) + 2;
    m_serial = path.substr(off, path.size()-off-4);
    
    if (log_startstop) {
        Write(": Log started");
    }
}

/**
 * Destructor. Closes the file pointer.
 */
DataLog::~DataLog() {
    if (m_log_startstop) {
        Write(": Log closed");
    }
    fclose(m_fp);
}

/**
 * Retrieves the unique serial (timestamp) for this datalog.
 * @return The serial of this datalog.
 */
std::string DataLog::GetSerial() {
    return m_serial;
}

/**
 * Writes data to the log file, verbatim.
 * @param sz The size of the buffer to be written.
 * @param buf The pointer to the buffer.
 */
void DataLog::Write(size_t sz, const char *buf) {
    fwrite(buf, 1, sz, m_fp);
}

/**
 * Writes a line of text to the file, prepending a timestamp.
 * @param fmt A format string.
 * @param ... Arguments to be printed according to the format string.
 */
void DataLog::Write(const char *fmt, ...) {
    char buf[BUFSIZ];
    va_list va;

    va_start(va, fmt);
    vsnprintf(buf, BUFSIZ, fmt, va);
    va_end(va);
    
    struct tm ts = GetTimestamp();
    fprintf(m_fp, "%02d/%02d/%04d %02d:%02d:%02d%s\n", 
            ts.tm_mday, ts.tm_mon+1, ts.tm_year+1900,
            ts.tm_hour, ts.tm_min, ts.tm_sec, buf);
    fflush(m_fp);
}

/**
 * Writes a line of text to the file (no timestamp).
 * @param fmt A format string.
 * @param ... Arguments to be printed according to the format string.
 */
void DataLog::PlainWrite(const char *fmt, ...) {
    va_list va;
    va_start(va, fmt);
    vfprintf(m_fp, fmt, va);
    va_end(va);
    fflush(m_fp);
}