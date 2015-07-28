/**
 * @file gps.cpp
 * @brief Base GPS interaction code.
 */

#include "common.h"
#include "gps_feed.h"

using namespace picopter;
using std::chrono::duration_cast;
using std::chrono::seconds;
using std::chrono::milliseconds;
using steady_clock = std::chrono::steady_clock;
using std::this_thread::sleep_for;

const int GPS::WAIT_PERIOD;

/**
 * Constructor. Intialises default stuff.
 */
GPS::GPS(Options *opts)
: m_fix_timeout(FIX_TIMEOUT_DEFAULT)
, m_data{{NAN,NAN,NAN,NAN,NAN,NAN},{NAN,NAN,NAN,NAN,NAN,NAN}, NAN}
, m_last_fix(999)
, m_quit(false)
{
    if (opts) {
        opts->SetFamily("GPS");
        m_fix_timeout = opts->GetInt("FIX_TIMEOUT", FIX_TIMEOUT_DEFAULT);
    }
}

/**
 * Constructor. Constructs a new GPS with default settings.
 */
GPS::GPS() : GPS(NULL) {}

/**
 * Destructor. Sets m_quit to true.
 */
GPS::~GPS() {
    m_quit = true;
}

/**
 * Returns the amount of time since the last GPS fix was acquired.
 * @return The time (in seconds) since the last GPS fix
 */
int GPS::TimeSinceLastFix() {
    return m_last_fix;
}

/**
 * Determines if there is a current GPS fix or not
 * @return true iff there is a GPS fix.
 */
bool GPS::HasFix() {
    return m_last_fix < m_fix_timeout;
}

/**
 * Waits for a GPS fix.
 * @param timeout The timeout (in ms) before this method returns.
 *                Default is no timeout (-1).
 * @return true iff a GPS fix was acquired.
 */
bool GPS::WaitForFix(int timeout) {
    static const milliseconds wait(WAIT_PERIOD);
    auto end = steady_clock::now() + milliseconds(timeout);
    bool hasFix = false;
    
    while (!(hasFix = HasFix()) && (timeout < 0 || steady_clock::now() > end)) {
        sleep_for(wait);
    }
    return hasFix;
}

/**
 * Returns the latest GPS fix, if any.
 * @param d A pointer to the output location. If no value is present for that
 *          parameter, then that value is filled with NaN.
 */
void GPS::GetLatest(GPSData *d) {
    std::lock_guard<std::mutex> lock(m_worker_mutex);
    *d = m_data;
}

/**
 * Returns the current altitude, relative to the ground, if any.
 * @return The current relative altitude or NaN if currently unavailable.
 */
double GPS::GetLatestRelAlt() {
    std::lock_guard<std::mutex> lock(m_worker_mutex);
    return m_data.fix.alt - m_data.fix.groundalt;
}