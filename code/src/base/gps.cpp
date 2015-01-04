/**
 * @file gps.cpp
 * @brief GPS interaction code.
 * Uses gpsd to interact with the GPS
 */

#include "picopter.h"
#include "libgpsmm.h"

using namespace picopter;
using std::chrono::duration_cast;
using std::chrono::seconds;
typedef std::chrono::steady_clock steady_clock;

/**
 * Constructor. Establishes a connection to gpsd, assuming it is running
 * on the default gpsd port. Starts the worker thread to receive GPS data.
 * @throws std::invalid_argument if a connection to gpsd cannot be established.
 */
GPS::GPS()
: m_data{}
, m_last_fix(999)
, m_quit(false)
{
    m_gps_rec = new gpsmm("localhost", DEFAULT_GPSD_PORT);
    if (m_gps_rec->stream(WATCH_ENABLE|WATCH_JSON) == NULL) {
        throw std::invalid_argument("gpsd is not running");
    }
    m_worker = std::thread(&GPS::gpsLoop, this);
    
    std::cout << m_data.load().fix.lat << std::endl;
    std::cout << m_data.load().fix.lon << std::endl;
    std::cout << m_data.load().err.lat << std::endl;
    std::cout << m_data.load().err.lon << std::endl;
    std::cout << m_data.load().timestamp << std::endl;
    
    Log(LOG_INFO, "IS GPSData LOCK FREE? %d", m_data.is_lock_free());
    Log(LOG_INFO, "IS m_last_fix LOCK FREE? %d", m_last_fix.is_lock_free());
}

/**
 * Destructor. Stops the worker thread and waits for it to exit.
 */
GPS::~GPS() {
    m_quit = true;
    m_worker.join();
    delete m_gps_rec;
}

/**
 * Main worker thread. Polls gpsd for new GPS data and updates as necessary.
 */
void GPS::gpsLoop() {
    auto tp = steady_clock::now() - std::chrono::seconds(999);
    Log(LOG_INFO, "GPS Started!");
    
    while (!m_quit) {
        m_last_fix = duration_cast<seconds>(steady_clock::now() - tp).count();
        if (m_last_fix > 0) {
            Log(LOG_INFO, "LAST FIX: %d", m_last_fix.load());
        }
                
        if (m_gps_rec->waiting(CYCLE_TIMEOUT) && !m_quit) {
            struct gps_data_t* data;
            if ((data = m_gps_rec->read()) == NULL) {
                Log(LOG_WARNING, "Failed to read GPS data");
            } else if (data->set & LATLON_SET) {
                GPSData d;
                Log(LOG_INFO, "Got GPS data!");
                d.fix.lat = data->fix.latitude;
                d.err.lat = data->fix.epy;
                d.fix.lon = data->fix.longitude;
                d.err.lon = data->fix.epx;
                d.timestamp = data->fix.time;
                m_data = d;
                
                std::cout << d.fix.lat << std::endl;
                std::cout << d.fix.lon << std::endl;
                std::cout << d.err.lat << std::endl;
                std::cout << d.err.lon << std::endl;
                std::cout << d.timestamp << std::endl;
                
                tp = steady_clock::now();
            }
        }
    }
}

/**
 * Returns the amount of time since the last GPS fix was acquired.
 * @return The time (in seconds) since the last GPS fix
 */
int GPS::timeSinceLastFix() {
    return m_last_fix;
}

/**
 * Returns the latest GPS fix, if any.
 * @param d A pointer to the output location. If no value is present for that
 *          parameter, then that value is filled with NaN.
 */
void GPS::getLatest(GPSData *d) {
    *d = m_data;
}