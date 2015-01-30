/**
 * @file gps.cpp
 * @brief GPS interaction code.
 * Uses gpsd to interact with the GPS
 */

#include "picopter.h"
#include "libgpsmm.h"
#include <cmath>

using namespace picopter;
using std::chrono::duration_cast;
using std::chrono::seconds;
using steady_clock = std::chrono::steady_clock;

/**
 * Constructor. Establishes a connection to gpsd, assuming it is running
 * on the default gpsd port. Starts the worker thread to receive GPS data.
 * @param opts A pointer to options, if any (NULL for defaults)
 * @throws std::invalid_argument if a connection to gpsd cannot be established.
 */
GPS::GPS(Options *opts)
: m_cycle_timeout(CYCLE_TIMEOUT_DEFAULT)
, m_data{{{NAN,NAN},{NAN,NAN}, NAN}}
, m_last_fix(999)
, m_quit(false)
{
    m_gps_rec = new gpsmm("localhost", DEFAULT_GPSD_PORT);
    m_worker = std::thread(&GPS::gpsLoop, this);
    
    if (opts) {
        opts->SetFamily("GPS");
        m_cycle_timeout = opts->GetInt("CYCLE_TIMEOUT", CYCLE_TIMEOUT_DEFAULT);
    }
    
    GPSData d = m_data.load();
    Log(LOG_INFO, "GPSData: Fix {%f, %f}, Unc {%f, %f}, time %f",
        d.fix.lat, d.fix.lon, d.err.lat, d.err.lon, d.timestamp);
    
    Log(LOG_INFO, "IS GPSData LOCK FREE? %d", m_data.is_lock_free());
    Log(LOG_INFO, "IS m_last_fix LOCK FREE? %d", m_last_fix.is_lock_free());
}

/**
 * Constructor. Constructs a new GPS with default settings.
 */
GPS::GPS() : GPS(NULL) {}

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
    
    bool gpsd_likely_not_running = false;
    while (!m_quit && ((m_gps_rec->stream(WATCH_ENABLE|WATCH_JSON) == NULL))) {
        if (!gpsd_likely_not_running) {
            Log(LOG_WARNING, "Could not stream from gpsd. Check that it's running.");
            gpsd_likely_not_running = true;
        }
        std::this_thread::sleep_for(seconds(1));
    }
    
    while (!m_quit) {
        m_last_fix = duration_cast<seconds>(steady_clock::now() - tp).count();
        if (m_last_fix > 0) {
            Log(LOG_INFO, "LAST FIX: %d", m_last_fix.load());
        }
                
        if (m_gps_rec->waiting(m_cycle_timeout) && !m_quit) {
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
                
                
                time_t t = (time_t) d.timestamp;
                Log(LOG_INFO, "Current fix: (%.6f +/- %.1fm, %.6f +/- %.1fm) at %s",
                    d.fix.lat, d.err.lat, d.fix.lon, d.err.lon, ctime(&t));
                
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