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
using std::chrono::milliseconds;
using steady_clock = std::chrono::steady_clock;
using std::this_thread::sleep_for;

/**
 * Constructor. Establishes a connection to gpsd, assuming it is running
 * on the default gpsd port. Starts the worker thread to receive GPS data.
 * @param opts A pointer to options, if any (NULL for defaults)
 * @throws std::invalid_argument if a connection to gpsd cannot be established.
 */
GPS::GPS(Options *opts)
: m_cycle_timeout(CYCLE_TIMEOUT_DEFAULT)
, m_fix_timeout(FIX_TIMEOUT_DEFAULT)
, m_had_fix(false)
, m_data{{{NAN,NAN,NAN,NAN},{NAN,NAN,NAN,NAN}, NAN}}
, m_last_fix(999)
, m_quit(false)
{
    m_gps_rec = new gpsmm("localhost", DEFAULT_GPSD_PORT);
    if (m_gps_rec->stream(WATCH_ENABLE|WATCH_JSON) == NULL) {
        delete m_gps_rec;
        throw std::invalid_argument("gpsd is not running");
    }
    m_worker = std::thread(&GPS::GPSLoop, this);
    
    if (opts) {
        opts->SetFamily("GPS");
        m_cycle_timeout = opts->GetInt("CYCLE_TIMEOUT", CYCLE_TIMEOUT_DEFAULT);
        m_fix_timeout = opts->GetInt("FIX_TIMEOUT", FIX_TIMEOUT_DEFAULT);
    }
    
    //GPSData d = m_data.load();
    //Log(LOG_INFO, "GPSData: Fix {%f, %f}, Unc {%f, %f}, time %f",
    //    d.fix.lat, d.fix.lon, d.err.lat, d.err.lon, d.timestamp);
    
    //Log(LOG_INFO, "IS GPSData LOCK FREE? %d", m_data.is_lock_free());
    //Log(LOG_INFO, "IS m_last_fix LOCK FREE? %d", m_last_fix.is_lock_free());
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
void GPS::GPSLoop() {
    auto last_fix = steady_clock::now() - seconds(m_fix_timeout);
    
    Log(LOG_INFO, "GPS Started!");
    while (!m_quit) {
        m_last_fix = duration_cast<seconds>(steady_clock::now() - last_fix).count();
        if (m_had_fix && !HasFix()) {
            Log(LOG_WARNING, "Lost the GPS fix. Last fix: %d seconds ago.",
                m_last_fix.load());
            m_had_fix = false;
        }

        if (m_gps_rec->waiting(m_cycle_timeout) && !m_quit) {
            struct gps_data_t* data;
            if ((data = m_gps_rec->read()) == NULL) {
                Log(LOG_WARNING, "Failed to read GPS data");
            } else if ((data->set & LATLON_SET) && (data->set & SPEED_SET)) {
                GPSData d = m_data;
                //GPSData d2 = d;
                d.fix.lat = DEG2RAD(data->fix.latitude);
                d.fix.lon = DEG2RAD(data->fix.longitude);
                d.fix.speed = data->fix.speed;
                
                if (data->set & TRACK_SET) {
                    d.fix.heading = DEG2RAD(data->fix.track);
                    //std::cout << "CALC " << RAD2DEG(TRUEBEARING(navigation::CoordBearing(d2.fix, d.fix))) << std::endl;
                    if (data->set & TRACKERR_SET) {
                        d.err.heading = DEG2RAD(data->fix.epd);
                    }
                }
                if (data->set & SPEEDERR_SET) {
                    d.err.speed = data->fix.eps;
                }
                if (data->set & HERR_SET) {
                    d.err.lat = data->fix.epy;
                    d.err.lon = data->fix.epx;
                }
                if (data->set & TIME_SET) {
                    d.timestamp = data->fix.time;
                }
                m_data = d;
                
                time_t t = (time_t) d.timestamp;
                Log(LOG_INFO, "Current fix: (%.6f +/- %.1fm, %.6f +/- %.1fm) [%.2f +/- %.2f at %.2f +/- %.2f] at %s",
                    RAD2DEG(d.fix.lat), d.err.lat, RAD2DEG(d.fix.lon), d.err.lon,
                    d.fix.speed, d.err.speed, RAD2DEG(d.fix.heading), 
                    RAD2DEG(d.err.heading), ctime(&t));
                
                last_fix = steady_clock::now();
                m_had_fix = true;
            }
        }
    }
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
    milliseconds len(timeout);
    milliseconds wait(WAIT_PERIOD);
    auto start = steady_clock::now();
    auto now = start;
    bool hasFix = false;
    
    while (!(hasFix = HasFix()) && (timeout < 0 || (now - start) > len)) {
        sleep_for(wait);
        now = steady_clock::now();
    }
    return hasFix;
}

/**
 * Returns the latest GPS fix, if any.
 * @param d A pointer to the output location. If no value is present for that
 *          parameter, then that value is filled with NaN.
 */
void GPS::GetLatest(GPSData *d) {
    *d = m_data;
}