/**
 * @file gps_gpsd.cpp
 * @brief GPS interaction code.
 * Uses gpsd to interact with the GPS
 */

#include "common.h"
#include "gps_gpsd.h"
#include "libgpsmm.h"

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
GPSGPSD::GPSGPSD(Options *opts)
: GPS(opts)
, m_cycle_timeout(CYCLE_TIMEOUT_DEFAULT)
, m_had_fix(false)
, m_log("gps_gpsd")
{
    m_gps_rec = new gpsmm("localhost", DEFAULT_GPSD_PORT);
    if (m_gps_rec->stream(WATCH_ENABLE|WATCH_JSON) == NULL) {
        delete m_gps_rec;
        throw std::invalid_argument("gpsd is not running");
    }
    m_worker = std::thread(&GPSGPSD::GPSLoop, this);
    
    if (opts) {
        opts->SetFamily("GPS");
        m_cycle_timeout = opts->GetInt("CYCLE_TIMEOUT", CYCLE_TIMEOUT_DEFAULT);
    }
}

/**
 * Constructor. Constructs a new GPS with default settings.
 */
GPSGPSD::GPSGPSD() : GPSGPSD(NULL) {}

/**
 * Destructor. Stops the worker thread and waits for it to exit.
 */
GPSGPSD::~GPSGPSD() {
    m_quit = true;
    m_worker.join();
    delete m_gps_rec;
}

/**
 * Main worker thread. Polls gpsd for new GPS data and updates as necessary.
 */
void GPSGPSD::GPSLoop() {
    auto last_fix = steady_clock::now() - seconds(m_fix_timeout);
    bool read_fail = false;
    
    Log(LOG_INFO, "GPS Started!");
    while (!m_quit) {
        m_last_fix = duration_cast<seconds>(steady_clock::now() - last_fix).count();
        if (m_had_fix && !HasFix()) {
            Log(LOG_WARNING, "Lost the GPS fix. Last fix: %d seconds ago.",
                m_last_fix.load());
            m_log.Write(": Lost fix");
            m_had_fix = false;
        }

        if (m_gps_rec->waiting(m_cycle_timeout) && !m_quit) {
            struct gps_data_t* data;
            if ((data = m_gps_rec->read()) == NULL) {
                if (!read_fail) {
                    Log(LOG_WARNING, "Failed to read GPS data");
                    read_fail = true;
                }
                sleep_for(milliseconds(200));
            } else if ((data->set & LATLON_SET) && (data->set & SPEED_SET)) {
                std::unique_lock<std::mutex> lock(m_worker_mutex);
                GPSData &d = m_data;
                //GPSData d2 = d;
                d.fix.lat = data->fix.latitude;
                d.fix.lon = data->fix.longitude;
                d.fix.speed = data->fix.speed;
                
                if (data->set & TRACK_SET) {
                    d.fix.heading = data->fix.track;
                    //std::cout << "CALC " << RAD2DEG(TRUEBEARING(navigation::CoordBearing(d2.fix, d.fix))) << std::endl;
                    if (data->set & TRACKERR_SET) {
                        d.err.heading = data->fix.epd;
                    }
                }
                if (data->satellites_used > 4 && data->set & ALTITUDE_SET) {
                    d.fix.alt = data->fix.altitude;
                    if (data->set & VERR_SET) {
                        d.err.alt = data->fix.epv;
                    }
                    if (std::isnan(d.fix.groundalt) || d.fix.alt < d.fix.groundalt) {
                        d.fix.groundalt = d.fix.alt;
                        Log(LOG_INFO, "Using %.2fm as the ground altitude.", d.fix.groundalt);
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
                lock.unlock();
                
                m_log.Write(": (%.6f +/- %.1fm, %.6f +/- %.1fm) [%.2f +/- %.2f at %.2f +/- %.2f]",
                    d.fix.lat, d.err.lat, d.fix.lon, d.err.lon,
                    d.fix.speed, d.err.speed, d.fix.heading, d.err.heading);
                
                last_fix = steady_clock::now();
                m_had_fix = true;
                read_fail = false;
            }
        }
    }
}