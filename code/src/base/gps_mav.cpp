/**
 * @file gps_mav.cpp
 * @brief GPS interaction code.
 * Uses MAVLink to interact with the GPS
 */

#include "common.h"
#include "gps_mav.h"

using namespace picopter;
using std::chrono::duration_cast;
using std::chrono::seconds;
using std::chrono::milliseconds;
using steady_clock = std::chrono::steady_clock;
using namespace std::placeholders;

/**
 * Constructor. Establishes a connection to gpsd, assuming it is running
 * on the default gpsd port. Starts the worker thread to receive GPS data.
 * @param opts A pointer to options, if any (NULL for defaults)
 * @throws std::invalid_argument if a connection to gpsd cannot be established.
 */
GPSMAV::GPSMAV(FlightBoard *fb, Options *opts)
: GPS(opts)
, m_had_fix(false)
, m_log("gps_mav")
{
    fb->RegisterHandler(MAVLINK_MSG_ID_GLOBAL_POSITION_INT,
        std::bind(&GPSMAV::GPSInput, this, _1));
    fb->RegisterHandler(MAVLINK_MSG_ID_GPS_RAW_INT,
        std::bind(&GPSMAV::GPSInput, this, _1));
    Log(LOG_INFO, "GPS Started!");
}

/**
 * Constructor. Constructs a new GPS with default settings.
 */
GPSMAV::GPSMAV(FlightBoard *fb) : GPSMAV(fb, NULL) {}

/**
 * Destructor.
 */
GPSMAV::~GPSMAV() {
}

/**
 * Main worker callback.
 */
void GPSMAV::GPSInput(const mavlink_message_t *msg) {
    //Fixme...
    static auto last_fix = steady_clock::now() - seconds(m_fix_timeout);
    
    m_last_fix = duration_cast<seconds>(steady_clock::now() - last_fix).count();
    if (m_had_fix && !HasFix()) {
        Log(LOG_WARNING, "Lost the GPS fix. Last fix: %d seconds ago.",
            m_last_fix.load());
        m_log.Write(": Lost fix");
        m_had_fix = false;
    }

    if (msg->msgid == MAVLINK_MSG_ID_GLOBAL_POSITION_INT) {
        mavlink_global_position_int_t pos;
        mavlink_msg_global_position_int_decode(msg, &pos);
        std::unique_lock<std::mutex> lock(m_worker_mutex);
        
        GPSData &d = m_data;
        d.fix.lat = pos.lat*1e-7;
        d.fix.lon = pos.lon*1e-7;
        d.fix.alt = pos.alt*1e-3;
        d.fix.groundalt = d.fix.alt - pos.relative_alt*1e-3;
        if (pos.hdg != UINT16_MAX) {
            d.fix.heading = pos.hdg*1e-2;
        }
        lock.unlock();

        m_log.Write(": (%.7f, %.7f, %.3f) [%.3f]",
            d.fix.lat, d.fix.lon, pos.relative_alt*1e-3, d.fix.heading);
        
        last_fix = steady_clock::now();
        m_had_fix = true;
    }
}