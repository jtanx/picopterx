/**
 * @file gps_naza.cpp
 * @brief GPS interaction code.
 * Uses the NAZA decoder to interact with the NAZA GPS
 */

#include "common.h"
#include "gps_naza.h"
#include "NazaDecoderLib.h"
#include <wiringSerial.h>

using namespace picopter;
using std::chrono::duration_cast;
using std::chrono::seconds;
using std::chrono::milliseconds;
using steady_clock = std::chrono::steady_clock;
using std::this_thread::sleep_for;

/**
 * Constructor. Opens the RPi's serial port and reads off data.
 */
GPSNaza::GPSNaza(Options *opts)
: GPS(opts)
, m_had_fix(false)
, m_log("gps_naza")
{
    m_decoder = new NazaDecoderLib();
    m_fd = serialOpen("/dev/ttyAMA0", 115200);
    if (m_fd < 0) {
        throw std::invalid_argument("Could not open ttyAMA0");
    }
    m_worker = std::thread(&GPSNaza::GPSLoop, this);
}

/**
 * Constructor. Constructs a new GPS with default settings.
 */
GPSNaza::GPSNaza() : GPSNaza(NULL) {}

/**
 * Destructor. Stops the worker thread and waits for it to exit.
 */
GPSNaza::~GPSNaza() {
    m_quit = true;
    m_worker.join();
    delete m_decoder;
    serialClose(m_fd);
}

/**
 * Main worker thread. Polls gpsd for new GPS data and updates as necessary.
 */
void GPSNaza::GPSLoop() {
    auto last_fix = steady_clock::now() - seconds(m_fix_timeout);
    
    Log(LOG_INFO, "GPS Started!");
    while (!m_quit) {
        int data_avail = serialDataAvail(m_fd);
        m_last_fix = duration_cast<seconds>(steady_clock::now() - last_fix).count();
        if (m_had_fix && !HasFix()) {
            Log(LOG_WARNING, "Lost the GPS fix. Last fix: %d seconds ago.",
                m_last_fix.load());
            m_log.Write(": Lost fix");
            m_had_fix = false;
        }
        
        if (data_avail < 0) {
            m_last_fix = m_fix_timeout;
        } else if (data_avail == 0) {
            sleep_for(milliseconds(WAIT_PERIOD));
        } else {
            uint8_t message = m_decoder->decode(serialGetchar(m_fd));
            switch (message) {
                case NAZA_MESSAGE_GPS: {
                    if (m_decoder->getFixType() != NazaDecoderLib::NO_FIX) {
                        GPSData d;
                        
                        if (!m_had_fix) {
                        	Log(LOG_INFO, "Got fix! (%.3f, %.3f)", m_decoder->getLat(), m_decoder->getLon());
                        	m_had_fix = true;
                        }
                        
                        d.fix.lat = m_decoder->getLat();
                        d.fix.lon = m_decoder->getLon();
                        d.fix.speed = m_decoder->getSpeed();
                        d.fix.heading = m_decoder->getCog();
                        m_data = d;
                        
                        m_log.Write(": (%.6f, %.6f) [%.2f at %.2f]",
                            d.fix.lat, d.fix.lon, d.fix.speed, d.fix.heading);
                        last_fix = steady_clock::now();
                    }
                } break;
                case NAZA_MESSAGE_COMPASS: {
                    GPSData current = m_data;
                    current.fix.bearing = m_decoder->getHeadingNc();
                    m_data = current;
                } break;
            }
        }
    }
}
