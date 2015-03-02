/**
 * @file flightcontroller.cpp
 * @brief Base controller. Ties in all the equipment (sensors and actuators).
 * @todo Add in camera things.
 */

#include "picopter.h"

using namespace picopter;

/**
 * Helper method to initialise a base module.
 * @tparam Item The class of the base module to be initialised.
 * @param what A description of what the base module is.
 * @param pt A reference to the pointer where the result will be stored.
 * @param opts A pointer to the options instance, or NULL.
 * @param b The buzzer.
 * @param required Indicated if this module is required.
 * @param retries The number of retries to make if initialisation fails.
 */
template <typename Item>
void InitialiseItem(const char *what, Item* &pt, Options *opts, Buzzer *b, bool required, int retries = -1) {
    pt = nullptr;
    for (int i = 0; !pt && (retries < 0 || i <= retries); i++) {
        try {
            pt = new Item(opts);
        } catch (const std::invalid_argument &e) {
            Log(LOG_WARNING, "Failed to initialise %s (%s); retrying in 1 second...", what, e.what());
            b->play(500, 20, 100);
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
    }
    
    if (!pt) {
        if (required) {
            throw std::invalid_argument(what);
        } else {
            Log(LOG_WARNING, "Failed to initialise %s; skipping.", what);
        }
    }
}

/**
 * The flight controller constructor.
 * Initialises all members as necessary.
 * @throws std::invalid_argument if a required component fails to initialise.
 */
FlightController::FlightController(Options *opts)
: fb(m_fb)
, imu(m_imu)
, gps(m_gps)
, buzzer(m_buzzer)
{
    m_buzzer = new Buzzer();
    
    InitialiseItem("GPS", m_gps, opts, m_buzzer, true, 3);
    InitialiseItem("IMU", m_imu, opts, m_buzzer, false, 1);
    InitialiseItem("flight board", m_fb, opts, m_buzzer, true, 3);
    
    m_buzzer->playWait(200, 200, 100);
}

/**
 * Constructor. Constructs a new flight controller with default settings.
 */
FlightController::FlightController() : FlightController(NULL) {}

/**
 * Destructor. Stops/cleans up the base components (depends on RAII)
 */
FlightController::~FlightController() {
    delete m_fb;
    delete m_imu;
    delete m_gps;
    delete m_buzzer;
}