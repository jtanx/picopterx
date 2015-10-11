/**
 * @file user_tracker.cpp
 * @brief The user tracking code.
 */

#include "common.h"
#include "user_tracker.h"

using namespace picopter;
using namespace picopter::navigation;
using std::chrono::steady_clock;
using std::chrono::milliseconds;
using std::chrono::microseconds;
using std::chrono::seconds;
using std::chrono::duration_cast;
using std::this_thread::sleep_for;

/**
 * Constructor; creates a new user tracking module.
 * @param opts A pointer to options, if any (NULL for defaults)
 */
UserTracker::UserTracker(Options *opts)
: m_wpt{}
, m_geofence_sw{-31.9803622462528, 115.817576050758}
, m_geofence_ne{-31.9797547847258, 115.818262696266}
, m_wpt_available(false)
, m_finished{false}
{
}

/** 
 * Constructor. Constructs with default settings.
 */
UserTracker::UserTracker() : UserTracker(NULL) {}

/**
 * Destructor.
 */
UserTracker::~UserTracker() {}

/**
 * Runs the user tracking code. Should only be called once, and only by
 * the FlightController class.
 * @param fc The flight controller that initiated this call.
 * @param opts Any user-specified options (unused).
 */
void UserTracker::Run(FlightController *fc, void *opts) {
    Log(LOG_INFO, "User tracking initiated; awaiting authorisation...");
    SetCurrentState(fc, STATE_AWAITING_AUTH);
    
    if (!fc->WaitForAuth()) {
        Log(LOG_INFO, "All stop acknowledged; quitting!");
        return;
    }
    
    Log(LOG_INFO, "Authorisation acknowledged; tracking user!");
    if (!fc->gps->WaitForFix(200)) {
        Log(LOG_WARNING, "No GPS fix; quitting.");
        return;
    }
    
    int seq = 0;
    std::unique_lock<std::mutex> lock(m_worker_mutex);
    SetCurrentState(fc, STATE_TRACKING_USER);
    while (!fc->CheckForStop()) {
        m_signaller.wait_for(lock, seconds(1), [this,fc]{return m_wpt_available || fc->CheckForStop();});
        if (m_wpt_available) {
            fc->fb->SetGuidedWaypoint(seq++, 1, 0, m_wpt, true);
            m_wpt_available = false;
        }
    }
    
    fc->fb->Stop();
    m_finished = true;
}

/**
 * Updates the currently held user position.
 * @param [in] wpt The location of the user
 */
void UserTracker::UpdateUserPosition(Coord2D wpt) {
    if (CoordInBounds(wpt, m_geofence_sw, m_geofence_ne)) {
        std::unique_lock<std::mutex> lock(m_worker_mutex);
        m_wpt.lat = wpt.lat;
        m_wpt.lon = wpt.lon;
        m_wpt.alt = 0;
        m_wpt_available = true;
        lock.unlock();
        m_signaller.notify_one();
        Log(LOG_DEBUG, "Got user wpt: %.6f, %.6f", wpt.lat, wpt.lon);
    } else {
        Log(LOG_DEBUG, "Rejected user wpt (outside geofence): %.6f, %.6f",
            wpt.lat, wpt.lon);
    }
}

/**
 * Indicates whether or not the task has completed running.
 * @return true iff the task has completed running.
 */
bool UserTracker::Finished() {
    return m_finished;
}