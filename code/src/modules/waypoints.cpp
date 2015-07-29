/**
 * @file waypoints.cpp
 * @brief Contains the waypoints handling code.
 */

#include "common.h"
#include "waypoints.h"
#define _USE_MATH_DEFINES
#include <cmath>

using namespace picopter;
using namespace picopter::navigation;

/**
 * Constructor. Creates a new waypoint maneuvering module.
 * @param [in] opts  A pointer to options, if any (NULL for defaults)
 * @param [in] pts The set of waypoints to move to.
 * @param [in] method The method for moving through the specified waypoints.
 */
Waypoints::Waypoints(Options *opts, std::deque<Coord2D> pts, WaypointMethod method)
: m_pts(pts)
, m_method(method)
, m_update_interval(100)
, m_waypoint_radius(1.2)
, m_waypoint_idle(3000)
, m_finished{false}
{
    Options empty;
    if (opts == NULL) {
        opts = &empty;
    }
    
    opts->SetFamily("WAYPOINTS");
    m_update_interval = opts->GetInt("UPDATE_INTERVAL", m_update_interval);
    m_waypoint_radius = opts->GetReal("WAYPOINT_RADIUS", m_waypoint_radius);
    m_waypoint_idle = opts->GetInt("WAYPOINT_IDLE_TIME", m_waypoint_idle);
}

/** 
 * Constructor. Constructs with default settings.
 */
Waypoints::Waypoints(std::deque<Coord2D> pts, WaypointMethod method)
: Waypoints(NULL, pts, method) {}

/**
 * Destructor.
 */
Waypoints::~Waypoints() {
    
}

/**
 * Runs the waypoint following code. Should only be called once, and only by
 * the FlightController class.
 * @param fc The flight controller that initiated this call.
 * @param opts Any user-specified options (unused).
 */
void Waypoints::Run(FlightController *fc, void *opts) {
    GPSData d;
    Coord2D next_point;
    int req_seq = 0, at_seq = 0;
    double wp_distance;
    
    Log(LOG_INFO, "Waypoints movement initiated; awaiting authorisation...");
    SetCurrentState(fc, STATE_AWAITING_AUTH);
    
    if (!fc->WaitForAuth()) {
        Log(LOG_INFO, "All stop acknowledged; quitting!");
        return;
    } else if (m_pts.empty()) {
        Log(LOG_WARNING, "No waypoints specified; quitting.");
        return;
    }
    
    Log(LOG_INFO, "Authorisation acknowledged.");
    if (!fc->gps->WaitForFix(200)) {
        Log(LOG_WARNING, "No GPS fix; quitting.");
        return;
    }
    
    SetCurrentState(fc, STATE_WAYPOINTS_MOVING);
    while (!fc->CheckForStop()) {
        if (!fc->gps->HasFix()) {
            Log(LOG_WARNING, "GPS Fix was lost! Falling back to manual mode.");
            fc->buzzer->Play(1000,100,100);
            fc->fb->Stop();
            return;
        }
        
        if (at_seq < req_seq) {
            fc->gps->GetLatest(&d);
            wp_distance = CoordDistance(d.fix, next_point);

            if (wp_distance < m_waypoint_radius) {
                Log(LOG_INFO, "At waypoint, idling...");
                fc->buzzer->Play(1000, 1000, 100);
                at_seq++;
                SetCurrentState(fc, STATE_WAYPOINTS_IDLING);
                fc->Sleep(m_waypoint_idle);
            }
        } else if (m_pts.empty()) {
            Log(LOG_INFO, "Completed waypoint navigation.");
            fc->buzzer->Play(2000, 2000, 100);
            break;
        } else {
            Log(LOG_INFO, "Moving to next waypoint.");
            fc->buzzer->Play(1000, 600, 100);
            next_point = m_pts.front();
            m_pts.pop_front();
            SetCurrentState(fc, STATE_WAYPOINTS_MOVING);
            fc->fb->SetGuidedWaypoint(req_seq++, m_waypoint_radius,
                m_waypoint_idle / 1000.0f, next_point.lat, next_point.lon, 0, true);
        }
        fc->Sleep(m_update_interval);
    }
    
    SetCurrentState(fc, STATE_WAYPOINTS_FINISHED);
    fc->fb->Stop();
    m_finished = true;
}

/**
 * Indicates whether or not the task has completed running.
 * @return true iff the task has completed running.
 */
bool Waypoints::Finished() {
    return m_finished;
}