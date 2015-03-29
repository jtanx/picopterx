/**
 * @file waypoints.cpp
 * @brief Contains the waypoints handling code.
 */

#include "common.h"
#include "waypoints.h"

using namespace picopter;
using namespace picopter::navigation;

Waypoints::Waypoints(Options *opts, std::deque<Coord2D> pts, WaypointMethod method)
: m_pts(pts)
, m_method(method)
, m_update_interval(100)
, m_waypoint_radius(1.2)
, m_waypoint_control(2.4)
, m_pid1(0,0,0,0)
{
    Options empty;
    if (opts == NULL) {
        opts = &empty;
    }
    
    opts->SetFamily("WAYPOINTS");
    switch (method) {
        case WAYPOINT_SIMPLE: {
            double kp = opts->GetReal("SIMPLE_Kpxy", 10);
            double tr = opts->GetReal("SIMPLE_Trxy", 500);
            double td = opts->GetReal("SIMPLE_Tdxy", 0);
            m_update_interval = opts->GetInt("SIMPLE_UPDATE_INTERVAL", m_update_interval);
            m_waypoint_radius = opts->GetReal("SIMPLE_WAYPOINT_RADIUS", m_waypoint_radius);
            m_waypoint_control = opts->GetReal("SIMPLE_WAYPOINT_CONTROL_RANGE", m_waypoint_control);
            
            m_pid1.SetTunings(kp, tr, td);
            m_pid1.SetInterval(m_update_interval / 1000.0);
            m_pid1.SetInputLimits(-m_waypoint_control, m_waypoint_control);
            m_pid1.SetOutputLimits(0, opts->GetReal("SIMPLE_SPEED_LIMIT", 35));
            m_pid1.SetSetPoint(0);
        } break;
        default:
            throw std::invalid_argument("Unknown waypoint method");
    }
}

Waypoints::Waypoints(std::deque<Coord2D> pts, WaypointMethod method)
: Waypoints(NULL, pts, method) {}

Waypoints::~Waypoints() {
    
}

void Waypoints::Run(FlightController *fc, void *opts) {
    Log(LOG_INFO, "Waypoints movement initiated; awaiting authorisation...");
    SetCurrentState(fc, STATE_AWAITING_AUTH);
    
    if (!fc->WaitForAuth()) {
        Log(LOG_INFO, "All stop acknowledged; quitting!");
        return;
    } else if (m_pts.empty()) {
        Log(LOG_WARNING, "No waypoints specified; quitting.");
        return;
    }
    
    double copter_bearing;
    Log(LOG_INFO, "Authorisation acknowledged.");
    if (!fc->InferBearing(&copter_bearing)) {
        Log(LOG_INFO, "Exiting waypoints navigation; no usable bearing.");
        return;
    }
    
    Coord2D next_point = m_pts.front();
    m_pts.pop_front(); //ffs
    while (!fc->CheckForStop()) {
        GPSData d;
        double wp_distance, wp_bearing;
        
        fc->gps->GetLatest(&d);
        copter_bearing = d.fix.heading;
        wp_distance = CoordDistance(d.fix, next_point);
        wp_bearing = CoordBearing(d.fix, next_point);
        
        if (wp_distance < m_waypoint_radius) {
            Log(LOG_INFO, "At waypoint");
            if (m_pts.empty()) {
                Log(LOG_INFO, "Completed waypoint navigation.");
                break;
            } else {
                next_point = m_pts.front();
                m_pts.pop_front();
            }
        } else {
            FlightData d = {0};
            double speed;
            m_pid1.SetProcessValue(wp_distance);
            speed = m_pid1.Compute();
            d.aileron = speed * sin(wp_bearing - copter_bearing);
            d.elevator = speed * cos(wp_bearing - copter_bearing);
            
            fc->fb->SetData(&d);
        }
        fc->Sleep(m_update_interval);
    }
    
    fc->fb->Stop();
}