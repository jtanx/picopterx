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

Waypoints::Waypoints(Options *opts, std::deque<Coord2D> pts, WaypointMethod method)
: m_pts(pts)
, m_method(method)
, m_update_interval(100)
, m_waypoint_radius(1.2)
, m_waypoint_control(12)
, m_waypoint_idle(3000)
, m_pid1(0,0,0,0)
{
    Options empty;
    if (opts == NULL) {
        opts = &empty;
    }
    
    opts->SetFamily("WAYPOINTS");
    switch (method) {
        case WAYPOINT_SIMPLE: {
            double kp = opts->GetReal("SIMPLE_Kpxy", 1);
            double tr = opts->GetReal("SIMPLE_Trxy", 6);
            double td = opts->GetReal("SIMPLE_Tdxy", 0);
            m_update_interval = opts->GetInt("SIMPLE_UPDATE_INTERVAL", m_update_interval);
            m_waypoint_radius = opts->GetReal("SIMPLE_WAYPOINT_RADIUS", m_waypoint_radius);
            m_waypoint_control = opts->GetReal("SIMPLE_WAYPOINT_CONTROL_RANGE", m_waypoint_control);
            m_waypoint_idle = opts->GetInt("SIMPLE_WAYPOINT_IDLE_TIME", m_waypoint_idle);
            
            //Interval MUST be set before tunings.
            m_pid1.SetInterval(m_update_interval / 1000.0);
            m_pid1.SetTunings(kp, tr, td);
            m_pid1.SetInputLimits(-m_waypoint_control, 0);
            m_pid1.SetOutputLimits(0, opts->GetReal("SIMPLE_SPEED_LIMIT", 40));
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
    
    Log(LOG_INFO, "Authorisation acknowledged.");
    if (!fc->gps->WaitForFix(200)) {
        Log(LOG_WARNING, "No GPS fix; quitting.");
        return;
    }
    
    GPSData d;
    double copter_bearing;
    fc->gps->GetLatest(&d);
    
    if (std::isnan(d.fix.bearing)) {
        Log(LOG_WARNING, "No compass bearing present; falling back to bearing inferral.");
        if (!fc->InferBearing(&copter_bearing)) {
            Log(LOG_INFO, "Exiting waypoints navigation; no usable bearing.");
            return;
        }
    } else {
        Log(LOG_INFO, "Bearing determined from compass: %.2f", d.fix.bearing);
        copter_bearing = d.fix.bearing;
    }
    
    SetCurrentState(fc, STATE_WAYPOINTS_MOVING);
    Coord2D next_point = m_pts.front();
    m_pts.pop_front(); //ffs
    while (!fc->CheckForStop()) {
        double wp_distance, wp_bearing;
        
        if (!fc->gps->HasFix()) {
            Log(LOG_WARNING, "GPS Fix was lost! Falling back to manual mode.");
            fc->buzzer->Play(1000,100,100);
            fc->fb->Stop();
            return;
        }
        
        fc->gps->GetLatest(&d);
        if (!std::isnan(d.fix.bearing)) {
            copter_bearing = d.fix.bearing;
        }
        wp_distance = CoordDistance(d.fix, next_point);
        wp_bearing = CoordBearing(d.fix, next_point);
        
        //Log(LOG_INFO, "%.2f m away at a bearing of %.2f deg", wp_distance, wp_bearing);
        if (wp_distance < m_waypoint_radius) {
            Log(LOG_INFO, "At waypoint, idling...");
            fc->fb->Stop();
            
            SetCurrentState(fc, STATE_WAYPOINTS_IDLING);
            fc->Sleep(m_waypoint_idle);
            
            //Reset the controller
            m_pid1.Reset();
            if (m_pts.empty()) {
                Log(LOG_INFO, "Completed waypoint navigation.");
                break;
            } else {
                Log(LOG_INFO, "Moving to next waypoint.");
                SetCurrentState(fc, STATE_WAYPOINTS_MOVING);
                next_point = m_pts.front();
                m_pts.pop_front();
            }
        } else {
            FlightData d = {0};
            double speed;
            //Log(LOG_INFO, "%.2f", wp_distance);
            m_pid1.SetProcessValue(-wp_distance);
            speed = m_pid1.Compute();
            Log(LOG_INFO, "%.2f, %.2f", speed, wp_distance);
            
            d.aileron = speed * sin(DEG2RAD(wp_bearing - copter_bearing));
            d.elevator = speed * cos(DEG2RAD(wp_bearing - copter_bearing));
            //Log(LOG_INFO, "%d, %d", d.aileron, d.elevator);
            
            fc->fb->SetData(&d);
        }
        auto now = std::chrono::steady_clock::now();
        fc->Sleep(m_update_interval);
        auto diff = std::chrono::steady_clock::now() - now;
        Log(LOG_WARNING, "%d ms", std::chrono::duration_cast<std::chrono::milliseconds>(diff).count());
    }
    
    SetCurrentState(fc, STATE_WAYPOINTS_FINISHED);
    fc->fb->Stop();
}