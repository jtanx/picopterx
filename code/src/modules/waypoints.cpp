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
using std::chrono::seconds;
using std::chrono::milliseconds;
using std::chrono::steady_clock;
using std::chrono::duration_cast;

/**
 * Constructor. Creates a new waypoint maneuvering module.
 * @param [in] opts  A pointer to options, if any (NULL for defaults)
 * @param [in] pts The set of waypoints to move to.
 * @param [in] method The method for moving through the specified waypoints.
 */
Waypoints::Waypoints(Options *opts, std::deque<Waypoint> pts, WaypointMethod method)
: m_pts(pts)
, m_method(method)
, m_update_interval(100)
, m_waypoint_radius(1.2)
, m_waypoint_idle(3000)
, m_sweep_spacing(3)
, m_image_counter(0)
, m_finished{false}
, m_log("waypoints")
{
    Options empty;
    if (opts == NULL) {
        opts = &empty;
    }
    
    opts->SetFamily("WAYPOINTS");
    m_update_interval = opts->GetInt("UPDATE_INTERVAL", m_update_interval);
    m_waypoint_radius = opts->GetReal("WAYPOINT_RADIUS", m_waypoint_radius);
    m_waypoint_idle = opts->GetInt("WAYPOINT_IDLE_TIME", m_waypoint_idle);
    m_sweep_spacing = opts->GetInt("LAWNMOWER_SWEEP_SPACING", m_sweep_spacing);
    
    if (method == WAYPOINT_LAWNMOWER) {
        if (m_pts.size() < 2) {
            throw std::invalid_argument("Cannot do lawnmower with less than 2 waypoints");
        } else {
            int j = 0;
            m_pts = std::move(GenerateLawnmowerPattern(m_pts[0], m_pts[1]));
            for (const Waypoint &i : m_pts) {
                Log(LOG_INFO, "Lawnmower waypoint %d: (%.7f, %.7f)", j++, i.pt.lat, i.pt.lon);
            }
            m_waypoint_idle = opts->GetInt("LAWNMOWER_IDLE_TIME", 0);
        }
    } else if (method == WAYPOINT_SPIRAL) {
        if (m_pts.size() < 2) {
            throw std::invalid_argument("Cannot do spiral with less than 2 waypoints");
        } else {
            m_pts = std::move(GenerateSpiralPattern(m_pts[0], m_pts[1]));
            m_waypoint_idle = opts->GetInt("SPIRAL_IDLE_TIME", 0);
        }
    }
}

/** 
 * Constructor. Constructs with default settings.
 */
Waypoints::Waypoints(std::deque<Waypoint> pts, WaypointMethod method)
: Waypoints(NULL, pts, method) {}

/**
 * Destructor.
 */
Waypoints::~Waypoints() {
    
}

/**
 * Generates the sweeping lawnmower search pattern.
 * It will sweep from the start to end position along the shortest side.
 * @param [in] start The starting coordinate.
 * @param [in] end The ending coordinate.
 * @return The list of waypoints to travel to for the given lawnmower pattern.
 */
std::deque<Waypoints::Waypoint> Waypoints::GenerateLawnmowerPattern(Waypoint start, Waypoint end) {
    //Determine which way we are sweeping
    Waypoint sx = {{start.pt.lat, end.pt.lon}};
    double d1 = CoordDistance(start.pt, sx.pt), d2 = CoordDistance(sx.pt, end.pt);
    int points = static_cast<int>(std::min(d1, d2) / m_sweep_spacing);
    std::deque<Waypoint> pts;
    
    if (points != 0) {
        bool modlat = false;
        double frac;
        if (d1 > d2) {
            frac = (end.pt.lat - start.pt.lat) / points;
            modlat = true;
        } else {
            frac = (end.pt.lon - start.pt.lon) / points;
        }
        
        for (int i = 0; i < points; i++) {
            Waypoint v1{}, v2{};
            if (modlat) {
                v1.pt.lat = start.pt.lat + frac*i;
                v1.pt.lon = start.pt.lon;
                v2.pt.lat = v1.pt.lat;
                v2.pt.lon = end.pt.lon;
            } else {
                v1.pt.lat = start.pt.lat;
                v1.pt.lon = start.pt.lon + frac*i;
                v2.pt.lat = end.pt.lat;
                v2.pt.lon = v1.pt.lon;
            }
            
            if (i%2) {
                pts.push_back(v2);
                pts.push_back(v1);
            } else {
                pts.push_back(v1);
                pts.push_back(v2);
            }
        }
        
        if (points%2 == 0) {
            if (modlat) {
                sx.pt.lat = end.pt.lat;
                sx.pt.lon = start.pt.lon;
            }
            pts.push_back(sx);
        }
    } else {
        pts.push_back(start);
    }
    pts.push_back(end);
    
    return pts;
}

std::deque<Waypoints::Waypoint> Waypoints::GenerateSpiralPattern(Waypoint centre, Waypoint edge) {
    //Use a flat Earth approximation to generate the waypoints
    double radius = CoordDistance(centre.pt, edge.pt);
    double start_angle = 90-CoordBearing(centre.pt, edge.pt);
    double offset_x, offset_y;
    double earth_radius = 6378137;
    double delta = 360.0/(2*M_PI*radius/4.0); //Angle increment
    std::deque<Waypoint> pts;
    
    //Convert angle into -180 to 180, relative to positive x axis.
    if (start_angle < -180) {
        start_angle += 360;
    }
    
    //for (int alt = 0; alt < edge.pt.alt - centre.pt.alt; alt += 2) {
        for (double i = 0; i < 360; i += delta) {
            Waypoint pt{};
            offset_x = radius * cos(DEG2RAD(start_angle+i));
            offset_y = radius * sin(DEG2RAD(start_angle+i));
            
            offset_x /= earth_radius * cos(DEG2RAD(centre.pt.lat));
            offset_y /= earth_radius;
            
            pt.pt.lat = centre.pt.lat + RAD2DEG(offset_y);
            pt.pt.lon = centre.pt.lon + RAD2DEG(offset_x);
            pt.roi = centre.pt;
            pt.has_roi = true;
            
            if (i == 0) {
                pt.pt.alt = 0;//alt;
            }
            pts.push_back(pt);
        }
        //pts.push_back(starting pt)
    //}
    return pts;
}

/**
 * Runs the waypoint following code. Should only be called once, and only by
 * the FlightController class.
 * @param fc The flight controller that initiated this call.
 * @param opts Any user-specified options (unused).
 */
void Waypoints::Run(FlightController *fc, void *opts) {
    GPSData d;
    Waypoint next_point;
    int req_seq = 0, at_seq = 0;
    double wp_distance;
    std::vector<ObjectInfo> detected_objects;
    auto last_detection = steady_clock::now()-seconds(30); //Hysteresis for object detection
    
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
            wp_distance = CoordDistance(d.fix, next_point.pt);

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
                m_waypoint_idle / 1000.0f, next_point.pt.lat, next_point.pt.lon, 0, true);
            fc->fb->SetWaypointSpeed(3);
            if (next_point.has_roi) {
                fc->fb->SetRegionOfInterest(next_point.roi);
            } else {
                fc->fb->UnsetRegionOfInterest();
            }
        }
        
        //Coord3D cord = {1,0,0};
        //fc->fb->SetRegionOfInterest(cord);
        if (fc->cam && ((steady_clock::now()-last_detection) > seconds(3))) {
            fc->cam->GetDetectedObjects(&detected_objects);
            if (detected_objects.size() > 0) {
                ObjectInfo object = detected_objects.front();
                Log(LOG_INFO, "Detected object! Recording...");
                fc->buzzer->Play(500, 800, 100);
                //fc->fb->Stop();
                //fc->Sleep(700);
                
                std::string path = 
                    std::string(PICOPTER_HOME_LOCATION "/pics/wpt_") +
                    m_log.GetSerial() + "_" + 
                    std::to_string(m_image_counter++) + std::string(".jpg");
                
                fc->cam->TakePhoto(path);
                fc->gps->GetLatest(&d);
                m_log.Write(" Detected object: ID: %d", object.id);
                m_log.Write(" Location: (%.7f, %.7f) at %.2fm", 
                    d.fix.lat, d.fix.lon, d.fix.alt-d.fix.groundalt);
                m_log.Write(" Image: %s", path.c_str());
                m_log.Write(" Object count in frame: %d", detected_objects.size());
                last_detection = steady_clock::now();
                
                Log(LOG_INFO, "Continuing...");
                //fc->fb->SetGuidedWaypoint(req_seq, m_waypoint_radius,
                //    m_waypoint_idle / 1000.0f, next_point.lat, next_point.lon, 0, true);
            }
        }
        
        fc->Sleep(m_update_interval);
    }
    
    SetCurrentState(fc, STATE_WAYPOINTS_FINISHED);
    fc->fb->UnsetRegionOfInterest();
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
