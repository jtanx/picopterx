/**
 * @file waypoints.cpp
 * @brief Contains the waypoints handling code.
 */

#include "common.h"
#include "waypoints.h"
#include "pathplan.h"
#include "gridspace.h"
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
Waypoints::Waypoints(Options *opts, std::deque<Waypoint> pts, std::deque<std::deque<Coord3D> > zones, GridSpace *gridspace, WaypointMethod method)
: m_pts(pts)
, m_method(method)
, m_update_interval(100)
, m_waypoint_radius(1.2)
, m_waypoint_alt_radius(0.2)
, m_waypoint_alt_minimum(4)
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
    m_waypoint_alt_radius = opts->GetReal("WAYPOINT_ALT_RADIUS", m_waypoint_alt_radius);
    m_waypoint_alt_minimum = opts->GetReal("WAYPOINT_ALT_MINIMUM", m_waypoint_alt_minimum);
    m_waypoint_idle = opts->GetInt("WAYPOINT_IDLE_TIME", m_waypoint_idle);
    m_sweep_spacing = opts->GetInt("LAWNMOWER_SWEEP_SPACING", m_sweep_spacing);
    
    if (method == WAYPOINT_LAWNMOWER) {
        if (m_pts.size() < 2) {
            throw std::invalid_argument("Cannot do lawGridSpace *gridspace,nmower with less than 2 waypoints");
        } else {
            int j = 0;
            m_pts = std::move(GenerateLawnmowerPattern(m_pts[0], m_pts[1]));
            for (const Waypoint &i : m_pts) {
                Log(LOG_INFO, "Lawnmower waypoint %d: (%.7f, %.7f)", j++, i.pt.lat, i.pt.lon);
            }
            m_waypoint_idle = opts->GetInt("LAWNMOWER_IDLE_TIME", 0);
        }
    } else if (method == WAYPOINT_SPIRAL || method == WAYPOINT_SPIRAL_OUT) {
        if (m_pts.size() < 2) {
            throw std::invalid_argument("Cannot do spiral with less than 2 waypoints");
        } else {
            if (CoordDistance(m_pts[0].pt, m_pts[1].pt) < 0.5) {
                throw std::invalid_argument("Spiral radius is too small");
            }
            if (m_pts.size() >= 3) {
                if (CoordDistance(m_pts[0].pt, m_pts[2].pt) < 0.5) {
                    throw std::invalid_argument("Spiral radius is too small");
                }
                m_pts = std::move(GenerateSpiralPattern(m_pts[0], m_pts[1], m_pts[2], method == WAYPOINT_SPIRAL_OUT));
            } else {
                m_pts = std::move(GenerateSpiralPattern(m_pts[0], m_pts[1], m_pts[1], method == WAYPOINT_SPIRAL_OUT));
            }
            m_waypoint_idle = opts->GetInt("SPIRAL_IDLE_TIME", 0);
        }
    }
    
    //Avoid collision zones.
    if (zones.size() > 0 && gridspace != NULL) {
        PathPlan plan(gridspace);
        for (std::deque<Coord3D> zone : zones) {
            plan.addPolygon(zone);
        }
        m_pts = std::move(plan.generateFlightPlan(m_pts));
    }
    
    //Write out the waypoints to file.
    for (size_t i = 0; i < m_pts.size(); i++) {
        if (m_pts[i].has_roi) {
            m_log.Write(": Waypoint %d: (%.7f, %.7f, %.3f) [%.7f, %.7f, %.3f]",
            i+1, m_pts[i].pt.lat, m_pts[i].pt.lon, m_pts[i].pt.alt,
            m_pts[i].roi.lat, m_pts[i].roi.lon, m_pts[i].roi.alt);
        } else {
            m_log.Write(": Waypoint %d: (%.7f, %.7f, %.3f) []",
            i+1, m_pts[i].pt.lat, m_pts[i].pt.lon, m_pts[i].pt.alt);
        }
    }
}

/** 
 * Constructor. Constructs with default settings.
 */
Waypoints::Waypoints(std::deque<Waypoint> pts, std::deque<std::deque<Coord3D> > zones, GridSpace *gridspace, WaypointMethod method)
: Waypoints(NULL, pts, zones, gridspace, method) {}

Waypoints::Waypoints(std::deque<Waypoint> pts, WaypointMethod method)
: Waypoints(NULL, pts, std::deque<std::deque<Coord3D> >(), NULL, method) {}

Waypoints::Waypoints(Options *opts, std::deque<Waypoint> pts, WaypointMethod method)
: Waypoints(NULL, pts, std::deque<std::deque<Coord3D> >(), NULL, method) {}

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

/**
 * Generates a 3D spiral pattern.
 * @param [in] centre The centre point to rotate about.
 * @param [in] edge1 Defines the starting position and altitude.
 * @param [in] edge2 Defines the ending position and altitude.
 * @param [in] face_out Boolean indicating whether or not to face outwards or
 *                      inwards when spiralling.
 * @return A list of waypoints needed to move in a spiral pattern.
 */
std::deque<Waypoints::Waypoint> Waypoints::GenerateSpiralPattern(Waypoint centre, Waypoint edge1, Waypoint edge2, bool face_out) {
    double start_radius = CoordDistance(centre.pt, edge1.pt);
    double end_radius   = CoordDistance(centre.pt, edge2.pt);
    double start_angle  = CoordBearingX(centre.pt, edge1.pt);
    double end_angle    = CoordBearingX(centre.pt, edge2.pt);
    double climb_rate;
    int revs = 1;
    std::deque<Waypoint> pts;
    
    //Cannot have altitude change if we don't know both in absolute measures.
    if (edge1.pt.alt == 0 || edge2.pt.alt == 0) {
        edge1.pt.alt = edge2.pt.alt = 0;
    }
    
    //The climb rate, in m/revolution
    climb_rate = std::min(1.0, edge2.pt.alt-edge1.pt.alt);
    if (climb_rate > 0) {
        revs = ceil((edge2.pt.alt-edge1.pt.alt)/climb_rate);
    }
    
    for (int i = 0; i < revs;i++) {
        for (double j = 0; j < 360;) {
            Waypoint pt{};
            double pct = (i + j/360.0)/revs;
            double radius = start_radius + (end_radius-start_radius)*pct;
            double alt = edge1.pt.alt + (edge2.pt.alt-edge1.pt.alt)*pct;
            double angle = start_angle + (end_angle-start_angle)*pct;
            
            pt.pt = CoordAddOffset(centre.pt, radius, angle+j);
            pt.pt.alt = alt;
            pt.roi = face_out ? CoordAddOffset(centre.pt, radius+10, angle+j) :
                     centre.pt;
            pt.has_roi = true;
            
            pts.push_back(pt);
            j += 360/(2*M_PI*radius/4);
        }
    }
    
    edge2.roi = face_out ? CoordAddOffset(centre.pt, end_radius+10, end_angle) :
                centre.pt;
    edge2.has_roi = true;
    pts.push_back(edge2);
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
    double wp_distance, wp_alt_delta;
    std::vector<ObjectInfo> detected_objects;
    auto last_detection = steady_clock::now()-seconds(30); //Hysteresis for object detection
    //Write out GPS data at about 2Hz.
    int writeout_interval = std::max(500/m_update_interval, 1);
    int writeout_counter = 0;
    
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
        
        fc->gps->GetLatest(&d);
        if ((writeout_counter++ % writeout_interval) == 0) {
            m_log.Write(": At: (%.7f, %.7f, %.3f) [%.3f]",
                d.fix.lat, d.fix.lon, d.fix.alt - d.fix.groundalt, d.fix.heading);
        }
            
        if (at_seq < req_seq) {
            wp_distance = CoordDistance(d.fix, next_point.pt);
            wp_alt_delta = next_point.pt.alt != 0 ? 
                std::fabs((d.fix.alt-d.fix.groundalt)-next_point.pt.alt) : 0;

            if (wp_distance < m_waypoint_radius && wp_alt_delta < m_waypoint_alt_minimum) {
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
            
            //Aus regs: Cannot fly above 400ft (138m). We'll just limit it to 100m.
            next_point.pt.alt = next_point.pt.alt >= m_waypoint_alt_minimum ? 
                std::min(next_point.pt.alt, 100.0) : 0;
            fc->fb->SetGuidedWaypoint(req_seq++, m_waypoint_radius,
                m_waypoint_idle / 1000.0f, next_point.pt, next_point.pt.alt == 0);
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
                m_log.Write(": Detected object: ID: %d", object.id);
                m_log.Write(": Location: (%.7f, %.7f, %.3f) [%.3f]", 
                    d.fix.lat, d.fix.lon, d.fix.alt-d.fix.groundalt, d.fix.heading);
                m_log.Write(": Image: %s", path.c_str());
                m_log.Write(": Object count in frame: %d", detected_objects.size());
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
