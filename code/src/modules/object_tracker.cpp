/**
 * @file object_tracker.cpp
 * @brief The object tracking code.
 */

#include "common.h"
#include "object_tracker.h"

using namespace picopter;
using namespace picopter::navigation;
using std::chrono::steady_clock;
using std::chrono::milliseconds;
using std::chrono::microseconds;
using std::chrono::seconds;
using std::chrono::duration_cast;
using std::this_thread::sleep_for;

static void printFlightData(FlightData*);

ObjectTracker::ObjectTracker(Options *opts, int camwidth, int camheight, TrackMethod method)
: m_camwidth(camwidth)
, m_camheight(camheight)
, m_pidx(0,0,0,0.03)
, m_pidy(0,0,0,0.03)
, m_track_method{method}
, SEARCH_GIMBAL_LIMIT(60)
{
    Options clear;
    if (!opts) {
        opts = &clear;
    }
    
    //Observation mode: Don't actually send the commands to the props
    opts->SetFamily("GLOBAL");
    m_observation_mode = opts->GetBool("OBSERVATION_MODE", false);

    //The gain has been configured for a 320x240 image, so scale accordingly.
    opts->SetFamily("OBJECT_TRACKER");
    TRACK_TOL = opts->GetInt("TRACK_TOL", m_camwidth/7);
    TRACK_Kpx = opts->GetReal("TRACK_Kpx", 50);
    TRACK_Kpy = opts->GetReal("TRACK_Kpy", 30);
    TRACK_TauIx = opts->GetReal("TRACK_TauIx", 5);//3.8);
    TRACK_TauDx = opts->GetReal("TRACK_TauDx", 0.004);//0.008);
    TRACK_TauIy = opts->GetReal("TRACK_TauIy", 4);
    TRACK_TauDy = opts->GetReal("TRACK_TauDy", 0.004);
    TRACK_SPEED_LIMIT_X = opts->GetInt("TRACK_SPEED_LIMIT_X", 40);
    TRACK_SPEED_LIMIT_Y = opts->GetInt("TRACK_SPEED_LIMIT_Y", 50);
    TRACK_SETPOINT_X = opts->GetReal("TRACK_SETPOINT_X", 0);
    TRACK_SETPOINT_Y = opts->GetReal("TRACK_SETPOINT_Y", 0);

    m_pidx.SetTunings(TRACK_Kpx, TRACK_TauIx, TRACK_TauDx);
    m_pidx.SetInputLimits(-M_PI/2, M_PI/2);
    m_pidx.SetOutputLimits(-TRACK_SPEED_LIMIT_X, TRACK_SPEED_LIMIT_X);
    m_pidx.SetSetPoint(TRACK_SETPOINT_X);
    
    m_pidy.SetTunings(-TRACK_Kpy, TRACK_TauIy, TRACK_TauDy);
    m_pidy.SetInputLimits(-8,8);
    m_pidy.SetOutputLimits(-TRACK_SPEED_LIMIT_Y, TRACK_SPEED_LIMIT_Y);
    m_pidy.SetSetPoint(TRACK_SETPOINT_Y);
}

ObjectTracker::ObjectTracker(int camwidth, int camheight, TrackMethod method)
: ObjectTracker(NULL, camwidth, camheight, method) {}

ObjectTracker::~ObjectTracker() {
    
}

ObjectTracker::TrackMethod ObjectTracker::GetTrackMethod() {
    return m_track_method.load(std::memory_order_relaxed);
}

void ObjectTracker::SetTrackMethod(TrackMethod method) {
    Log(LOG_INFO, "Track method: %d", method);
    m_track_method.store(method, std::memory_order_relaxed);
}

void ObjectTracker::Run(FlightController *fc, void *opts) {
    if (fc->cam == NULL) {
        Log(LOG_WARNING, "Not running object detection - no usable camera!");
        return;
    } /*else if (!fc->gps->WaitForFix(30)) {
        Log(LOG_WARNING, "Not running object detection - no GPS fix!");
        return;
    }*/
    
    Log(LOG_INFO, "Object detection initiated; awaiting authorisation...");
    SetCurrentState(fc, STATE_AWAITING_AUTH);
    if (!fc->WaitForAuth()) {
        Log(LOG_INFO, "All stop acknowledged; quitting!");
        return;
    }
    
    Log(LOG_INFO, "Authorisation acknowledged. Finding object to track...");
    SetCurrentState(fc, STATE_TRACKING_SEARCHING);
    
    Point2D detected_object = {0,0};
    Point2D input_limits = {m_camwidth/2.0, m_camheight/2.0};
    Point3D object_body_coords = {0,0,0};  //location of the target in body coordinates
    Point3D object_limits = {0,0,0}; //Location limits in body coordinates

    std::vector<Point2D> locations;
    FlightData course;
    GPSData gps_position;
    auto last_fix = steady_clock::now() - seconds(2);
    bool had_fix = false;

    while (!fc->CheckForStop()) {
        double update_rate = 1.0 / fc->cam->GetFramerate();
        auto sleep_time = microseconds((int)(1000000*update_rate));

        fc->cam->GetDetectedObjects(&locations);
        fc->fb->GetData(&course);
        fc->gps->GetLatest(&gps_position);
        
        //Do we have an object?
        if (locations.size() > 0) {                                             
            detected_object = locations.front();
            SetCurrentState(fc, STATE_TRACKING_LOCKED);
            
            //Set PID update intervals
            m_pidx.SetInterval(update_rate);
            m_pidy.SetInterval(update_rate);
            
            //Determine trajectory to track the object (PID control)
            EstimatePositionFromImageCoords(&gps_position, &course, &detected_object, &object_body_coords);
            //Determine input limits to prevent integral windup
            EstimatePositionFromImageCoords(&gps_position, &course, &input_limits, &object_limits);
            m_pidx.SetInputLimits(-object_limits.x, object_limits.x);
            m_pidy.SetInputLimits(-object_limits.y, object_limits.y);
            Log(LOG_INFO, "LX: %.2f, LY: %.2f", object_limits.x, object_limits.y);

            if (!m_observation_mode) {
                CalculateTrackingTrajectory(fc, &course, &object_body_coords, true);
                fc->fb->SetData(&course);
            }
            printFlightData(&course);
            last_fix = steady_clock::now();
            had_fix = true;
        } else if (had_fix && steady_clock::now() - last_fix < seconds(2)) {
            //We had an object, attempt to follow according to last known position, but slower
            //Set PID update intervals
            m_pidx.SetInterval(update_rate);
            m_pidy.SetInterval(update_rate);
            
            m_pidx.Reset();
            m_pidy.Reset();
            
            //Determine trajectory to track the object (PID control)
            if (!m_observation_mode) {
                CalculateTrackingTrajectory(fc, &course, &object_body_coords, false);
                fc->fb->SetData(&course);
            }
            printFlightData(&course);
        } else {
            //Object lost; we should do a search pattern (TBA)
            SetCurrentState(fc, STATE_TRACKING_SEARCHING);
            
            //Reset the accumulated error in the PIDs
            m_pidx.Reset();
            m_pidy.Reset();
            //m_pid_yaw.Reset();

            fc->fb->Stop();
            if (had_fix) {
                fc->cam->SetArrow({0,0});
                Log(LOG_WARNING, "No object detected. Idling.");
                had_fix = false;
            }
        }
        
        sleep_for(sleep_time);
    }
    fc->cam->SetArrow({0,0});
    Log(LOG_INFO, "Object detection ended.");
    fc->fb->Stop();
}

//create the body coordinate vector for the object in the image
//in the absence of a distance sensor, we're assuming the object is on the ground, at the height we launched from.
void ObjectTracker::EstimatePositionFromImageCoords(GPSData *pos, FlightData *current, Point2D *object_location, Point3D *object_position){
    #warning "using 6m as ground level"
    double launchAlt = 6.0; //the James Oval is about 6m above sea level
    double heightAboveTarget = std::max(pos->fix.alt - launchAlt, 0.0);

    #warning "using 0.9m as height above target"
    heightAboveTarget = 0.9;    //hard-coded for lab test

    //Calibration factor Original: 2587.5 seemed too low from experimental testing
    //0.9m high 
    double L = 3687.5 * m_camwidth/2592.0;
    double gimbalVertical = 50; //gimbal angle that sets the camera to point straight down

    //Angles from image normal
    double theta = atan(object_location->y/L); //y angle
    double phi   = atan(object_location->x/L); //x angle
    
    //Tilt - in radians from vertical
    double gimbalTilt = DEG2RAD(gimbalVertical - current->gimbal);
    double objectAngleY = gimbalTilt + theta;
    double forwardPosition = tan(objectAngleY) * heightAboveTarget;
    double lateralPosition = (object_location->x / L) / cos(objectAngleY) * heightAboveTarget;

    object_position->y = forwardPosition;
    object_position->x = lateralPosition;
    object_position->z = heightAboveTarget;

    Log(LOG_INFO, "HAT: %.1f m, X: %.2fdeg, Y: %.2fdeg, FP: %.2fm, LP: %.2fm",
        heightAboveTarget, RAD2DEG(phi), RAD2DEG(objectAngleY), forwardPosition, lateralPosition);
}

void ObjectTracker::CalculateTrackingTrajectory(FlightController *fc, FlightData *course, Point3D *object_position, bool has_fix) {
    double trackx, tracky;
    
    if (!has_fix) {
        //Decay the speed
        trackx = course->rudder * 0.995;
        tracky = course->elevator * 0.995;
    } else {
        //Zero the course commands
        memset(course, 0, sizeof(FlightData));
        
        //double desiredSlope = 1;
        //double desiredForwardPosition = object_position->z/desiredSlope;
        double desiredForwardPosition = 1; //1m away
        m_pidy.SetSetPoint(desiredForwardPosition);

        //need to add a way of passing target bearings over here so we don't re-compute, can I use a Point2D?
        //This needs to be angle from the center, hence 90deg - angle
        double phi = M_PI/2 - atan2(object_position->y,object_position->x);

        m_pidx.SetProcessValue(-phi);   //rename this to m_pid_yaw or something, we can seriously use an X controller in chase now.
        m_pidy.SetProcessValue(object_position->y);
        Log(LOG_INFO, "PIDX: %.2f, PIDY: %.2f", -phi, -object_position->y);

        /*    //Now we can take full advantage of this omnidirectional platform.
        objectDistance = sqrt(object_position->x*object_position->x + object_position->y*object_position->y);
        distanceError = objectDistance - desiredForwardPosition;
        m_pid_yaw.SetProcessValue(-phi);
        m_pidx.SetProcessValue(-object_position->x * (distanceError/objectDistance) );  //the place we want to put the copter, relative to the copter.
        m_pidy.SetProcessValue(-object_position->y * (distanceError/objectDistance) );
        */

        trackx = m_pidx.Compute();
        tracky = m_pidy.Compute();
    }
    
    course->elevator = tracky;
    course->rudder = trackx;
    //Fix the angle for now...
    //course->gimbal = 50;
    fc->cam->SetArrow({100*trackx/TRACK_SPEED_LIMIT_X, -100*tracky/TRACK_SPEED_LIMIT_Y});

}

void printFlightData(FlightData* data) {
    printf("A: %03d E: %03d R: %03d G: %03d\r",
        data->aileron, data->elevator, data->rudder, data->gimbal);
    fflush(stdout);
}
