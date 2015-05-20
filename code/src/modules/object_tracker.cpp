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
, m_pidw(0,0,0,0.03)
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

    TRACK_Kpw = opts->GetReal("TRACK_Kpw", 60);
    TRACK_Kpx = opts->GetReal("TRACK_Kpx", 40);
    TRACK_Kpy = opts->GetReal("TRACK_Kpy", 30);
    //double TRACK_Kpz = opts->GetReal("TRACK_Kpz", 50);
    TRACK_TauIw = opts->GetReal("TRACK_TauIw", 5);
    TRACK_TauIx = opts->GetReal("TRACK_TauIx", 5);
    TRACK_TauIy = opts->GetReal("TRACK_TauIy", 5);
    //double TRACK_TauIz = opts->GetReal("TRACK_TauIz", 5);
    TRACK_TauDw = opts->GetReal("TRACK_TauDw", 0.000); //0.004 caused oscillation
    TRACK_TauDx = opts->GetReal("TRACK_TauDx", 0.000);//0.008);//0.008);
    TRACK_TauDy = opts->GetReal("TRACK_TauDy", 0.000);
    //double TRACK_TauDz = opts->GetReal("TRACK_TauDz", 0.004);//0.008);
    TRACK_SPEED_LIMIT_W = opts->GetInt("TRACK_SPEED_LIMIT_W", 40);
    TRACK_SPEED_LIMIT_X = opts->GetInt("TRACK_SPEED_LIMIT_X", 50);
    TRACK_SPEED_LIMIT_Y = opts->GetInt("TRACK_SPEED_LIMIT_Y", 50);
    //int TRACK_SPEED_LIMIT_Z = opts->GetInt("TRACK_SPEED_LIMIT_Z", 50);
    TRACK_SETPOINT_W = opts->GetReal("TRACK_SETPOINT_W", 0);
    TRACK_SETPOINT_X = opts->GetReal("TRACK_SETPOINT_X", 0);
    TRACK_SETPOINT_Y = opts->GetReal("TRACK_SETPOINT_Y", 0);
    //double TRACK_SETPOINT_Z = opts->GetReal("TRACK_SETPOINT_Z", 0);

    m_pidw.SetTunings(TRACK_Kpw, TRACK_TauIw, TRACK_TauDw);
    m_pidw.SetInputLimits(-M_PI/2, M_PI/2);
    m_pidw.SetOutputLimits(-TRACK_SPEED_LIMIT_W, TRACK_SPEED_LIMIT_W);
    m_pidw.SetSetPoint(TRACK_SETPOINT_W);
    
    m_pidx.SetTunings(TRACK_Kpx, TRACK_TauIx, TRACK_TauDx);
    m_pidx.SetInputLimits(-8, 8);
    m_pidx.SetOutputLimits(-TRACK_SPEED_LIMIT_X, TRACK_SPEED_LIMIT_X);
    m_pidx.SetSetPoint(TRACK_SETPOINT_X);
    
    m_pidy.SetTunings(TRACK_Kpy, TRACK_TauIy, TRACK_TauDy);
    m_pidy.SetInputLimits(-8,8);
    m_pidy.SetOutputLimits(-TRACK_SPEED_LIMIT_Y, TRACK_SPEED_LIMIT_Y);
    m_pidy.SetSetPoint(TRACK_SETPOINT_Y);
    //throttle not used
    //m_pidz.SetTunings(-TRACK_Kpz, TRACK_TauIz, TRACK_TauDz);
    //m_pidz.SetInputLimits(-8,8);
    //m_pidz.SetOutputLimits(-TRACK_SPEED_LIMIT_Z, TRACK_SPEED_LIMIT_Z);
    //m_pidz.SetSetPoint(TRACK_SETPOINT_Z);
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
    //Point2D input_limits = {m_camwidth/2.0, m_camheight/2.0};
    Point3D object_body_coords = {0,0,0};  //location of the target in body coordinates

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
            
            EstimatePositionFromImageCoords(&gps_position, &course, &detected_object, &object_body_coords);

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
            m_pidw.Reset();

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
    double heightAboveTarget = std::max(pos->fix.alt - pos->fix.groundalt, 0.0);
    
    if (std::isnan(heightAboveTarget)) {
        //Todo: ??
        static bool has_warned = false;
        if (!has_warned) {
            Log(LOG_DEBUG, "No usable altitude; falling back to 4m!");
            has_warned = true;
        }
        heightAboveTarget = 4;
    }
    
    //Comment this out to use dynamic altitude.
    #warning "using 4m as height above target"
    heightAboveTarget = 4;    //hard-coded for lab test
    
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

    double lateralPosition = ((object_location->x / L) / cos(objectAngleY)) * heightAboveTarget;

    object_position->y = forwardPosition;
    object_position->x = lateralPosition;
    object_position->z = heightAboveTarget;

    Log(LOG_DEBUG, "HAT: %.1f m, X: %.2fdeg, Y: %.2fdeg, FP: %.2fm, LP: %.2fm",
        heightAboveTarget, RAD2DEG(phi), RAD2DEG(objectAngleY), forwardPosition, lateralPosition);
}

void ObjectTracker::CalculateTrackingTrajectory(FlightController *fc, FlightData *course, Point3D *object_position, bool has_fix) {
    double trackx, tracky, trackw;
    
    if (!has_fix) {
        //Decay the speed
        trackx = course->aileron * 0.995;
        tracky = course->elevator * 0.995;
        trackw = course->rudder * 0.995;
    } else {
        //Zero the course commands
        memset(course, 0, sizeof(FlightData));
        
        double desiredSlope = 0.8;    //Maintain about the same camera angle 
        double desiredForwardPosition = object_position->z/desiredSlope;
        //double desiredForwardPosition = 1; //1m away
        //m_pidy.SetSetPoint(desiredForwardPosition);
        m_pidx.SetSetPoint(0);  //We can't use set-points properly because the PID loops need to cooperate between pitch and roll.
        m_pidy.SetSetPoint(0);  //maybe swap x,y out for PID on distance and object bearing?

        //We can't quite use the original image bearing here, computing against actual position is more appropriate.
        //This needs to be angle from the center, hence 90deg - angle
        double phi = M_PI/2 - atan2(object_position->y,object_position->x);

        /*
        m_pidx.SetProcessValue(-phi);   //rename this to m_pid_yaw or something, we can seriously use an X controller in chase now.
        m_pidy.SetProcessValue(object_position->y);
        */
            //Now we can take full advantage of this omnidirectional platform.
        double objectDistance = std::sqrt(object_position->x*object_position->x + object_position->y*object_position->y);
        double distanceError = desiredForwardPosition-objectDistance;
        m_pidw.SetProcessValue(-phi);
        m_pidx.SetProcessValue(-(object_position->x/objectDistance) * std::abs(distanceError));  //the place we want to put the copter, relative to the copter.
        m_pidy.SetProcessValue((object_position->y/objectDistance) * distanceError);
        //m_pidz.SetProcessValue(  //throttle controller not used

        Log(LOG_DEBUG, "PIDX: %.2f, PIDY: %.2f", -phi, -object_position->y);

        trackw = m_pidw.Compute();
        trackx = m_pidx.Compute();
        tracky = m_pidy.Compute();
        //trackz = m_pidz.Compute();
    }
    
    course->rudder = trackw;
    course->aileron = trackx;
    course->elevator = tracky;
    //Fix the angle for now...
    //course->gimbal = 50;
    fc->cam->SetArrow({100*trackw/TRACK_SPEED_LIMIT_W, -100*tracky/TRACK_SPEED_LIMIT_Y});

}

void printFlightData(FlightData* data) {
    printf("A: %03d E: %03d R: %03d G: %03d\r",
        data->aileron, data->elevator, data->rudder, data->gimbal);
    fflush(stdout);
}
