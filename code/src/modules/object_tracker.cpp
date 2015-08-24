/**
 * @file object_tracker.cpp
 * @brief The object tracking code.
 */

#include "common.h"
#include "object_tracker.h"
#include <opencv2/opencv.hpp>

using namespace picopter;
using namespace picopter::navigation;
using std::chrono::steady_clock;
using std::chrono::milliseconds;
using std::chrono::microseconds;
using std::chrono::seconds;
using std::chrono::duration_cast;
using std::this_thread::sleep_for;

/**
 * Constructs a new (image based) object tracker.
 * @param [in] opts A pointer to options, if any (NULL for defaults).
 * @param [in] method The tracking method to use (deprecated).
 */
ObjectTracker::ObjectTracker(Options *opts, TrackMethod method)
: m_pidw(0,0,0,0.03)
, m_pidx(0,0,0,0.03)
, m_pidy(0,0,0,0.03)
, m_track_method{method}
, m_finished{false}
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

    TRACK_Kpw = opts->GetReal("TRACK_Kpw", 50);
    TRACK_Kpx = opts->GetReal("TRACK_Kpx", 22);
    TRACK_Kpy = opts->GetReal("TRACK_Kpy", 22);
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

/**
 * Constructor. Same as calling ObjectTracker(NULL, method)
 * @param [in] method A pointer to options, if any (NULL for defaults).
 */
ObjectTracker::ObjectTracker(TrackMethod method)
: ObjectTracker(NULL, method) {}

/**
 * Destructor.
 */
ObjectTracker::~ObjectTracker() {
    
}

/**
 * Returns the current tracking method.
 * @return The current tracking method.
 */
ObjectTracker::TrackMethod ObjectTracker::GetTrackMethod() {
    return m_track_method.load(std::memory_order_relaxed);
}

/**
 * Sets the current tracking method.
 * @param [in] method The tracking method to set to.
 */
void ObjectTracker::SetTrackMethod(TrackMethod method) {
    Log(LOG_INFO, "Track method: %d", method);
    m_track_method.store(method, std::memory_order_relaxed);
}

/**
 * Main computation loop of the object tracker.
 * @param [in] fc The flight controller that initiated this run.
 * @param [in] opts Closure (unused).
 */
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
    
    ObjectInfo detected_object = {};
    //Location of the target in body coordinates is in detected_object.offset
    //Point2D input_limits = {m_camwidth/2.0, m_camheight/2.0};

    std::vector<ObjectInfo> locations;
    FlightData course;
    GPSData gps_position;
    IMUData imu_data;
    auto last_fix = steady_clock::now() - seconds(2);
    bool had_fix = false;

    while (!fc->CheckForStop()) {
        double update_rate = 1.0 / fc->cam->GetFramerate();
        auto sleep_time = microseconds((int)(1000000*update_rate));

        fc->cam->GetDetectedObjects(&locations);
        fc->fb->GetData(&course);
        fc->gps->GetLatest(&gps_position);
        fc->imu->GetLatest(&imu_data);
        //Do we have an object?
        if (locations.size() > 0) {                                             
            detected_object = locations.front();
            SetCurrentState(fc, STATE_TRACKING_LOCKED);
            
            //Set PID update intervals
            m_pidx.SetInterval(update_rate);
            m_pidy.SetInterval(update_rate);
            
            EstimatePositionFromImageCoords(&gps_position, &course, &imu_data, &detected_object);

            if (!m_observation_mode) {
                CalculateTrackingTrajectory(fc, &course, &detected_object, true);
                fc->fb->SetData(&course);
            }
            LogSimple(LOG_DEBUG, "A: %03d E: %03d R: %03d G: %03d\r",
                course.aileron, course.elevator, course.rudder, course.gimbal);
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
                CalculateTrackingTrajectory(fc, &course, &detected_object, false);
                fc->fb->SetData(&course);
            }
            LogSimple(LOG_DEBUG, "A: %03d E: %03d R: %03d G: %03d\r",
                course.aileron, course.elevator, course.rudder, course.gimbal);
        } else {
            //Object lost; we should do a search pattern (TBA)
            SetCurrentState(fc, STATE_TRACKING_SEARCHING);
            
            //Reset the accumulated error in the PIDs
            m_pidx.Reset();
            m_pidy.Reset();
            m_pidw.Reset();

            fc->fb->Stop();
            if (had_fix) {
                fc->cam->SetTrackingArrow({0,0,0});
                Log(LOG_WARNING, "No object detected. Idling.");
                had_fix = false;
            }
        }
        
        sleep_for(sleep_time);
    }
    fc->cam->SetTrackingArrow({0,0,0});
    Log(LOG_INFO, "Object detection ended.");
    fc->fb->Stop();
    m_finished = true;
}

/**
 * Indicates whether or not the task has completed running.
 * @return true iff the task has completed running.
 */
bool ObjectTracker::Finished() {
    return m_finished;
}

/**
 * Create the body coordinate vector for the object in the image.
 * In the absence of a distance sensor, we're assuming the object is on the
 * ground, at the height we launched from.
 * 
 * @todo Review the use of the LIDAR-Lite for distance sensing.
 * @param [in] pos The current position of the copter.
 * @param [in] current The current outputs of the copter (for gimbal angle).
 * @param [in,out] object The detected object.
 */
void ObjectTracker::EstimatePositionFromImageCoords(GPSData *pos, FlightData *current, IMUData *imu_data, ObjectInfo *object) {
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

    //taking Euler chained rotations:
    double a;
    a = imu_data->roll;
    cv::Matx33d Rx(1,      0,       0,
                   0, cos(a), -sin(a),
                   0, sin(a),  cos(a));

    a = imu_data->pitch;
    cv::Matx33d Ry(cos(a), 0, -sin(a),
                        0, 1,       0,
                   sin(a), 0,  cos(a));
    a = imu_data->yaw;
    cv::Matx33d Rz(cos(a), -sin(a), 0,
                   sin(a),  cos(a), 0,
                        0,       0, 1);
    cv::Matx33d Rbody = Rx*Ry*Rz;


    /*
    lidar dist;
    imu_data->roll;
    imu_data->pitch;
    imu_data->yaw;
    object->location.lat;
    object->location.lon;
    object->location.alt;
    object->offset.x;
    object->offset.y;
    object->offset.z;
    */


    //Comment this out to use dynamic altitude.
    //#warning "using 4m as height above target"
    //heightAboveTarget = 4;    //hard-coded for lab test
    
    //Calibration factor Original: 2587.5 seemed too low from experimental testing
    //0.9m high 
    double L = 3687.5 * object->image_width/2592.0;
    double gimbalVertical = 50; //gimbal angle that sets the camera to point straight down


    //3D vector of the target on the image plane
    cv::Matx13d RelCam(object->position.x, object->position.y, L);
    //3D vector of the target in global rotations, relative to the copter
    cv::Matx13d RelBody = RelCam * Rbody;

    //Angles from image normal
    double theta = atan(object->position.y/L); //y angle
    double phi   = atan(object->position.x/L); //x angle
    
    //Tilt - in radians from vertical
    double gimbalTilt = DEG2RAD(gimbalVertical - current->gimbal);
    double objectAngleY = gimbalTilt + theta;
    double forwardPosition = tan(objectAngleY) * heightAboveTarget;

    double lateralPosition = ((object->position.x / L) / cos(objectAngleY)) * heightAboveTarget;

    object->offset.y = forwardPosition;
    object->offset.x = lateralPosition;
    object->offset.z = heightAboveTarget;



    Log(LOG_DEBUG, "HAT: %.1f m, X: %.2fdeg, Y: %.2fdeg, FP: %.2fm, LP: %.2fm",
        heightAboveTarget, RAD2DEG(phi), RAD2DEG(objectAngleY), forwardPosition, lateralPosition);
}


/**
 * Calculates the trajectory (flight output) needed to track the object.
 * @param [in] fc The flight controller.
 * @param [in,out] course The current outputs of the copter.
 * @param [in] object The detected object.
 * @param [in] has_fix Indicates if we have a fix on the object or not.
 */
void ObjectTracker::CalculateTrackingTrajectory(FlightController *fc, FlightData *course, ObjectInfo *object, bool has_fix) {
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
        double desiredForwardPosition = object->offset.z/desiredSlope;
        //double desiredForwardPosition = 1; //1m away
        //m_pidy.SetSetPoint(desiredForwardPosition);
        m_pidx.SetSetPoint(0);  //We can't use set-points properly because the PID loops need to cooperate between pitch and roll.
        m_pidy.SetSetPoint(0);  //maybe swap x,y out for PID on distance and object bearing?

        //We can't quite use the original image bearing here, computing against actual position is more appropriate.
        //This needs to be angle from the center, hence 90deg - angle
        double phi = M_PI/2 - atan2(object->offset.y,object->offset.x);

        /*
        m_pidx.SetProcessValue(-phi);   //rename this to m_pid_yaw or something, we can seriously use an X controller in chase now.
        m_pidy.SetProcessValue(object->offset.y);
        */
            //Now we can take full advantage of this omnidirectional platform.
        double objectDistance = std::sqrt(object->offset.x*object->offset.x + object->offset.y*object->offset.y);
        double distanceError = desiredForwardPosition-objectDistance;
        m_pidw.SetProcessValue(-phi);
        m_pidx.SetProcessValue(-(object->offset.x/objectDistance) * std::abs(distanceError));  //the place we want to put the copter, relative to the copter.
        m_pidy.SetProcessValue((object->offset.y/objectDistance) * distanceError);
        //m_pidz.SetProcessValue(  //throttle controller not used

        Log(LOG_DEBUG, "PIDX: %.2f, PIDY: %.2f", -phi, -object->offset.y);

        trackw = m_pidw.Compute();
        trackx = m_pidx.Compute();
        tracky = m_pidy.Compute();
        //trackz = m_pidz.Compute();
    }
    
    //course->rudder = trackw;
    course->aileron = trackx;
    course->elevator = tracky;
    //Fix the angle for now...
    //course->gimbal = 50;
    fc->cam->SetTrackingArrow({trackx/TRACK_SPEED_LIMIT_X,
        -tracky/TRACK_SPEED_LIMIT_Y, trackw/TRACK_SPEED_LIMIT_W});

}
