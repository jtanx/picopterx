/**
 * @file object_tracker.cpp
 * @brief The object tracking code.
 */

#include "common.h"
#include "object_tracker.h"

using namespace picopter;
using picopter::navigation::Point2D;
using std::chrono::steady_clock;
using std::chrono::milliseconds;
using std::chrono::microseconds;
using std::chrono::seconds;
using std::chrono::duration_cast;
using std::this_thread::sleep_for;

static void printFlightData(FlightData*);

ObjectTracker::ObjectTracker(Options *opts, TrackMethod method)
: m_pidx(0,0,0,0.03)
, m_pidy(0,0,0,0.03)
, m_track_method{method}
, SEARCH_GIMBAL_LIMIT(60)
{
    Options clear;
    if (!opts) {
        opts = &clear;
    }
    
    opts->SetFamily("OBJECT_TRACKER");
    TRACK_TOL = opts->GetInt("TRACK_TOL", CAMERA_WIDTH/8);
    TRACK_Kp = opts->GetReal("TRACK_Kp", 0.16);
    TRACK_TauI = opts->GetReal("TRACK_TauI", 3.5);
    TRACK_TauD = opts->GetReal("TRACK_TauD", 0.0000002);
    TRACK_SPEED_LIMIT = opts->GetInt("TRACK_SPEED_LIMIT", 50);
    TRACK_SETPOINT_X = opts->GetReal("TRACK_SETPOINT_X", 0);
    //We bias the vertical limit to be higher due to the pitch of the camera.
    TRACK_SETPOINT_Y = opts->GetReal("TRACK_SETPOINT_Y", -CAMERA_HEIGHT/15);
    
    m_pidx.SetTunings(TRACK_Kp, TRACK_TauI, TRACK_TauD);
    m_pidx.SetInputLimits(-CAMERA_WIDTH/2, CAMERA_WIDTH/2);
    m_pidx.SetOutputLimits(-TRACK_SPEED_LIMIT, TRACK_SPEED_LIMIT);
    m_pidx.SetSetPoint(TRACK_SETPOINT_X);
    
    m_pidy.SetTunings(TRACK_Kp*CAMERA_WIDTH/CAMERA_HEIGHT, TRACK_TauI, TRACK_TauD);
    m_pidy.SetInputLimits(-CAMERA_HEIGHT/2, CAMERA_HEIGHT/2);
    m_pidy.SetOutputLimits(-TRACK_SPEED_LIMIT, TRACK_SPEED_LIMIT);
    m_pidy.SetSetPoint(TRACK_SETPOINT_Y);
}

ObjectTracker::ObjectTracker(TrackMethod method) : ObjectTracker(NULL, method) {}

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
    }
    
    Log(LOG_INFO, "Object detection initiated; awaiting authorisation...");
    SetCurrentState(fc, STATE_AWAITING_AUTH);
    if (!fc->WaitForAuth()) {
        Log(LOG_INFO, "All stop acknowledged; quitting!");
        return;
    }
    
    Log(LOG_INFO, "Authorisation acknowledged. Finding object to track...");
    SetCurrentState(fc, STATE_TRACKING_SEARCHING);
    
    Point2D detected_object = {0,0};
    std::vector<Point2D> locations;
    FlightData course;
    auto last_fix = steady_clock::now() - seconds(2);
    bool had_fix = false;
    
    while (!fc->CheckForStop()) {
        double update_rate = 1.0 / fc->cam->GetFramerate();
        auto sleep_time = microseconds((int)(1000000*update_rate));
        
        fc->cam->GetDetectedObjects(&locations);
        fc->fb->GetData(&course);
        
        //Do we have an object?
        if (locations.size() > 0) {                                             
            detected_object = locations.front();
            SetCurrentState(fc, STATE_TRACKING_LOCKED);
            
            //Set PID update intervals
            m_pidx.SetInterval(update_rate);
            m_pidy.SetInterval(update_rate);
            
            //Determine trajectory to track the object (PID control)
            CalculateTrackingTrajectory(fc, &course, &detected_object, TRACK_SPEED_LIMIT);
            fc->fb->SetData(&course);
            printFlightData(&course);
            last_fix = steady_clock::now();
            had_fix = true;
        } else if (had_fix && steady_clock::now() - last_fix < seconds(2)) {
            //We had an object, attempt to follow according to last known position, but slower
            //Set PID update intervals
            m_pidx.SetInterval(update_rate);
            m_pidy.SetInterval(update_rate);
            
            //Determine trajectory to track the object (PID control)
            CalculateTrackingTrajectory(fc, &course, &detected_object, TRACK_SPEED_LIMIT/2);
            fc->fb->SetData(&course);
            printFlightData(&course);
        } else {
            //Object lost; we should do a search pattern (TBA)
            SetCurrentState(fc, STATE_TRACKING_SEARCHING);
            
            //Reset the accumulated error in the PIDs
            m_pidx.Reset();
            m_pidy.Reset();
            
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

void ObjectTracker::CalculateTrackingTrajectory(FlightController *fc, FlightData *course, navigation::Point2D *object_location, int speed_limit) {
    //Zero the course commands
    memset(course, 0, sizeof(FlightData));
    if(object_location->magnitude() < TRACK_TOL) {
        m_pidx.Reset();
        m_pidy.Reset();
        fc->cam->SetArrow({0,0});
    } else {
        TrackMethod method = GetTrackMethod();
        
        m_pidx.SetProcessValue(-object_location->x);
        m_pidy.SetProcessValue(-object_location->y);
        
        double trackx = m_pidx.Compute(), tracky = m_pidy.Compute();
        double speed = std::sqrt(trackx*trackx + tracky*tracky);
        if (speed > speed_limit) {
            trackx = trackx * speed_limit / speed;
            tracky = tracky * speed_limit / speed;
        }
        
        course->elevator = tracky;
        if (method == TRACK_ROTATE) {
            course->rudder =trackx;
        } else {
            course->aileron = trackx;
        }
        
        fc->cam->SetArrow({100*trackx/TRACK_SPEED_LIMIT, -100*tracky/TRACK_SPEED_LIMIT});
    }
}

void printFlightData(FlightData* data) {
    printf("A: %03d E: %03d R: %03d G: %03d\r",
        data->aileron, data->elevator, data->rudder, data->gimbal);
    fflush(stdout);
}