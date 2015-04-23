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

#define TOL_strafe (CAMERA_WIDTH/5)			//Pixels
#define KP_strafe 0.16
#define KI_strafe 0
#define TauI_strafe 3.5
#define TauD_strafe 0.0000002
#define STRAFE_WAIT 200 //Milliseconds

#define TOL_rotate (CAMERA_WIDTH/5)			//Pixels
#define KP_rotate 0.1

#define SPEED_LIMIT 50			//Percent
#define SPIN_SPEED 20			//Percent
#define RAISE_GIMBAL_PERIOD 3	//Seconds

#define GIMBAL_LIMIT 70		//degrees
#define GIMBAL_STEP 5		//degrees

void setCourse_followObject(FlightData*, Point2D*,double);
void setCourse_spin(FlightData*, bool);
void setCourse_faceObject(FlightData*, Point2D*);
void setCourse_forwardsLowerGimbal(FlightData*, Point2D*);

void printFlightData(FlightData*);


ObjectTracker::ObjectTracker(Options *opts)
: m_pid1(0,0,0,0)
{
    
}

ObjectTracker::ObjectTracker() : ObjectTracker(NULL) {}

ObjectTracker::~ObjectTracker() {
    
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
    
    Point2D red_object = {0,0}, red_object_old = {0,0};
    auto last_raised_gimbal_time = steady_clock::now();
    auto now = last_raised_gimbal_time;
    bool raise_gimbal = false;
                                    
    while (!fc->CheckForStop()) {
        std::vector<Point2D> locations;
        FlightData course;
        
        fc->cam->GetDetectedObjects(&locations);
        fc->fb->GetData(&course);
        if (locations.size() > 0) {                                             //We have an object
            double fps = fc->cam->GetFramerate();
            red_object = locations.front();
            SetCurrentState(fc, STATE_TRACKING_LOCKED);
            
            if(course.gimbal == 0) {						                    //And if gimbal not pitched
                setCourse_followObject(&course, &red_object, fps);		//PI ON OBJECT
                fc->fb->SetData(&course);
                printFlightData(&course);
                sleep_for(microseconds((int)(1000000*1.0/fps)));
                //sleep_for(microseconds(200));//SMALL DELAY
            } else if(abs(red_object.x) < TOL_rotate) {			//Or if gimbal pitched, but in square on object
                setCourse_forwardsLowerGimbal(&course, &red_object);	//MOVE FORWARDS and Lower Gimbal
                fc->fb->SetData(&course);
                printFlightData(&course);
                sleep_for(microseconds(100));										//SMALL DELAY
            } else { //Or if gimbal pitched, but not directly facing object - need to rotate to face it.	
                setCourse_faceObject(&course, &red_object);				//PID YAW
                fc->fb->SetData(&course);
                printFlightData(&course);
                sleep_for(microseconds(100));
            }
            red_object_old = red_object;
        } else if(course.gimbal < GIMBAL_LIMIT) {	//No object found, but can pitch gimbal up to search a wider area - look for it
            SetCurrentState(fc, STATE_TRACKING_SEARCHING);
            now = steady_clock::now();
            raise_gimbal = (duration_cast<seconds>(now-last_raised_gimbal_time).count() > RAISE_GIMBAL_PERIOD);
            setCourse_spin(&course, raise_gimbal);
            fc->fb->Stop();
            //fc->fb->SetData(&course);
            printFlightData(&course);
            if(raise_gimbal) last_raised_gimbal_time = now;
            sleep_for(microseconds(200));												//SMALL DELAY
        } else { //No object found, nowhere else to look, give up.
            Log(LOG_WARNING, "No object detected. Idling.");
            fc->fb->Stop();
            //break;
        }
    }
    Log(LOG_INFO, "Object detection ended.");
    fc->fb->Stop();
}


//HELPER FUNCTIONS
void setCourse_followObject(FlightData *course, Point2D *red_object, double fps) {
    static PID pidx(KP_strafe, TauI_strafe, TauD_strafe, STRAFE_WAIT*1e-6);
    static PID pidy(KP_strafe, TauI_strafe, TauD_strafe, STRAFE_WAIT*1e-6);
    static bool initted = false;

    if (!initted) {
        pidx.SetInputLimits(-CAMERA_WIDTH/2, CAMERA_WIDTH/2);
        pidx.SetOutputLimits(-SPEED_LIMIT, SPEED_LIMIT);
        pidx.SetSetPoint(0);
        //We bias the vertical limit to be higher due to the pitch of the camera.
        pidy.SetInputLimits(-CAMERA_HEIGHT/2, CAMERA_HEIGHT/2);
        pidy.SetOutputLimits(-SPEED_LIMIT, SPEED_LIMIT);
        pidy.SetSetPoint(CAMERA_HEIGHT/8);
        initted = true;
    }
    
    //Zero the course commands
    memset(course, 0, sizeof(FlightData));
    if( red_object->magnitude() < TOL_strafe) {
        pidx.Reset();
        pidy.Reset();
    } else {
        pidx.SetProcessValue(-red_object->x);
        pidy.SetProcessValue(-red_object->y);
        pidx.SetInterval(1.0/fps);
        pidy.SetInterval(1.0/fps);
        course->rudder = pidx.Compute();
        course->elevator = pidy.Compute();
        
        double speed = std::sqrt(course->rudder * course->rudder + course->elevator * course->elevator);
        if(speed > SPEED_LIMIT) {
            course->rudder = (int) (course->rudder*SPEED_LIMIT/speed);
            course->elevator = (int) (course->elevator*SPEED_LIMIT/speed);
        }
    }
}

void setCourse_spin(FlightData *course, bool raise_gimbal) {
    course->aileron = 0;
    course->elevator = 0;
    course->rudder = SPIN_SPEED;
    if(raise_gimbal) {
        course->gimbal += GIMBAL_STEP;
    }
    if(course->gimbal > GIMBAL_LIMIT) {
        course->gimbal = GIMBAL_LIMIT;
    }
}

void setCourse_faceObject(FlightData *course, Point2D *red_object) {
    course->aileron = 0;
    course->elevator = 0;
    
    if(abs(red_object->x) < TOL_rotate) {
        course->rudder = 0;
    } else {
        course->rudder = (int)(KP_rotate * red_object->x);
        if(course->rudder > SPIN_SPEED) course->rudder = SPIN_SPEED;
        if(course->rudder < -SPIN_SPEED) course->rudder = -SPIN_SPEED;
    }
}

void setCourse_forwardsLowerGimbal(FlightData *course, Point2D *red_object) {
    course->aileron = 0;
    course->elevator = SPEED_LIMIT/2;
    course->rudder = 0;
    if(red_object->y < 0) {
        course->gimbal -= GIMBAL_STEP;
    }
    if(course->gimbal < 0) course->gimbal = 0;
}

void printFlightData(FlightData* data) {
    printf("A: %03d E: %03d R: %03d G: %03d\r",
        data->aileron, data->elevator, data->rudder, data->gimbal);
    fflush(stdout);
}
