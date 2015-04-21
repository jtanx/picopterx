/**
 * @file object_tracker.cpp
 * @brief The object tracking code.
 */

#include "common.h"
#include "object_tracker.h"

using namespace picopter;
using std::chrono::steady_clock;
using std::chrono::milliseconds;
using std::chrono::microseconds;
using std::chrono::seconds;
using std::chrono::duration_cast;
using std::this_thread::sleep_for;

#define TOL_strafe (CAMERA_WIDTH/4)			//Pixels
#define KP_strafe 0.15
#define KI_strafe 0
#define TauI_strafe 4
#define TauD_strafe 0.0000003
#define STRAFE_WAIT 200 //Milliseconds

#define TOL_rotate (CAMERA_WIDTH/5)			//Pixels
#define KP_rotate 0.1

#define SPEED_LIMIT 40			//Percent
#define SPIN_SPEED 20			//Percent
#define RAISE_GIMBAL_PERIOD 3	//Seconds

#define GIMBAL_LIMIT 70		//degrees
#define GIMBAL_STEP 5		//degrees

void setCourse_followObject(FlightData*, ObjectLocation*, ObjectLocation*,double);
void setCourse_spin(FlightData*, bool);
void setCourse_faceObject(FlightData*, ObjectLocation*);
void setCourse_forwardsLowerGimbal(FlightData*, ObjectLocation*);

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
    
    ObjectLocation red_object, red_object_old;
    auto last_raised_gimbal_time = steady_clock::now();
    auto now = last_raised_gimbal_time;
    bool raise_gimbal = false;
                                    
    while (!fc->CheckForStop()) {
        std::vector<ObjectLocation> locations;
        FlightData course;
        
        fc->cam->GetObjectLocations(&locations);
        fc->fb->GetData(&course);
        if (locations.size() > 0) {                                             //We have an object
            double fps = fc->cam->GetFramerate();
            red_object = locations.front();
            SetCurrentState(fc, STATE_TRACKING_LOCKED);
            
            if(course.gimbal == 0) {						                    //And if gimbal not pitched
                setCourse_followObject(&course, &red_object, &red_object_old, fps);		//PI ON OBJECT
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
void setCourse_followObject(FlightData *course, ObjectLocation *red_object, ObjectLocation *red_object_old, double fps) {
    static PID pidx(KP_strafe, TauI_strafe, TauD_strafe, STRAFE_WAIT*1e-6);
    static PID pidy(KP_strafe, TauI_strafe, TauD_strafe, STRAFE_WAIT*1e-6);
    static bool initted = false;
    
    //Log(LOG_WARNING, "SHIT!");
    if (!initted) {
        pidx.SetInputLimits(-CAMERA_WIDTH/2, CAMERA_WIDTH/2);
        pidx.SetOutputLimits(-SPEED_LIMIT, SPEED_LIMIT);
        pidx.SetSetPoint(0);
        pidy.SetInputLimits(-CAMERA_HEIGHT/2, CAMERA_HEIGHT/2);
        pidy.SetOutputLimits(-SPEED_LIMIT, SPEED_LIMIT);
        pidy.SetSetPoint(0);
        initted = true;
    }
    
    if( (red_object->x * red_object->x) + (red_object->y * red_object->y) < TOL_strafe * TOL_strafe) {
        course->aileron = 0;
        course->elevator = 0;
        course->rudder = 0;
        course->gimbal = 0;
        pidx.Reset();
        pidy.Reset();
    } else {
        pidx.SetProcessValue(-red_object->x);//-red_object->x);
        pidy.SetProcessValue(-red_object->y);
        pidx.SetInterval(1.0/fps);
        pidy.SetInterval(1.0/fps);
        course->aileron = pidx.Compute();
        course->elevator = pidy.Compute();
        
        if(course->aileron * course->aileron + course->elevator * course->elevator > SPEED_LIMIT*SPEED_LIMIT) {
            double speed = sqrt(pow(course->aileron, 2)+pow(course->elevator, 2));
            course->aileron = (int) (course->aileron*SPEED_LIMIT/speed);
            course->elevator = (int) (course->elevator*SPEED_LIMIT/speed);
        }
        //printf("\n%d, %d, %.2f\n",red_object->x, red_object->y, 1.0/fps);
    }
    red_object_old->x = red_object->x;
    red_object_old->y = red_object->y;
    course->rudder = 0;
    course->gimbal = 0;
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

void setCourse_faceObject(FlightData *course, ObjectLocation *red_object) {
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

void setCourse_forwardsLowerGimbal(FlightData *course, ObjectLocation *red_object) {
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
