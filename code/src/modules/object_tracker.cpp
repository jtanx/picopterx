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

    TRACK_Kpw = opts->GetReal("TRACK_Kpw", 0.2);
    TRACK_Kpx = opts->GetReal("TRACK_Kpx", 0.2);
    TRACK_Kpy = opts->GetReal("TRACK_Kpy", 0.2);
    //double TRACK_Kpz = opts->GetReal("TRACK_Kpz", 50);
    TRACK_TauIw = opts->GetReal("TRACK_TauIw", 1.0);
    TRACK_TauIx = opts->GetReal("TRACK_TauIx", 1.0);
    TRACK_TauIy = opts->GetReal("TRACK_TauIy", 1.0);
    //double TRACK_TauIz = opts->GetReal("TRACK_TauIz", 5);
    TRACK_TauDw = opts->GetReal("TRACK_TauDw", 0.0005); //0.004 caused oscillation
    TRACK_TauDx = opts->GetReal("TRACK_TauDx", 0.0005);//0.008);//0.008);
    TRACK_TauDy = opts->GetReal("TRACK_TauDy", 0.0005);
    //double TRACK_TauDz = opts->GetReal("TRACK_TauDz", 0.004);//0.008);
    TRACK_SPEED_LIMIT_W = opts->GetInt("TRACK_SPEED_LIMIT_W", 20);
    TRACK_SPEED_LIMIT_X = opts->GetInt("TRACK_SPEED_LIMIT_X", 4);
    TRACK_SPEED_LIMIT_Y = opts->GetInt("TRACK_SPEED_LIMIT_Y", 4);
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

//TIME_TYPE ObjectTracker::timeSinceStart(){
//    return microseconds(steady_clock::now() - m_task_start);
//}
//TIME_TYPE ObjectTracker::timeSince(std::chrono::time_point<std::chrono::steady_clock> new_time){
//    return microseconds(new_time - m_task_start);
//}

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

    std::vector<Observation> visibles; //things we can currently see
    std::vector<Observations> knownThings; //things we know of


    Vec3D course{};
    EulerAngle gimbal;
    GPSData gps_position;
    IMUData imu_data;
    m_task_start = steady_clock::now();
    TIME_TYPE last_loop = steady_clock::now() - m_task_start;     //the current time for the samples being collected below
    TIME_TYPE loop_start = last_loop;
    //TIME_TYPE last_fix = sample_time - seconds(2);   //no fix (deprecate)

    bool had_fix = false;

    while (!fc->CheckForStop()) {
        last_loop = loop_start;
        loop_start = steady_clock::now() - m_task_start;
        TIME_TYPE loop_period = (loop_start - last_loop);


        double update_rate = 1.0 / fc->cam->GetFramerate();
        TIME_TYPE sleep_time = microseconds((int)(1000000*update_rate));    //how long to wait for the next frame (FIXME)


        fc->cam->GetDetectedObjects(&locations);
        fc->fb->GetGimbalPose(&gimbal);
        fc->gps->GetLatest(&gps_position);
        fc->imu->GetLatest(&imu_data);
        double lidar_range = (double)fc->lidar->GetLatest() / (100.0); //convert lidar range to metres

        
        //Now would be a good time to use the velocity and acceleration handler
        for(uint i=0; i<knownThings.size(); i++){
            knownThings.at(i).updateObject(loop_period);
        }//blur the location.
        

        //start making observation structures
        Observation lidarObservation = ObservationFromLidar(loop_start, &gps_position, &gimbal, &imu_data, lidar_range);

        //Did the camera see anything?
        if (locations.size() > 0) {
            detected_object = locations.front();

            //build observations for each
            std::vector<Observation> visibles; //things we can currently see
            visibles.reserve(locations.size()); //save multiple reallocations
            for(uint i=0; i<locations.size(); i++){
                visibles.push_back(ObservationFromImageCoords(loop_start, &gps_position, &gimbal, &imu_data, &detected_object));
            }

            //distinguish between many objects
            for(uint j=0; j<visibles.size(); j++){
                uint bestFit = 0;
                double fit=0;
                for(uint i=0; i<knownThings.size(); i++){
                    //Find the best fit
                    //compare characteristic data here.
                    double thisFit = knownThings.at(i).getSameProbability(visibles.at(j));
                    if(thisFit>fit){
                        bestFit = i;
                        fit=thisFit;
                    }
                }
                //How well does it fit?
                if(fit>OVERLAP_CONFIDENCE){
                    knownThings.at(bestFit).appendObservation(visibles.at(j));
                }else{
                    //doesn't fit anything well. Track it for later.
                    Observations newThing(visibles.at(j));
                    knownThings.push_back(newThing);
                }
            }

            //Now we have some things to track with known locations, without needing to see them.
            //Which one do we track?  The first thing it saw sounds right.
            
            //CalculateTrackingTrajectory(knownThings.front());

            SetCurrentState(fc, STATE_TRACKING_LOCKED);


            //Set PID update intervals
            m_pidx.SetInterval(update_rate);
            m_pidy.SetInterval(update_rate);
            
            EstimatePositionFromImageCoords(&gps_position, &gimbal, &imu_data, &detected_object, lidar_range);

            if (!m_observation_mode) {
                CalculateTrackingTrajectory(fc, &course, &detected_object, true);
                fc->fb->SetBodyVel(course);
            }
            LogSimple(LOG_DEBUG, "x: %.1f y: %.1f z: %.1f G: (%03.1f, %03.1f, %03.1f)\r",
                course.x, course.y, course.z, gimbal.roll, gimbal.pitch, gimbal.yaw);
            had_fix = true;


        //} else if (had_fix && steady_clock::now() - last_fix < seconds(2)) {
        } else if (had_fix && knownThings.front().lastObservation() < seconds(2)) {
            
            //We had an object, attempt to follow according to last known position, but slower
            //Set PID update intervals
            m_pidx.SetInterval(update_rate);
            m_pidy.SetInterval(update_rate);
            
            m_pidx.Reset();
            m_pidy.Reset();
            
            //Determine trajectory to track the object (PID control)
            if (!m_observation_mode) {
                CalculateTrackingTrajectory(fc, &course, &detected_object, false);
                fc->fb->SetBodyVel(course);
            }
            LogSimple(LOG_DEBUG, "x: %.1f y: %.1f z: %.1f G: (%03.1f, %03.1f, %03.1f)\r",
                course.x, course.y, course.z, gimbal.roll, gimbal.pitch, gimbal.yaw);
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
 * @param [in] imu_data The pitch and roll of the copter
 * @param [in] gimbal The current gimbal angle.
 * @param [in,out] object The detected object.
 */
void ObjectTracker::EstimatePositionFromImageCoords(GPSData *pos, EulerAngle *gimbal, IMUData *imu_data, ObjectInfo *object, double lidar_range) {
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
    //find the transformation matrix from camera frame to ground.
    cv::Matx33d Mbody = GimbalToBody(gimbal);
    cv::Matx33d Mstable = BodyToLevel(imu_data);
    //cv::Matx33d MYaw = LevelToGround(imu_data);

    //Comment this out to use dynamic altitude.
    //#warning "using 4m as height above target"
    //heightAboveTarget = 4;    //hard-coded for lab test
    
    double L = FOCAL_LENGTH * object->image_width;

    //3D vector of the target on the image plane, zero pose is straight down
    cv::Vec3d RelCam(object->position.y, object->position.x, L);
    //3D vector of the target, relative to the copter's stabilised frame
    cv::Vec3d RelBody = Mstable * Mbody * RelCam;

    //Angles from image normal
    double theta = atan2(RelCam[0],RelCam[2]); //Pitch angle of target in camera frame from vertical
    double phi   = atan2(RelCam[1],RelCam[2]); //Roll angle of target in camera frame from vertical
    
    bool useAlt = true;
    bool useLidar = UseLidar(object, lidar_range);

    if(useLidar){
        double k = lidar_range/norm(RelBody, cv::NORM_L2);
        RelBody *= k;
    }else if(useAlt){
        double k = heightAboveTarget / RelBody[2];
        RelBody *= k;
    }

    object->offset.x = RelBody[0];
    object->offset.y = RelBody[1];
    object->offset.z = RelBody[2];
    Log(LOG_DEBUG, "HAT: %.1f m, X: %.2fdeg, Y: %.2fdeg, FP: %.2fm, LP: %.2fm",
        heightAboveTarget, RAD2DEG(phi), RAD2DEG(theta), object->offset.y, object->offset.x);
}

cv::Matx33d ObjectTracker::GimbalToBody(EulerAngle *gimbal){
    return rotationMatrix(gimbal->roll,gimbal->pitch,gimbal->yaw);
}
cv::Matx33d ObjectTracker::BodyToGround(IMUData *imu_data){
    return rotationMatrix(imu_data->roll, imu_data->pitch, imu_data->yaw);
}
cv::Matx33d ObjectTracker::BodyToLevel(IMUData *imu_data){
    return rotationMatrix(imu_data->roll, imu_data->pitch, 0);
}
cv::Matx33d ObjectTracker::LevelToGround(IMUData *imu_data){
    return rotationMatrix(0,0,imu_data->yaw);
}
//

/**
 * determines whether or not the object in frame overlaps the lidar
 * @param [in] object The detected object.
 */
bool ObjectTracker::UseLidar(ObjectInfo *object, double lidar_range){
    //the centre of the lidar spot in the image frame
    double lidarCentreX = 0.1;
    double lidarCentreY = -0.2;
    double lidarRadius = 0.01;
    
    double x = (object->position.x / object->image_width) - lidarCentreX;
    double y = (object->position.y / object->image_height) - lidarCentreY;
    return (sqrt((x*x)+(y*y)) < lidarRadius);
}

/**
 * Create an observation structure from the blob of colour
 * 
 * @param [in] pos The current position of the copter.
 * @param [in] gimbal The current gimbal angle.
 * @param [in] imu_data The pitch and roll of the copter
 * @param [in] object The detected object.
 */
Observation ObjectTracker::ObservationFromImageCoords(TIME_TYPE sample_time, GPSData *pos, EulerAngle *gimbal, IMUData *imu_data, ObjectInfo *object){
    
    double L = FOCAL_LENGTH * object->image_width;     
    cv::Vec3d RelCam(object->position.y, object->position.x, L);    
    double theta = atan2(RelCam[0],RelCam[2]); //Pitch angle of target in camera frame from vertical    
    double phi   = atan2(RelCam[1],RelCam[2]); //Roll angle of target in camera frame from vertical     
    
    cv::Matx33d Mblob = rotationMatrix(phi,theta,0);   //the angle between the camera normal and the blob (deg)
    //find the transformation matrix from camera frame to ground.
    cv::Matx33d Mbody = GimbalToBody(gimbal);
    cv::Matx33d Mstable = BodyToLevel(imu_data);
    //cv::Matx33d MYaw = LevelToGround(imu_data);

    cv::Matx33d axes ( 
        0.5, 0, 0,
        0, 0.5, 0,
        0, 0, 0.0); //totally unknown depth
    cv::Matx31d vect (0,0,0);
    Distrib occular_ray = {axes,vect};

    occular_ray = stretchDistrib(occular_ray, 3);   //how big? we can't have conical distributions yet.

    occular_ray = rotateDistrib(occular_ray, Mblob);
    occular_ray = rotateDistrib(occular_ray, Mbody);
    occular_ray = rotateDistrib(occular_ray, Mstable);

    cv::Matx33d zeroAxes ( 
        0, 0, 0,
        0, 0, 0,
        0, 0, 0); //totally unknown
    Distrib zeroDistrib = {zeroAxes,vect};

    Observation imageObservation;
    imageObservation.location = occular_ray;
    imageObservation.velocity = zeroDistrib;
    imageObservation.acceleration = zeroDistrib;
    imageObservation.source = CAMERA_BLOB;
    imageObservation.camDetection = *object;

    return imageObservation;

}

/**
 * Create an observation structure from the lidar data
 * 
 * @param [in] pos The current position of the copter.
 * @param [in] gimbal The current gimbal angle.
 * @param [in] imu_data The pitch and roll of the copter
 * @param [in] lidar_range The length of the lidar beam.
 */
Observation ObjectTracker::ObservationFromLidar(TIME_TYPE sample_time, GPSData *pos, EulerAngle *gimbal, IMUData *imu_data, double lidar_range){
    cv::Matx33d MLidar = rotationMatrix(-6,-3,0);   //the angle between the camera and the lidar (deg)
    //find the transformation matrix from camera frame to ground.
    cv::Matx33d Mbody = GimbalToBody(gimbal);
    cv::Matx33d Mstable = BodyToLevel(imu_data);
    //cv::Matx33d MYaw = LevelToGround(imu_data);

    Distrib lidarspot = generatedistrib();
    
    double spotWidth = lidar_range*sin(DEG2RAD(3));
    lidarspot = stretchDistrib(lidarspot, spotWidth, spotWidth, 0.02);
    lidarspot = translateDistrib(lidarspot, 0,0,lidar_range);
    lidarspot = rotateDistrib(lidarspot, MLidar);
    lidarspot = rotateDistrib(lidarspot, Mbody);
    lidarspot = rotateDistrib(lidarspot, Mstable);

    cv::Matx33d zeroAxes ( 
        0, 0, 0,
        0, 0, 0,
        0, 0, 0); //totally unknown
    cv::Matx31d vect (0,0,0);
    Distrib zeroDistrib = {zeroAxes,vect};

    Observation lidarObservation;
    
    lidarObservation.location = lidarspot;
    lidarObservation.velocity = zeroDistrib;
    lidarObservation.acceleration = zeroDistrib;
    lidarObservation.source = LIDAR;
    //lidarObservation.camDetection = NULL;

    return lidarObservation;
}




/**
 * Calculates the trajectory (flight output) needed to track the object.
 * @param [in] fc The flight controller.
 * @param [in,out] course The current outputs of the copter.
 * @param [in] object The detected object.
 * @param [in] has_fix Indicates if we have a fix on the object or not.
 */
void ObjectTracker::CalculateTrackingTrajectory(FlightController *fc, Vec3D *course, ObjectInfo *object, bool has_fix) {
    double trackx, tracky, trackw;
    
    if (!has_fix) {
        //Decay the speed
        trackx = course->x * 0.995;
        tracky = course->y * 0.995;
        //trackw = course->rudder * 0.995;
    } else {
        //Zero the course commands
        memset(course, 0, sizeof(Vec3D));
        
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
        m_pidx.SetProcessValue((object->offset.x/objectDistance) * std::abs(distanceError));  //the place we want to put the copter, relative to the copter.
        m_pidy.SetProcessValue((object->offset.y/objectDistance) * distanceError);
        //m_pidz.SetProcessValue(  //throttle controller not used

        Log(LOG_DEBUG, "PIDX: %.2f, PIDY: %.2f", -phi, -object->offset.y);

        trackw = m_pidw.Compute();
        trackx = m_pidx.Compute();
        tracky = m_pidy.Compute();
        //trackz = m_pidz.Compute();
    }
    
    //TODO: course->z? This controls altitude.
    //To change yaw, use fb->SetYaw(bearing_or_offset, is_relative)
    course->x = trackx;
    course->y = tracky;
    //Fix the angle for now...
    //course->gimbal = 50;
    fc->cam->SetTrackingArrow({trackx/TRACK_SPEED_LIMIT_X,
        -tracky/TRACK_SPEED_LIMIT_Y, trackw/TRACK_SPEED_LIMIT_W});

}



/**
 * Calculates the desired location of the copter based on the estimated location of the object.
 * @param [in] fc The flight controller.
 * @param [in] object The detected object.
 * @param [in] has_fix Indicates if we have a fix on the object or not. (deprecate?)
 */
void CalculateVantagePoint(FlightController *fc, Observations object, bool has_fix){

    if (has_fix) {
        
        //go to a fixed radius from the object (or fly to the north, into the sun etc)
        //calculate a position to make a better observation from?

    }else{
        //go to a fixed radius from where we think the object is (or fly to the north, into the sun etc)
        //the aim here could be to see the object at all

    }

}
void CalculatePath(){


}



