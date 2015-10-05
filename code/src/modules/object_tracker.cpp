/**
 * @file object_tracker.cpp
 * @brief The object tracking code.
 */

#include "common.h"
#include "object_tracker.h"

using namespace picopter;
using namespace picopter::navigation;
using namespace cv;

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
    m_demo_mode = opts->GetBool("DEMO_MODE", false);
    //The gain has been configured for a 320x240 image, so scale accordingly.
    opts->SetFamily("OBJECT_TRACKER");

    TRACK_Kpw = opts->GetReal("TRACK_Kpw", 0.5);
    TRACK_Kpx = opts->GetReal("TRACK_Kpx", 0.5);
    TRACK_Kpy = opts->GetReal("TRACK_Kpy", 0.5);
    //double TRACK_Kpz = opts->GetReal("TRACK_Kpz", 50);
    TRACK_TauIw = opts->GetReal("TRACK_TauIw", 0);
    TRACK_TauIx = opts->GetReal("TRACK_TauIx", 0);
    TRACK_TauIy = opts->GetReal("TRACK_TauIy", 0);
    //double TRACK_TauIz = opts->GetReal("TRACK_TauIz", 5);
    TRACK_TauDw = opts->GetReal("TRACK_TauDw", 0.000); //0.004 caused oscillation
    TRACK_TauDx = opts->GetReal("TRACK_TauDx", 0.000);//0.008);//0.008);
    TRACK_TauDy = opts->GetReal("TRACK_TauDy", 0.000);
    //double TRACK_TauDz = opts->GetReal("TRACK_TauDz", 0.004);//0.008);
    TRACK_SPEED_LIMIT_W = opts->GetInt("TRACK_SPEED_LIMIT_W", 20);
    TRACK_SPEED_LIMIT_X = opts->GetInt("TRACK_SPEED_LIMIT_X", 4);
    TRACK_SPEED_LIMIT_Y = opts->GetInt("TRACK_SPEED_LIMIT_Y", 4);
    //int TRACK_SPEED_LIMIT_Z = opts->GetInt("TRACK_SPEED_LIMIT_Z", 50);
    TRACK_SETPOINT_W = opts->GetReal("TRACK_SETPOINT_W", 0);
    TRACK_SETPOINT_X = opts->GetReal("TRACK_SETPOINT_X", 0);
    TRACK_SETPOINT_Y = opts->GetReal("TRACK_SETPOINT_Y", 0);
    //double TRACK_SETPOINT_Z = opts->GetReal("TRACK_SETPOINT_Z", 0);
    
    TRACK_SETPOINT_X = opts->GetReal("TRACK_SETPOINT_X", 0);
    TRACK_SETPOINT_Y = opts->GetReal("TRACK_SETPOINT_Y", 0);

    desiredSlope = opts->GetReal("TRACK_SLOPE", 0.8);
    observation_image_rows = opts->GetReal("OBS_IMAGE_ROWS",240);
    observation_image_cols = opts->GetReal("OBS_IMAGE_COLS",320);
    print_observation_map = opts->GetBool("PRINT_OBS_MAP",false);
    

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
    
    //Get the launch position.
    if (!fc->fb->GetHomePosition(&launch_point)) {
        Log(LOG_WARNING, "I don't know the launch position! Bailing!!!");
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
    fc->gps->GetLatest(&gps_position);
    //launch_point = Coord3D{gps_position.fix.lat, gps_position.fix.lon, gps_position.fix.alt};

    Mat observation_map(observation_image_rows, observation_image_cols, CV_8UC4);

    EulerAngle pose;
    pose.roll = 0;
    pose.pitch = 45;
    pose.yaw = 0;
    fc->fb->ConfigureGimbal();

    while (!fc->CheckForStop()) {
        fc->fb->SetGimbalPose(pose);

        last_loop = loop_start;
        loop_start = steady_clock::now() - m_task_start;
        TIME_TYPE loop_period = (loop_start - last_loop);

        //clear the printable map
        observation_map = Mat::zeros(observation_map.rows, observation_map.cols, CV_8UC4);

        double update_rate = 1.0 / fc->cam->GetFramerate();
        TIME_TYPE sleep_time = microseconds((int)(1000000*update_rate));    //how long to wait for the next frame (FIXME)


        fc->cam->GetDetectedObjects(&locations);
        fc->fb->GetGimbalPose(&gimbal);
        fc->gps->GetLatest(&gps_position);
        fc->imu->GetLatest(&imu_data);

        //LogSimple(LOG_DEBUG, "Copter is at: lat: %.4f, lon: %.4f, alt %.4f", gps_position.fix.lat, gps_position.fix.lon, gps_position.fix.alt);
        //LogSimple(LOG_DEBUG, "IMU is at: roll: %.4f, pitch: %.4f, yaw %.4f", imu_data.roll, imu_data.pitch, imu_data.yaw);
        //LogSimple(LOG_DEBUG, "Gimbal is at: roll: %.4f, pitch: %.4f, yaw %.4f", gimbal.roll, gimbal.pitch, gimbal.yaw);
        
        //LogSimple(LOG_DEBUG, "loop_start %u", duration_cast<microseconds>(loop_start).count());



        double lidar_range = (double)fc->lidar->GetLatest() / (100.0); //convert lidar range to metres

        
        //Now would be a good time to use the velocity and acceleration handler
        for(uint i=0; i<knownThings.size(); i++){
            //LogSimple(LOG_DEBUG, "object %d observed %uuS ago", i, duration_cast<microseconds>(loop_start - knownThings.at(i).lastObservation()).count());
            //LogSimple(LOG_DEBUG, "object %d observed at %u", i, duration_cast<microseconds>(knownThings.at(i).lastObservation()).count());
            //Vec3d V = knownThings.at(i).getLocation().vect;
            //Matx33d A = knownThings.at(i).getLocation().axes;
            //LogSimple(LOG_DEBUG, " at ground coords [%.4f,%.4f,%.4f]", V(0),V(1),V(2));
            //LogSimple(LOG_DEBUG, " with covariance\n\t\t\t\t[%.4f,%.4f,%.4f\n\t\t\t\t %.4f,%.4f,%.4f\n\t\t\t\t %.4f,%.4f,%.4f]", A(0,0),A(1,0),A(2,0),A(0,1),A(1,1),A(2,1),A(0,2),A(1,2),A(2,2));

            if( (loop_start - knownThings.at(i).lastObservation()) < seconds(10) ){

                knownThings.at(i).updateObject(loop_period);

                if(print_observation_map){  //plot the objects on the map
                    Distrib tmp = knownThings[i].getLocation();
                    rasterDistrib(&observation_map, &tmp, Vec4b(0,0,UCHAR_MAX,UCHAR_MAX), 1.0);  //red
                }

            }else{
                LogSimple(LOG_DEBUG,"Removing Lost Object %d", i);
                if((i==0) && (knownThings.size()>1)){
                    Log(LOG_WARNING, "Switching targets");
                }
                knownThings.erase(knownThings.begin()+i);
                i--;
            }
        }//blur the location.
        //LogSimple(LOG_DEBUG,"Tracking %d Objects", knownThings.size());

        //start making observation structures
        Observation lidarObservation = ObservationFromLidar(loop_start, &gps_position, &gimbal, &imu_data, lidar_range);
        rasterDistrib(&observation_map, &lidarObservation.location, Vec4b(UCHAR_MAX,0,0,UCHAR_MAX), 1.0);  //blue


        //Did the camera see anything?
        if (locations.size() > 0) {
            detected_object = locations.front();

            //build observations for each
            std::vector<Observation> visibles; //things we can currently see
            visibles.reserve(locations.size()); //save multiple reallocations
            for(uint i=0; i<locations.size(); i++){
                visibles.push_back(ObservationFromImageCoords(loop_start, &gps_position, &gimbal, &imu_data, &detected_object));
                
                //Vec3d V = visibles.back().location.vect;
                //Matx33d A = visibles.back().location.axes;
                //LogSimple(LOG_DEBUG, " detection:");
                //LogSimple(LOG_DEBUG, " at ground coords [%.4f,%.4f,%.4f]", V(0),V(1),V(2));
                //LogSimple(LOG_DEBUG, " with covariance\n\t\t\t\t[%.4f,%.4f,%.4f\n\t\t\t\t %.4f,%.4f,%.4f\n\t\t\t\t %.4f,%.4f,%.4f]", A(0,0),A(1,0),A(2,0),A(0,1),A(1,1),A(2,1),A(0,2),A(1,2),A(2,2));

            }

            if(print_observation_map){
                //print the observations and objects
                std::string filename = "tracker"+std::to_string(observation_map_count++)+".png";
                storeDistrib(&observation_map, filename);
            }


            //distinguish between many objects
            //matchObsToObj(visibles, knownThings);
            //ChooseObsToObj(visibles, knownThings);
            NoObjectMemory(visibles, knownThings);
        }

        //Now we have some things to track with known locations, without needing to see them.

        if(print_observation_map){
            for(uint j=0; j<visibles.size(); j++){
                rasterDistrib(&observation_map, &(visibles[j].location), Vec4b(0,UCHAR_MAX,0,UCHAR_MAX), 1.0);
            }
        }
         
        if(knownThings.size() > 0){   
            Coord3D object_gps_location = GPSFromGround(knownThings.front().getLocation().vect);
            
            //LogSimple(LOG_DEBUG, "last observation at: %u", duration_cast<microseconds>(loop_start).count());
            //LogSimple(LOG_DEBUG, "Object %d observed %uuS ago", 0, duration_cast<microseconds>(loop_start - knownThings.at(0).lastObservation()).count());
            LogSimple(LOG_DEBUG, "Object is at: lat: %.8f, lon: %.8f, alt %.4f", object_gps_location.lat, object_gps_location.lon, object_gps_location.alt);
        }


//        if(true){

        //Which one do we track?  The first thing it saw sounds right.
        if(knownThings.size() <= 0){
            LogSimple(LOG_WARNING, "No object detected. Waiting.");

        //had_fix && knownThings.front().lastObservation() < seconds(2)
        } else if((loop_start - knownThings.front().lastObservation()) <= milliseconds(500)){    //Did we see the thing this loop?

            SetCurrentState(fc, STATE_TRACKING_LOCKED);

            //Set PID update intervals
            m_pidw.SetInterval(update_rate);
            m_pidx.SetInterval(update_rate);    //FIXME!
            m_pidy.SetInterval(update_rate);    //we have a loop timer now.
  
            //Coord3D testpoint = launch_point;
            //testpoint.lat += 0.0001;
            //testpoint.lon += 0.0001;
            //testpoint.alt = 0;
            //Coord3D poi_point = launch_point;
            //poi_point.lat -= 0.0005;
            //poi_point.lon -= 0.01;
            //poi_point.alt = 0;
            //Observations testThing(ObservationFromRemote(poi_point));
            Coord3D vantage = CalculateVantagePoint(&gps_position, &knownThings.front(), true);
            Coord3D poi_point = GPSFromGround(knownThings.front().getLocation().vect);
            //Coord3D vantage = CalculateVantagePoint(&gps_position, &testThing, true);
            if (!m_observation_mode) {
                PathWaypoint(fc, &gps_position, &imu_data, vantage, poi_point);
                //CalculatePath(fc, &gps_position, &imu_data, vantage, poi_point, &course);
                //CalculatePath(fc, &gps_position, &imu_data, testpoint, poi_point, &course);
                //fc->fb->SetBodyVel(course);
            }
            LogSimple(LOG_DEBUG, "x: %.1f y: %.1f z: %.1f G: (%03.1f, %03.1f, %03.1f)\r",
                course.x, course.y, course.z, gimbal.roll, gimbal.pitch, gimbal.yaw);
            had_fix = true;



    //There's probably a use for this code segment other than holding the phone for a while, but I can't think of one.
        } else if (had_fix && (loop_start - knownThings.front().lastObservation()) < seconds(2)) {
            
            m_pidw.SetInterval(update_rate);
            m_pidx.SetInterval(update_rate);
            m_pidy.SetInterval(update_rate);

            Coord3D vantage = CalculateVantagePoint(&gps_position, &knownThings.front(), true);
            Coord3D poi_point = GPSFromGround(knownThings.front().getLocation().vect);
            if (!m_observation_mode) {
                PathWaypoint(fc, &gps_position, &imu_data, vantage, poi_point);
                //CalculatePath(fc, &gps_position, &imu_data, vantage, poi_point, &course);
                //fc->fb->SetBodyVel(course);
            }
            LogSimple(LOG_DEBUG, "x: %.1f y: %.1f z: %.1f G: (%03.1f, %03.1f, %03.1f)\r",
                course.x, course.y, course.z, gimbal.roll, gimbal.pitch, gimbal.yaw);


        } else {
            //Object lost; we should do a search pattern (TBA)
            SetCurrentState(fc, STATE_TRACKING_SEARCHING);

            //knownThings.clear();
            
            //Reset the accumulated error in the PIDs
            m_pidx.Reset();
            m_pidy.Reset();
            m_pidw.Reset();

            fc->fb->Stop();
            if (had_fix) {
                fc->cam->SetTrackingArrow({0,0,0});
                Log(LOG_WARNING, "Object Lost. Idling.");
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


Matx33d ObjectTracker::GimbalToBody(EulerAngle *gimbal){
    return rotationMatrix(gimbal->roll,gimbal->pitch,gimbal->yaw);
}
Matx33d ObjectTracker::BodyToGround(IMUData *imu_data){
    return rotationMatrix(imu_data->roll, imu_data->pitch, imu_data->yaw);
}
Matx33d ObjectTracker::BodyToLevel(IMUData *imu_data){
    return rotationMatrix(imu_data->roll, imu_data->pitch, 0);
}
Matx33d ObjectTracker::LevelToGround(IMUData *imu_data){
    return rotationMatrix(0,0,imu_data->yaw);
}

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
    Vec3d RelCam(object->position.y, object->position.x, L);    
    double phi   = RAD2DEG(-atan2(RelCam[1],RelCam[2])); //Roll angle of target in camera frame from vertical
    double theta = RAD2DEG( atan2(RelCam[0],RelCam[2])); //Pitch angle of target in camera frame from vertical
    
    Matx33d Mblob = rotationMatrix(phi,theta,0);   //the angle between the camera normal and the blob (deg)
    //find the transformation matrix from camera frame to ground.
    Matx33d Mbody = GimbalToBody(gimbal);

    Matx33d MGnd;
    if(m_demo_mode){
        MGnd = LevelToGround(imu_data);    //Disable the pitch and roll in simulation
    }else{
        MGnd = BodyToGround(imu_data);      //Enable them during live tests
    }
    //Matx33d Mstable = BodyToLevel(imu_data);
    //Matx33d MYaw = LevelToGround(imu_data);
    //Matx33d MGnd = BodyToGround(imu_data);

    Matx33d axes (
        0.5, 0,   0,
        0,   0.5, 0,
        0,   0,   0.0); //totally unknown depth
    Vec3d vect (0,0,0);
    Distrib occular_ray = {axes,vect};

    occular_ray = stretchDistrib(occular_ray, 10);   //how big? we can't have conical distributions yet.

    occular_ray = rotateDistrib(occular_ray, Mblob);
    occular_ray = rotateDistrib(occular_ray, Mbody);
    occular_ray = rotateDistrib(occular_ray, MGnd);

    Coord3D copterloc = {pos->fix.lat, pos->fix.lon, pos->fix.alt};
    occular_ray = translateDistrib(occular_ray, GroundFromGPS(copterloc));

    Matx33d zeroAxes ( 
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
    imageObservation.sample_time = sample_time;


    //Distrib estLocation = combineDistribs(occular_ray, AssumptionGroundLevel().location);
    //Vec3d C = GroundFromGPS(copterloc);
    //Vec3d B = estLocation.vect;
    //Vec3d V = B-C;
    //Matx33d &A = estLocation.axes;
    //LogSimple(LOG_DEBUG, " detection:");
    //LogSimple(LOG_DEBUG, " at copter [%.4f,%.4f,%.4f]", B(0),B(1),B(2));
    //LogSimple(LOG_DEBUG, " at object [%.4f,%.4f,%.4f]", C(0),C(1),C(2));
    //LogSimple(LOG_DEBUG, " at relative [%.4f,%.4f,%.4f]", V(0),V(1),V(2));
    //LogSimple(LOG_DEBUG, " with covariance\n\t\t\t\t[%.4f,%.4f,%.4f\n\t\t\t\t %.4f,%.4f,%.4f\n\t\t\t\t %.4f,%.4f,%.4f]", A(0,0),A(1,0),A(2,0),A(0,1),A(1,1),A(2,1),A(0,2),A(1,2),A(2,2));


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
    Matx33d MLidar = rotationMatrix(-6,-3,0);   //the angle between the camera and the lidar (deg)
    //find the transformation matrix from camera frame to ground.
    Matx33d Mbody = GimbalToBody(gimbal);
    
    Matx33d MGnd;
    if(m_demo_mode){
        MGnd = LevelToGround(imu_data);    //Disable the pitch and roll in simulation
    }else{
        MGnd = BodyToGround(imu_data);      //Enable them during live tests
    }
    //Matx33d Mstable = BodyToLevel(imu_data);
    //Matx33d MYaw = LevelToGround(imu_data);
    //Matx33d MGnd = BodyToGround(imu_data);

    Distrib lidarspot = generatedistrib();
    
    double spotWidth = lidar_range*sin(DEG2RAD(3));
    lidarspot = stretchDistrib(lidarspot, spotWidth, spotWidth, 0.02);
    lidarspot = translateDistrib(lidarspot, 0,0,lidar_range);
    lidarspot = rotateDistrib(lidarspot, MLidar);
    lidarspot = rotateDistrib(lidarspot, Mbody);
    lidarspot = rotateDistrib(lidarspot, MGnd);
    Coord3D copterloc = {pos->fix.lat, pos->fix.lon, pos->fix.alt};
    lidarspot = translateDistrib(lidarspot, GroundFromGPS( copterloc ));

    Matx33d zeroAxes (
        0, 0, 0,
        0, 0, 0,
        0, 0, 0); //totally unknown
    Vec3d vect (0,0,0);
    Distrib zeroDistrib = {zeroAxes,vect};

    Observation lidarObservation;
    
    lidarObservation.location = lidarspot;
    lidarObservation.velocity = zeroDistrib;
    lidarObservation.acceleration = zeroDistrib;
    lidarObservation.source = LIDAR;
    //lidarObservation.camDetection = NULL;

    return lidarObservation;
}

Observation ObjectTracker::ObservationFromRemote(Coord3D &pos){
    Observation retval = AssumptionGroundLevel();
    retval.location = generatedistrib();
    retval.location.vect = GroundFromGPS(pos);
    return retval;
}

Observation ObjectTracker::AssumptionGroundLevel(){
    Matx33d zeroAxes (
        0, 0, 0,
        0, 0, 0,
        0, 0, 0); //totally unknown
    Vec3d vect (0,0,0);

    Matx33d flatAxes (
        0, 0, 0,
        0, 0, 0,
        0, 0, 0.5);   //1m variance on ground level

    Matx33d slowAxes (
        0.5, 0, 0,
        0, 0.5, 0,
        0, 0, 0.5);   //1m variance per second


    Coord3D launch_ground = launch_point;
    launch_ground.alt = 0;    //is this sea level?
    Vec3d groundLevel = GroundFromGPS(launch_ground);

    Distrib flatDistrib = {flatAxes, groundLevel};
    //Distrib zeroDistrib = {zeroAxes, vect};
    Distrib slowDistrib = {slowAxes, vect};
    Observation Assumption;

    Assumption.location = flatDistrib;
    Assumption.velocity = slowDistrib;
    Assumption.acceleration = slowDistrib;
    Assumption.source = ASSUMPTION;
    Assumption.sample_time = m_task_start - m_task_start;
    return Assumption;

}

/**
 * Try to match observations to objects.
 * @param [in] visibles The observations, 
 * @param [in] knownThings The objects.
 */
bool ObjectTracker::matchObsToObj(std::vector<Observation> &visibles, std::vector<Observations> &knownThings){
    Observation theGround = AssumptionGroundLevel();

    uint n_obj = knownThings.size();
    uint n_obs = visibles.size();
    uint m_obs = n_obs;
    LogSimple(LOG_DEBUG,"Sorting out %d objects and %d observations", n_obj, n_obs);
//************ Warning, Coding Tired.
//              Sorry about the segfaults and memory leaks
//              I'm using integers were I should have pointers.
    double *fits = new double [n_obj * n_obs];
    if(fits==NULL) return false;
    uint *obj_index = new uint [n_obj];
    if(obj_index==NULL) {
        delete[] fits;
        return false;
    }
    uint *obs_index = new uint [n_obs];
    if(obs_index==NULL){
        delete[] fits;
        delete[] obj_index;
        return false;
    }

    uint best_obj = 0;  //the index of the best fitting object
    uint best_obs = 0;  //the index of the best fitting observation
    double fit;
    
    //compute all the fit factors
    for(uint j=0; j<n_obs; j++){
        obs_index[j] = j;
    }
    for(uint i=0; i<n_obj; i++){
        obj_index[i] = i;
        for(uint j=0; j<visibles.size(); j++){
            fits[i*m_obs + j] = knownThings.at(i).getSameProbability(visibles.at(j));
        }
    }

    while((n_obj>0) && (n_obs>0)){
        //LogSimple(LOG_DEBUG,"Remainder: %d objects and %d observations", n_obj, n_obs);
        fit = 0;
        for(uint i=0; i<n_obj; i++){
            for(uint j=0; j<n_obs; j++){
                if( fit < fits[obj_index[i]*m_obs + obs_index[j]]){
                    fit = fits[obj_index[i]*m_obs + obs_index[j]];     //take only the best fit
                    best_obj = obj_index[i];
                    best_obs = obs_index[j];
                }
            }
        }
        if(fit>OVERLAP_CONFIDENCE){
            LogSimple(LOG_DEBUG,"Matching observation %d to object %d", best_obs, best_obj);
            knownThings.at(best_obj).appendObservation(visibles.at(best_obs));
            knownThings.at(best_obj).appendObservation(theGround);   //maintain the assertion that the object is on or near the ground.
            best_obj = obj_index[--n_obj];  //remove one of each (put the tail in the gap)
            best_obs = obs_index[--n_obs];
        }else{
            LogSimple(LOG_DEBUG,"Confidence below threshold, creating %d objects", n_obs);
            break;
        }
    }
    if(n_obj == 0) LogSimple(LOG_DEBUG,"Run out of objects, Creating %d objects", n_obs);
    for(uint j=0; j<n_obs; j++){
        LogSimple(LOG_DEBUG,"Adding new object from obs %d", obs_index[j]);
        Observations newThing(theGround);   //starting assumption
        newThing.appendObservation(visibles.at(obs_index[j]));
        knownThings.push_back(newThing);
        LogSimple(LOG_DEBUG,"Done sorting");
    }
    delete[] fits;
    delete[] obj_index;
    delete[] obs_index;
    return true;
}

bool ObjectTracker::ChooseObsToObj(std::vector<Observation> &visibles, std::vector<Observations> &knownThings){
    Observation theGround = AssumptionGroundLevel();
    if(knownThings.size() <= 0){
        if(visibles.size() > 0){    //add the first thing on the list
            LogSimple(LOG_DEBUG,"Adding new object from obs %d", 0);
            Observations newThing(theGround);   //starting assumption
            newThing.appendObservation(visibles.at(0));
            knownThings.push_back(newThing);
        }
    }else{
        uint best = 0;  //the index of the best fitting object
        double fit = 0; //how well that object fit.
        for(uint i=0; i<visibles.size(); i++){
            double thisFit = knownThings.at(0).getSameProbability(visibles.at(i));
            if(thisFit>fit){
                fit = thisFit;
                best = i;
            }
        }
        knownThings.at(0).appendObservation(visibles.at(best));
        knownThings.at(0).appendObservation(theGround);
    }
    return true;
}


bool ObjectTracker::NoObjectMemory(std::vector<Observation> &visibles, std::vector<Observations> &knownThings){
    Observation theGround = AssumptionGroundLevel();
    if(visibles.size() > 0){    //add the first thing on the list
        knownThings.clear();    //destroy previous objects
        Observations newThing(theGround);   //starting assumption
        newThing.appendObservation(visibles.at(0));
        knownThings.push_back(newThing);
    }
    return true;
}

/**
 * Calculates the desired location of the copter based on the estimated location of the object.
 * @param [in] fc The flight controller.
 * @param [in] object The detected object.
 * @param [in] has_fix Indicates if we have a fix on the object or not. (deprecate?)
 */
Coord3D ObjectTracker::CalculateVantagePoint(GPSData *pos, Observations *object, bool has_fix){
    //spit out a Coord3D indicating the optimal location to see the object
        //go to a fixed radius from the object (or fly to the north, into the sun etc)
        //calculate a position to make a better observation from?

    Coord3D copter_coord = {pos->fix.lat, pos->fix.lon, pos->fix.alt};
    Vec3d copterloc = GroundFromGPS(copter_coord);
    Vec3d rel_loc =  object->getLocation().vect - copterloc;
    if(rel_loc(2)<0){
        //copter is below the target, don't move.
        return copter_coord;
    }
    double height_above_target = rel_loc(2);
    double level_radius = sqrt((rel_loc(0) * rel_loc(0))+(rel_loc(1) * rel_loc(1)));

    double des_level_radius = rel_loc(2)/desiredSlope;  //what radius is comfortable for this height?

    rel_loc *= des_level_radius/level_radius;   //
    rel_loc(2) = height_above_target;

    Vec3d des_copter_loc = object->getLocation().vect - rel_loc;

    Coord3D vantage = GPSFromGround(des_copter_loc);
    //LogSimple(LOG_DEBUG, "Vantage point lat: %.6f, lon: %.6f, alt %.2f", vantage.lat, vantage.lon, vantage.alt);
    return vantage;

}


void ObjectTracker::PathWaypoint(FlightController *fc, GPSData *pos, IMUData *imu_data, Coord3D dest, Coord3D poi){
    Coord3D copter_coord = {pos->fix.lat, pos->fix.lon, pos->fix.alt};
    Vec3d copter_loc = GroundFromGPS(copter_coord);
    Vec3d poi_loc = GroundFromGPS(poi);
    Vec3d poi_offset = poi_loc - copter_loc;
    double phi = RAD2DEG(atan2(poi_offset(1),poi_offset(0)));   //yaw Angle to object (what is our current yaw?)
    if(phi<0) phi +=360;
    
    LogSimple(LOG_DEBUG, "Sending alt %3.2f",dest.alt);
    dest.alt=0; //hack to not change altitude
    fc->fb->SetGuidedWaypoint(waypoint_seq++, 1, 0, dest, true);
    //sequence number, radius, dwell time, waypoint, is relative
    fc->fb->SetYaw(phi, false);

}

/**
 * Execute a control loop to put the copter at the vantage point
 * @param [in] fc The flight controller.
 * @param [in] pos The GPS location of the copter
 * @param [in] imu_data The pitch and roll of the copter
 * @param [in] dest The settling point of the loop
 * @param [out] course The velocity output from the loops
 */
void ObjectTracker::CalculatePath(FlightController *fc, GPSData *pos, IMUData *imu_data, Coord3D dest, Coord3D poi, Vec3D *course){
    //Observe Exclusion Zones?
    //really feel like I shouldn't be handling this on the Pi. I can just spool the above waypoint to the pixhawk.

    double trackx, tracky, trackw;
    //Zero the course commands
    memset(course, 0, sizeof(Vec3D));

    Coord3D copter_coord = {pos->fix.lat, pos->fix.lon, pos->fix.alt};
    Vec3d copter_loc = GroundFromGPS(copter_coord);
    Vec3d dest_loc = GroundFromGPS(dest);
    Vec3d poi_loc = GroundFromGPS(poi);

    Vec3d offset = dest_loc - copter_loc;
    Vec3d poi_offset = poi_loc - copter_loc;


    double phi = RAD2DEG(atan2(poi_offset(1),poi_offset(0)));   //yaw Angle to object (what is our current yaw?)
    //LogSimple(LOG_DEBUG, "Path: copter at N %.2f, E %.2f, Yaw %.2f", copter_loc(0), copter_loc(1), imu_data->yaw);
    //LogSimple(LOG_DEBUG, "Path: dest   at N %.2f, E %.2f, Yaw %.2f", dest_loc(0), dest_loc(1), RAD2DEG(atan2(offset(1),offset(0))));
    //LogSimple(LOG_DEBUG, "Path: target at N %.2f, E %.2f, Yaw %.2f", poi_loc(0), poi_loc(1), phi);


    Matx33d phi_mat = rotationMatrix(0,0,-imu_data->yaw);
    //LogSimple(LOG_DEBUG, "Path: relative: Forward %.2f, Right %.2f", (phi_mat * offset)(0), (phi_mat * offset)(1));

    //m_pidw.SetSetPoint(0);  //We can't use set-points properly because the PID loops need to cooperate between pitch and roll.
    m_pidx.SetSetPoint(0);  //We can't use set-points properly because the PID loops need to cooperate between pitch and roll.
    m_pidy.SetSetPoint(0);  //maybe swap x,y out for PID on distance and object bearing?

    //m_pidw.SetProcessValue(-phi);
    m_pidx.SetProcessValue(-(phi_mat * offset)(0));
    m_pidy.SetProcessValue(-(phi_mat * offset)(1));
    //m_pidx.SetProcessValue(-offset(0));
    //m_pidy.SetProcessValue(-offset(1));
    

    //m_pidz.SetProcessValue(  //throttle controller not used


//    LogSimple(LOG_DEBUG, "PIDW: %.2f, PIDY: %.2f", -phi, -offset(1));
    trackw = m_pidw.Compute();
    trackx = m_pidx.Compute();
    tracky = m_pidy.Compute();
    //trackz = m_pidz.Compute();

    //TODO: course->z? This controls altitude.
    //To change yaw, use fb->SetYaw(bearing_or_offset, is_relative)
    //fc->fb->SetYaw(phi, false);
    if(phi<0) phi +=360;
    fc->fb->SetYaw(phi, false);
    //is the course vector in NED?
    course->y = trackx;
    course->x = tracky;
    //Fix the angle for now...
    //course->gimbal = 50;
    fc->cam->SetTrackingArrow({course->x/TRACK_SPEED_LIMIT_X,
        -course->y/TRACK_SPEED_LIMIT_Y, trackw/TRACK_SPEED_LIMIT_W});

}
//using North, East, Down coordinates
Vec3d ObjectTracker::GroundFromGPS(Coord3D coord){

    Vec3d retval(
        DEG2RAD(coord.lat - launch_point.lat) * (1000*RADIUS_OF_EARTH),
        DEG2RAD(coord.lon - launch_point.lon) * (1000*RADIUS_OF_EARTH*cos(DEG2RAD(launch_point.lat))),
        (launch_point.alt - coord.alt)
        );
    return retval;
}
Coord3D ObjectTracker::GPSFromGround(Vec3d coord){
    Coord3D retval;
    retval.lat = launch_point.lat + RAD2DEG(coord(0)/(1000*RADIUS_OF_EARTH));
    retval.lon = launch_point.lon + RAD2DEG(coord(1)/(1000*RADIUS_OF_EARTH*cos(DEG2RAD(launch_point.lat))));
    retval.alt = launch_point.alt - coord(2);
    return retval;
}

