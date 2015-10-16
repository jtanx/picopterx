/** 
* 3D modelling with spiral pattern
* Detect an object and set initial radius.
* Set an initial radius.
*/

/*
 Final functions structure:
 
 ->>>> After pressing the env mapping.
 Actions
 (1) Lift of the GND (3 meters)
 (2) Rotate to locate object (colour set by user)
 (3) When object found, stop rotating and move towards the object (at a slow speed).
 (4) Stop when the object is fiting the whole screen
 (5) Rotate aroud the POI whilst taking pic and increaisng alt. (Height increment after completing every loop.
 (6) If the object lost return to action (2)
 
 End after completing the loops set by user
 */

#include "common.h"
#include "env_mapping.h"
#include "utility.h"
#define _USE_MATH_DEFINES
#include <cmath>

using namespace picopter;
using namespace picopter::navigation;

EnvironmentalMapping::EnvironmentalMapping(Options *opts, int radius)
: m_finished{false}
, m_radius(radius)
{

}

EnvironmentalMapping::EnvironmentalMapping(int radius)
: EnvironmentalMapping(NULL, radius) {}

EnvironmentalMapping::~EnvironmentalMapping() {

}

void EnvironmentalMapping::GotoLocation(FlightController *fc, Coord3D l, Coord3D roi, bool relative_alt) {
    GPSData d;
    double wp_distance, wp_alt_delta;     
    fc->fb->SetGuidedWaypoint(0, 3, 0, l, relative_alt);
    fc->fb->SetRegionOfInterest(roi);
    
    do {
        if (fc->lidar) {
            float v = fc->lidar->GetLatest() / 100;
            if (v > 0.1 && v < 2) {
                //STAHP
                fc->fb->Stop();
                break;
            }
        }
        fc->gps->GetLatest(&d);
        wp_distance = CoordDistance(d.fix, l);
        wp_alt_delta = l.alt != 0 ? 
            std::fabs((d.fix.alt-d.fix.groundalt)-l.alt) : 0;
        fc->Sleep(100);
    } while ((wp_distance > 2 || wp_alt_delta > 0.2) && !fc->CheckForStop());
}

/*
void EnvironmentalMapping::SearchingMapping(FlightController *fc) {
    //Rotate to search
    while (!fc->CheckForStop()) {
        fc->fb->SetYaw(5, true);
        fc->Sleep(100);
        
        fc->cam->GetDetectedObjects(&objects);
        if (objects.size() > 0) {
            //We have a detected object
            break;
        }
    }
    
    //Move forward slowly
    Vec3D msp{0, 0.2, 0};
    while (!fc->CheckForStop()) {
        fc->fb->SetBodyVel(msp);
        fc->cam->GetDetectedObjects(&objects);
        if (objects.size() > 0) {
            ObjectInfo o = objects.front();
            if (o.bounds.area() > 0.3*(o.image_width * o.image_height)) {
                fc->fb->Stop();
                break;
                break;
            }
        }
    }
}
*/

void EnvironmentalMapping::Run(FlightController *fc, void *opts) {
    Log(LOG_INFO, "Environmental mapping initiated; awaiting authorisation...");
    SetCurrentState(fc, STATE_AWAITING_AUTH);

    if (!fc->WaitForAuth()) {
        Log(LOG_INFO, "All stop acknowledged; quitting!");
        return;
    }

    Log(LOG_INFO, "Authorisation acknowledged.");
    if (!fc->gps->WaitForFix(200)) {
        Log(LOG_WARNING, "No GPS fix; quitting.");
        return;
    }

    //Takeoff if we're not in the air
    if (!fc->fb->IsInAir()) {
        UtilityModule module(UtilityModule::UTILITY_TAKEOFF);
        module.Run(fc, reinterpret_cast<void*>(3));
    }
    
    SetCurrentState(fc, STATE_ENV_MAPPING);
    std::vector<ObjectInfo> objects;
    
    //while (!fc->CheckForStop()){
    //    o.bounds.area > 0.1*(o.image_width * o.image_height))
    
    
    //double start_radius = fc->lidar->GetLatest();
    //double alt  = fc->gps->GetLatestRelAlt();
    GPSData d;
    fc->gps->GetLatest(&d);
    Coord3D centre = {d.fix.lat, d.fix.lon, d.fix.alt - d.fix.groundalt};
    centre = CoordAddOffset(centre, 7, 90-fc->imu->GetLatestYaw()); //10m in front of copter
    //centre = CoordAddOffset(centre, Vec3D{-7, 0, 0}); //5m To west of copter.

    //Ah, magic offsets
    float initial_yaw = 270-fc->imu->GetLatestYaw();
    fc->fb->SetRegionOfInterest(centre);
    for (int j=1; j<4; j+=1) {
        for (int i=0; i<360 && !fc->CheckForStop(); i=i+5) {
            Coord3D location = CoordAddOffset(centre, m_radius /*7 start_radius*/, i+initial_yaw);
            GotoLocation(fc, location, centre, false);
            Log(LOG_DEBUG, "%d", i);
            
            fc->cam->GetDetectedObjects(&objects);
            if (objects.size() > 0) {
                //Log(LOG_DEBUG, "%.7f, %.7f", centre.lat, centre.lon);
                std::string path = std::string(PICOPTER_HOME_LOCATION "/pics/mappingimages") +"_" +
                std::to_string(location.lat) + "_"+
                std::to_string(location.lon) + "_" +
                std::to_string(location.alt) +
                std::string(".jpg");

                fc->cam->TakePhoto(path);
            }
            fc->Sleep(100);
        }
        centre.alt += j;
        Log(LOG_DEBUG, "NEW LOOP");
    }
    //}

    fc->fb->UnsetRegionOfInterest();
    fc->fb->Stop();
    m_finished = true;
}

    
bool EnvironmentalMapping::Finished() {
    return m_finished;
}

