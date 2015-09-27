/** 
* 3D modelling with spiral pattern
* Detect an object and set initial radius.
* Set an initial radius.
*/

#include "common.h"
#include "env_mapping.h"
#define _USE_MATH_DEFINES
#include <cmath>

using namespace picopter;
using namespace picopter::navigation;

EnvironmentalMapping::EnvironmentalMapping(Options *opts)
: m_finished{false}
{

}

EnvironmentalMapping::EnvironmentalMapping() {

}

EnvironmentalMapping::~EnvironmentalMapping() {

}

void EnvironmentalMapping::GotoLocation(FlightController *fc, Coord3D l, Coord3D roi, bool relative_alt) {
    GPSData d;
    double wp_distance, wp_alt_delta;     
    fc->fb->SetGuidedWaypoint(0, 3, 0, l, relative_alt);
    fc->fb->SetRegionOfInterest(roi);
    
    do {
        fc->gps->GetLatest(&d);
        wp_distance = CoordDistance(d.fix, l);
        wp_alt_delta = l.alt != 0 ? 
            std::fabs((d.fix.alt-d.fix.groundalt)-l.alt) : 0;
        fc->Sleep(100);
    } while ((wp_distance > 2 || wp_alt_delta > 0.2) && !fc->CheckForStop());
}

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

    SetCurrentState(fc, STATE_ENV_MAPPING);
    //double start_radius = fc->lidar->GetLatest(); 
    //double alt  = fc->gps->GetLatestRelAlt();
    GPSData d;
    fc->gps->GetLatest(&d);
    Coord3D centre = {d.fix.lat, d.fix.lon, d.fix.alt - d.fix.groundalt};
    //Vec3D offset = RotateBodyToNED(Vec3D{-5, 0, 0}, fc->imu->GetLatestYaw()); //5m to left of copter.
    centre = CoordAddOffset(centre, Vec3D{-7, 0, 0}); //5m To west of copter.

    fc->fb->SetRegionOfInterest(centre);
    for (int j=0; j<3; j+=1) {
        for (int i=0; i<360 && !fc->CheckForStop(); i=i+5) {
            Coord3D location = CoordAddOffset(centre, 7/*start_radius*/, i);
            GotoLocation(fc, location, centre, false);
            Log(LOG_DEBUG, "%d", i);
            //Log(LOG_DEBUG, "%.7f, %.7f", centre.lat, centre.lon);    
            std::string path = std::string(PICOPTER_HOME_LOCATION "/pics/mappingimages") +"_" +
            std::to_string(location.lat) + "_"+ 
            std::to_string(location.lon) + "_" + 
            std::to_string(location.alt) +
            std::string(".jpg");

            fc->cam->TakePhoto(path);
            fc->Sleep(100);        
        }
        centre.alt += j;
        Log(LOG_DEBUG, "NEW LOOP");
    }

    fc->fb->UnsetRegionOfInterest();
    fc->fb->Stop();
    m_finished = true;
}

bool EnvironmentalMapping::Finished() {
    return m_finished;
}




