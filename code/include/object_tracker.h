/**
 * @file object_tracker.h
 * @brief The header file for the object tracking code.
 */

#ifndef _PICOPTERX_OBJECT_TRACKER_H
#define _PICOPTERX_OBJECT_TRACKER_H

/* For the Options class */
#include "opts.h"
#include "flightcontroller.h"
#include "navigation.h"
#include "PID.h"

namespace picopter {
    /**
     * Class for moving the hexacopter through waypoints.
     */
    class ObjectTracker : public FlightTask {
        public:
            typedef enum {
                TRACK_STRAFE,
                TRACK_ROTATE
            } TrackMethod;
        
            ObjectTracker(int camwidth, int camheight, TrackMethod method=TRACK_STRAFE);
            ObjectTracker(Options *opts, int camwidth, int camheight, TrackMethod method=TRACK_STRAFE);
            virtual ~ObjectTracker() override;
            
            TrackMethod GetTrackMethod();
            void SetTrackMethod(TrackMethod method);
            void Run(FlightController *fc, void *opts) override;
        private:
            bool m_observation_mode;
            int m_camwidth, m_camheight;
            PID m_pidw, m_pidx, m_pidy;//, m_pidz;
            std::atomic<TrackMethod> m_track_method;
            
            //int TRACK_TOL, 
            int SEARCH_GIMBAL_LIMIT;
            double TRACK_Kpw, TRACK_Kpx, TRACK_Kpy, TRACK_Kpz;
            double TRACK_TauIw, TRACK_TauIx, TRACK_TauIy, TRACK_TauIz;
            double TRACK_TauDw, TRACK_TauDx, TRACK_TauDy, TRACK_TauDz; 
            double TRACK_SETPOINT_W, TRACK_SETPOINT_X, TRACK_SETPOINT_Y, TRACK_SETPOINT_Z;
            int TRACK_SPEED_LIMIT_W, TRACK_SPEED_LIMIT_X, TRACK_SPEED_LIMIT_Y, TRACK_SPEED_LIMIT_Z;
            
            void EstimatePositionFromImageCoords(GPSData *pos, FlightData *current, navigation::Point2D *object_location, navigation::Point3D *object_position);
            void CalculateTrackingTrajectory(FlightController *fc, FlightData *course, navigation::Point3D *object_position, bool has_fix);
            /** Copy constructor (disabled) **/
            ObjectTracker(const ObjectTracker &other);
            /** Assignment operator (disabled) **/
            ObjectTracker& operator= (const ObjectTracker &other);
    };
}

#endif // _PICOPTERX_OBJECT_TRACKER_H
