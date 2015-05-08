/**
 * @file object_tracker.h
 * @brief The header file for the object tracking code.
 */

#ifndef _PICOPTERX_OBJECT_TRACKER_H
#define _PICOPTERX_OBJECT_TRACKER_H

#include "flightcontroller.h"
#include "navigation.h"
#include "PID.h"

namespace picopter {
    /* Forward declaration of the options class */
    class Options;

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
            PID m_pidx, m_pidy;
            std::atomic<TrackMethod> m_track_method;
            
            int TRACK_TOL, SEARCH_GIMBAL_LIMIT;
            double TRACK_Kp, TRACK_TauI, TRACK_TauD;
            double TRACK_SETPOINT_X, TRACK_SETPOINT_Y;
            int TRACK_SPEED_LIMIT_X, TRACK_SPEED_LIMIT_Y;
            
            void estimatePositionFromImageCoords(FlightController *fc, navigation::Point2D *object_location, navigation::Point3D *object_position);
            void CalculateTrackingTrajectory(FlightController *fc, FlightData *course, navigation::Point3D *object_position, bool has_fix);
            /** Copy constructor (disabled) **/
            ObjectTracker(const ObjectTracker &other);
            /** Assignment operator (disabled) **/
            ObjectTracker& operator= (const ObjectTracker &other);
    };
}

#endif // _PICOPTERX_OBJECT_TRACKER_H