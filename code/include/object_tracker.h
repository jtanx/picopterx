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
#include <opencv2/opencv.hpp>
 
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
        
            ObjectTracker(TrackMethod method=TRACK_STRAFE);
            ObjectTracker(Options *opts, TrackMethod method=TRACK_STRAFE);
            virtual ~ObjectTracker() override;
            
            TrackMethod GetTrackMethod();
            void SetTrackMethod(TrackMethod method);
            void Run(FlightController *fc, void *opts) override;
            bool Finished() override;
        private:
            bool m_observation_mode;
            PID m_pidw, m_pidx, m_pidy;//, m_pidz;
            std::atomic<TrackMethod> m_track_method;
            std::atomic<bool> m_finished;
            
            //int TRACK_TOL, 
            int SEARCH_GIMBAL_LIMIT;
            double TRACK_Kpw, TRACK_Kpx, TRACK_Kpy, TRACK_Kpz;
            double TRACK_TauIw, TRACK_TauIx, TRACK_TauIy, TRACK_TauIz;
            double TRACK_TauDw, TRACK_TauDx, TRACK_TauDy, TRACK_TauDz; 
            double TRACK_SETPOINT_W, TRACK_SETPOINT_X, TRACK_SETPOINT_Y, TRACK_SETPOINT_Z;
            int TRACK_SPEED_LIMIT_W, TRACK_SPEED_LIMIT_X, TRACK_SPEED_LIMIT_Y, TRACK_SPEED_LIMIT_Z;
            
            void EstimatePositionFromImageCoords(GPSData *pos, navigation::EulerAngle *gimbal, IMUData *imu_data, ObjectInfo *object);
            //void AbsoluteFromRelative(GPSData *pos, IMUData *imu_data, ObjectInfo *object);
            void CalculateTrackingTrajectory(FlightController *fc, navigation::Vec3D *current, ObjectInfo *object, bool has_fix);
            /** Copy constructor (disabled) **/
            ObjectTracker(const ObjectTracker &other);
            /** Assignment operator (disabled) **/
            ObjectTracker& operator= (const ObjectTracker &other);
    };
}

#endif // _PICOPTERX_OBJECT_TRACKER_H
