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
            ObjectTracker();
            ObjectTracker(Options *opts);
            virtual ~ObjectTracker() override;
            
            void Run(FlightController *fc, void *opts) override;
        private:
            PID m_pidx, m_pidy;
            int TRACK_TOL, TRACK_SPEED_LIMIT, SEARCH_GIMBAL_LIMIT;
            double TRACK_Kp, TRACK_TauI, TRACK_TauD;
            double TRACK_SETPOINT_X, TRACK_SETPOINT_Y;
            
            void CalculateTrackingTrajectory(FlightData *course, navigation::Point2D *object_location);
            
            /** Copy constructor (disabled) **/
            ObjectTracker(const ObjectTracker &other);
            /** Assignment operator (disabled) **/
            ObjectTracker& operator= (const ObjectTracker &other);
    };
}

#endif // _PICOPTERX_OBJECT_TRACKER_H