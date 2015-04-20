/**
 * @file object_tracker.h
 * @brief The header file for the object tracking code.
 */

#ifndef _PICOPTERX_OBJECT_TRACKER_H
#define _PICOPTERX_OBJECT_TRACKER_H

#include "flightcontroller.h"
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
            PID m_pid1;
            
            /** Copy constructor (disabled) **/
            ObjectTracker(const ObjectTracker &other);
            /** Assignment operator (disabled) **/
            ObjectTracker& operator= (const ObjectTracker &other);
    };
}

#endif // _PICOPTERX_OBJECT_TRACKER_H