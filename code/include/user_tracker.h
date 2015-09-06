/**
 * @file user_tracker.h
 * @brief The header file for the user tracking code.
 */

#ifndef _PICOPTERX_USER_TRACKER_H
#define _PICOPTERX_USER_TRACKER_H

/* For the Options class */
#include "opts.h"
#include "flightcontroller.h"
#include "navigation.h"

namespace picopter {
    /**
     * Class for following the user (follow-me mode)
     */
    class UserTracker : public FlightTask {
        public:
            UserTracker(Options *opts);
            UserTracker();
            virtual ~UserTracker() override;
            
            void UpdateUserPosition(navigation::Coord2D wpt);
            void Run(FlightController *fc, void *opts) override;
            bool Finished() override;
        private:
            /** The worker mutex **/
            std::mutex m_worker_mutex;
            /** Condition to signal the worker thread to wake up **/
            std::condition_variable m_signaller;
            /** The current user location **/
            navigation::Coord3D m_wpt;
            /** Indicates that there is a new user location for the worker thread **/
            bool m_wpt_available;
            /** Indicates whether or not we've finished running **/
            std::atomic<bool> m_finished;
            
            /** Copy constructor (disabled) **/
            UserTracker(const UserTracker &other);
            /** Assignment operator (disabled) **/
            UserTracker& operator= (const UserTracker &other);
    };
}

#endif // _PICOPTERX_USER_TRACKER_H
