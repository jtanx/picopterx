/**
 * @file utility.h
 * @brief The header file for the utility module.
 */

#ifndef _PICOPTERX_UTILITY_H
#define _PICOPTERX_UTILITY_H

/* For the Options class */
#include "opts.h"
#include "flightcontroller.h"
#include "navigation.h"
 
namespace picopter {
    /**
     * Utility class to perform tasks like take-offs etc.
     */
    class UtilityModule : public FlightTask {
        public:
            typedef enum {
                UTILITY_TAKEOFF,
                UTILITY_JOYSTICK,
                UTILITY_PICTURES
            } UtilityMethod;
            
            UtilityModule(Options *opts, UtilityMethod method);
            UtilityModule(UtilityMethod method);
            virtual ~UtilityModule() override;
            
            void UpdateJoystick(int throttle, int yaw, int x, int y);
            
            void Run(FlightController *fc, void *opts) override;
            bool Finished() override;
        private:
            /** Indicates whether or not we've finished running **/
            std::atomic<bool> m_finished;
            /** The utility method to perform. **/
            UtilityMethod m_method;
            /** Worker mutex. **/
            std::mutex m_worker_mutex;
            /** Indicates that there is new data for the worker thread **/
            bool m_data_available;
            /** Holds joystick information **/
            navigation::Vec4D m_joystick_data;
            /** Condition to signal the worker thread to wake up **/
            std::condition_variable m_signaller;
            
            /** Copy constructor (disabled) **/
            UtilityModule(const UtilityModule &other);
            /** Assignment operator (disabled) **/
            UtilityModule& operator= (const UtilityModule &other);
    };
}

#endif // _PICOPTERX_UTILITY_H
