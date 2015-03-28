/**
 * @file flightcontroller.h
 * @brief Defines the FlightController class.
 */

#ifndef _PICOPTERX_FLIGHTCONTROLLER_H
#define _PICOPTERX_FLIGHTCONTROLLER_H

#include "buzzer.h"
#include "gps_feed.h"
#include "imu_feed.h"
#include "flightboard.h"

namespace picopter {
    /* Forward declaration of the options class */
    class Options;
    /* Forward declaration of the flight controller class */
    class FlightController;
    
    /**
     * An enumeration describing the possible states of the flight controller.
     */
    typedef enum ControllerState{
        /** All stopped and not running anything. **/
        STATE_STOPPED,
        /** Inferring the bearing of the hexacopter using the GPS. **/
        STATE_INFER_BEARING
    } ControllerState;
    
    /**
     * An enumeration identifying the task to be run.
     */
    typedef enum TaskIdentifier {
        /** No task (i.e. not running anything). **/
        TASK_NONE,
        /** General waypoints task. **/
        TASK_WAYPOINTS,
        /** Waypoints special case: Lawnmower search pattern. **/
        TASK_LAWNMOWER,
        /** Use the camera to track an object. **/
        TASK_OBJECT_TRACKING,
        /** Track the user using coordinates supplied by them (e.g. phone). **/
        TASK_USER_TRACKING,
        /** Waypoints special case: Spiral up/down pattern. **/
        TASK_SPIRAL_SEARCH
    } TaskIdentifier;
    
    /**
     * The base class for all flight tasks.
     * All flight tasks must inherit from this class and implement its methods.
     */
    class FlightTask {
        public:
            /**
             * The method that will be called by the flight controller to
             * perform the task.
             * @param fc The pointer to the calling flight controller
             * @param opts Task-specific options.
             */
            virtual void Run(const FlightController *fc, void *opts) = 0;
    };
    
    /**
     * The base controller for the hexacopter.
     * It ties in all the actuators and sensors for access from a central point.
     */
    class FlightController {
        public:
            FlightController();
            FlightController(Options *opts);
            virtual ~FlightController();
            
            ControllerState GetCurrentState();
            TaskIdentifier GetCurrentTaskId();
            void Stop();
            bool CheckForStop();
            bool Sleep(int ms);
            bool RunTask(TaskIdentifier tid, FlightTask *task, void *opts);
            bool InferBearing(double *ret, int move_time=5000);
            
            /** A pointer to the flight board controller instance. **/
            FlightBoard* const &fb;
            /** A pointer to the IMU instance, or nullptr if not present. **/
            IMU* const &imu;
            /** A pointer to the GPS instance. **/
            GPS* const &gps;
            /** A pointer to the Buzzer instance. **/
            Buzzer* const &buzzer;
        private:
            /** Holds the sleep interval in ms. **/
            static const int SLEEP_PERIOD = 200;
            /** Holds the Buzzer instance. **/
            Buzzer *m_buzzer;
            /** Holds the GPS instance. **/
            GPS *m_gps;
            /** Holds the IMU instance. **/
            IMU *m_imu;
            /** Holds the flight board controller instance. **/
            FlightBoard *m_fb;
            
            /** Indicates if all operations should be stopped. **/
            std::atomic<bool> m_stop;
            /** Holds the current state of the flight controller. **/
            std::atomic<ControllerState> m_state;
            /** Holds the current task that is being run. **/
            std::atomic<TaskIdentifier> m_task_id;
            /** Holds the thread that runs the task. **/
            std::future<void> m_task_thread;
            /** The mutex used to control the currently run task. **/
            std::mutex m_task_mutex;
            
            /** Copy constructor (disabled) **/
            FlightController(const FlightController &other);
            /** Assignment operator (disabled) **/
            FlightController& operator= (const FlightController &other);
    };
}

#endif // _PICOPTERX_FLIGHTCONTROLLER_H