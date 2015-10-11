/**
 * @file flightcontroller.h
 * @brief Defines the FlightController class.
 */

#ifndef _PICOPTERX_FLIGHTCONTROLLER_H
#define _PICOPTERX_FLIGHTCONTROLLER_H

/* For the Options class */
#include "opts.h"
#include "buzzer.h"
#include "gps_gpsd.h"
#include "gps_mav.h"
#include "imu_feed.h"
#include "flightboard.h"
#include "camera_stream.h"
#include "lidar.h"

namespace picopter {
    /* Forward declaration of the flight task class */
    class FlightTask;
    
    /**
     * An enumeration describing the possible states of the flight controller.
     */
    typedef enum ControllerState{
        /** All stopped and not running anything. **/
        STATE_STOPPED,
        /** In RTL mode. **/
        STATE_RTL,
        /** Waiting for a GPS fix. **/
        STATE_GPS_WAIT_FOR_FIX,
        /** Awaiting user authorisation (the auto mode switch). **/
        STATE_AWAITING_AUTH,
        /** Inferring the bearing of the hexacopter using the GPS. **/
        STATE_INFER_BEARING,
        /** Moving towards the waypoint **/
        STATE_WAYPOINTS_MOVING,
        /** Waiting at the waypoint **/
        STATE_WAYPOINTS_IDLING,
        /** Finished the waypoints manoeuver **/
        STATE_WAYPOINTS_FINISHED,
        /** Searching for an object to track **/
        STATE_TRACKING_SEARCHING,
        /** Tracking a found object **/
        STATE_TRACKING_LOCKED,
        /** Tracking a user **/
        STATE_TRACKING_USER,
        /** Performing environmental mapping **/
        STATE_ENV_MAPPING,
        /** Awaiting motor arming **/
        STATE_UTILITY_AWAITING_ARM,
        /** Performing a take-off **/
        STATE_UTILITY_TAKEOFF,
        /** Joystick control **/
        STATE_UTILITY_JOYSTICK,
        /** Taking pictures **/
        STATE_UTILITY_PICTURES,
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
        TASK_SPIRAL_SEARCH,
        /** Environmental mapping **/
        TASK_ENVIRONMENTAL_MAPPING,
        /** Utility task **/
        TASK_UTILITY
    } TaskIdentifier;
    
    /**
     * The base controller for the hexacopter.
     * It ties in all the actuators and sensors for access from a central point.
     */
    class FlightController {
        /* Allow access to set the controller state */
        friend FlightTask;
        public:
            FlightController();
            FlightController(Options *opts);
            virtual ~FlightController();
            
            ControllerState GetCurrentState();
            TaskIdentifier GetCurrentTaskId();
            void Stop();
            bool CheckForStop();
            bool Sleep(int ms);
            bool WaitForAuth();
            bool ReloadSettings(Options *opts);
            bool RunTask(TaskIdentifier tid, std::shared_ptr<FlightTask> task, void *opts);
            
            friend std::ostream& operator<<(std::ostream &stream, FlightController &fc);
            
            /** A pointer to the flight board controller instance. **/
            FlightBoard* const &fb;
            /** A pointer to the IMU instance, or nullptr if not present. **/
            IMU* const &imu;
            /** A pointer to the GPS instance. **/
            GPS* const &gps;
            /** A pointer to the Buzzer instance. **/
            Buzzer* const &buzzer;
            /** A pointer to the Camera stream instance.**/
            CameraStream* const &cam;
            /** A pointer to the LIDAR instance. **/
            Lidar* const &lidar;
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
            /** Holds the Camera stream instance. **/
            CameraStream *m_camera;
            /** Holds the LIDAR instance. **/
            Lidar *m_lidar;
            
            /** Indicates if all operations should be stopped. **/
            std::atomic<bool> m_stop;
            /** Indicates when the flight controller should shutdown. **/
            std::atomic<bool> m_quit;
            /** Holds the current state of the flight controller. **/
            std::atomic<ControllerState> m_state;
            /** Holds the current task that is being run. **/
            std::atomic<TaskIdentifier> m_task_id;
            /** Holds the thread that runs the task. **/
            std::future<void> m_task_thread;
            /** The mutex used to control the currently run task. **/
            std::mutex m_task_mutex;
            /** Secondary mutex to control flight controller modifications. **/
            std::mutex m_control_mutex;
            /** The current task **/
            std::shared_ptr<FlightTask> m_task;
            /** HUD info **/
            HUDInfo m_hud;
            /** Flightboard status text **/
            std::string m_fb_status_text;
            /** Poor man's timeout **/
            int m_fb_status_counter;
            
            /** HUD Loop updater **/
            void HUDParser(const mavlink_message_t *msg);
            /** Update the current state **/
            ControllerState SetCurrentState(ControllerState state);
            /** Copy constructor (disabled) **/
            FlightController(const FlightController &other);
            /** Assignment operator (disabled) **/
            FlightController& operator= (const FlightController &other);
    };
    
    /**
     * The base class for all flight tasks.
     * All flight tasks must inherit from this class and implement its methods.
     */
    class FlightTask {
        public:
            /**
             * The destructor. Will be called immediately after Run() exits.
             */
            virtual ~FlightTask() {};
            /**
             * The method that will be called by the flight controller to
             * perform the task.
             * @param fc The pointer to the calling flight controller
             * @param opts Task-specific options.
             */
            virtual void Run(FlightController *fc, void *opts) = 0;
            /**
             * Indicates whether or not the task has finished running.
             * @return true iff the task has finished running.
             */
            virtual bool Finished() = 0;
        protected:
            /**
             * Sets the current state of the parent flight controller.
             * @return The previous controller state.
             */
            static ControllerState SetCurrentState(FlightController *fc, ControllerState state) {
                return fc->SetCurrentState(state);
            };
    };
}

#endif // _PICOPTERX_FLIGHTCONTROLLER_H
