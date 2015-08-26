/**
 * @file flightboard.h
 * @brief Defines the FlightBoard class.
 */

#ifndef _PICOPTERX_FLIGHTBOARD_H
#define _PICOPTERX_FLIGHTBOARD_H

/* For the Options class */
#include "opts.h"
/* For Coord3D */
#include "navigation.h"
/* For MAVProxy includes and other related baggage */
#include "mavcommslink.h"

namespace picopter {
    /* Forward declaration of the GPS class */
    class GPS;
    /* Forward declaration of the IMU class */
    class IMU;
    
    typedef navigation::EulerAngle GimbalAngle;
    
    /**
     * Contains information about the actuation of the hexacopter
     */
    typedef struct FlightData {
        /** Aileron speed, -100 to 100 **/
        int aileron;
        /** Elevator speed, -100 to 100 **/
        int elevator;
        /** Rudder speed, -100 to 100 **/
        int rudder;
        /** Gimbal angle, full Euler angles **/
        GimbalAngle gimbal;
    } FlightData;

    /**
     * Controls the actuation of the hexacopter.
     */
    class FlightBoard {
        public:
            typedef std::function<void(const mavlink_message_t*)> EventHandler;

            FlightBoard();
            FlightBoard(Options *opts);
            virtual ~FlightBoard();
            
            GPS* GetGPSInstance();
            IMU* GetIMUInstance();
            
            bool IsAutoMode();
            void Stop();

            //void SetLocalPosition(Coord4D pt);
            //void SetGlobalPosition(Coord4D pt);
            //void SetSpeed(Coord4D sp);
            //void SetAccel(Coord3D acc);
            bool SetGuidedWaypoint(int seq, float radius, float wait, float lat, float lon, float alt, bool relative_alt);
            bool SetRegionOfInterest(navigation::Coord3D roi);
            bool UnsetRegionOfInterest();
            bool SetWaypointSpeed(int sp);
            
            void GetData(FlightData *d);
            void SetData(FlightData *d);
            void SetAileron(int speed);
            void SetElevator(int speed);
            void SetRudder(int speed);
            void SetGimbal(GimbalAngle pose);

            int RegisterHandler(int msgid, EventHandler handler);
            void DeregisterHandler(int handlerid);
            void SendMessage(mavlink_message_t *msg);
        private:
            /** The autopilot connection timeout (in s) **/
            static const int HEARTBEAT_TIMEOUT_DEFAULT = 10;

            /** The hearbeat timeout **/
            int m_heartbeat_timeout;
            /** The time since the last heartbeat **/
            int m_last_heartbeat;
            /** Our GPS instance (separate class to handle GPS data parsing) **/
            GPS *m_gps;
            /** Our IMU instance (separate class to handle IMU data parsing) **/
            IMU *m_imu;
            /** Holds current flight data **/
            FlightData m_currentData;
            /** The MAVLink data connection **/
            MAVCommsLink *m_link;
            /** The shutdown signal **/
            std::atomic<bool> m_shutdown;
            /** Whether or not we are setting waypoints **/
            std::atomic<bool> m_waypoints_mode;
            /** Output worker mutex **/
            std::mutex m_output_mutex;
            /** Message receiving thread **/
            std::thread m_input_thread;
            /** Message sending thread **/
            std::thread m_output_thread;
            /** The system ID of the flight board we connect to **/
            int m_system_id;
            /** The component ID of the flight board we connect to **/
            int m_component_id;
            /** Our component ID that identifies us **/
            int m_flightboard_id;
            /** The current yaw of the copter **/
            std::atomic<double> m_current_yaw;
            /** Are we in auto (Guided) mode? **/
            std::atomic<bool> m_is_auto_mode;
            /** The event handler table **/
            EventHandler m_handler_table[256];

            /** Loop to receive and dispatch MAVLink  messages **/
            void InputLoop();
            /** Loop to send the setpoint via MAVLink **/
            void OutputLoop();
            /** Copy constructor (disabled) **/
            FlightBoard(const FlightBoard &other);
            /** Assignment operator (disabled) **/
            FlightBoard& operator= (const FlightBoard &other);
    };
}

#endif // _PICOPTERX_FLIGHTBOARD_H