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
    
    /**
     * Struct to hold information that might be displayed on a heads-up display.
     * This is basically the same as the MAVLink VFR_HUD message, plus a bit
     * extra.
     */
    typedef struct HUDInfo {
        /** UNIX epoch offset, in seconds **/
        int64_t unix_time_offset;
        /** Air speed, in m/s **/
        float air_speed;
        /** Ground speed, in m/s **/
        float ground_speed;
        /** Heading, in degrees **/
        int16_t  heading;
        /** Throttle percentage **/
        uint16_t throttle;
        /** Altitude above mean-sea level (m) **/
        float alt_msl;
        /** Climb rate; m/s **/
        float climb;
        
        /** LIDAR reading (m) **/
        float lidar;
        /** Gimbal position **/
        navigation::EulerAngle gimbal;
        
        /** Battery voltage, in Volts **/
        float batt_voltage;
        /** Battery current, in Amps **/
        float batt_current;
        /** Remaining battery capacity (percentage) **/
        int32_t batt_remaining;
        
        /** Position (altitude is above ground) **/
        navigation::Coord3D pos;
        /** Status message **/
        std::string status1;
        /** Flightboard status message **/
        std::string status2;
    } HUDInfo;

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
            void GetGimbalPose(navigation::EulerAngle *p);
            bool GetHomePosition(navigation::Coord3D *p);
            void GetLatestHUD(HUDInfo *i);
            
            bool IsAutoMode();
            bool IsRTL();
            bool IsInAir();
            bool IsArmed();
            
            void Stop();
            bool DoGuidedTakeoff(int alt);
            bool DoReturnToLaunch();
            
            bool SetGuidedWaypoint(int seq, float radius, float wait, navigation::Coord3D pt, bool relative_alt);
            bool SetWaypointSpeed(int sp);
            bool SetBodyVel(navigation::Vec3D v);
            bool SetBodyPos(navigation::Vec3D p);
            bool SetYaw(int bearing, bool relative);
            bool SetGimbalPose(navigation::EulerAngle pose);
            bool ConfigureGimbal();
            bool SetRegionOfInterest(navigation::Coord3D roi);
            bool UnsetRegionOfInterest();

            int RegisterHandler(int msgid, EventHandler handler);
            void DeregisterHandler(int handlerid);
            void SendMessage(mavlink_message_t *msg);
        private:
            /** The autopilot connection timeout (in s) **/
            static const int HEARTBEAT_TIMEOUT_DEFAULT = 4;

            /** The hearbeat timeout **/
            int m_heartbeat_timeout;
            /** The time since the last heartbeat **/
            int m_last_heartbeat;
            /** Our GPS instance (separate class to handle GPS data parsing) **/
            GPS *m_gps;
            /** Our IMU instance (separate class to handle IMU data parsing) **/
            IMU *m_imu;
            /** The MAVLink data connection **/
            MAVCommsLink *m_link;
            /** The shutdown signal **/
            std::atomic<bool> m_shutdown;
            /** Whether or not to disable local position sending **/
            std::atomic<bool> m_disable_local;
            /** Output worker mutex **/
            std::mutex m_output_mutex;
            /** Gimbal mutex **/
            std::mutex m_gimbal_mutex;
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
            /** Are we in RTL mode? **/
            std::atomic<bool> m_is_rtl;
            /** Are we flying? **/
            std::atomic<bool> m_is_in_air;
            /** Are the motors armed? **/
            std::atomic<bool> m_is_armed;
            /** Do we have a home position? **/
            std::atomic<bool> m_has_home_position;
            /** Watchdog counter on sending relative commands. **/
            int m_rel_watchdog;
            /** The current gimbal position **/
            navigation::EulerAngle m_gimbal;
            /** The home position (usually launch point) **/
            navigation::Coord3D m_home_position;
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
