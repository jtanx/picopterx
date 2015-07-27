/**
 * @file flightboard.h
 * @brief Defines the FlightBoard class.
 */

#ifndef _PICOPTERX_FLIGHTBOARD_H
#define _PICOPTERX_FLIGHTBOARD_H

/* For MAVProxy includes and other related baggage */
#include "mavcommslink.h"

namespace picopter {
    /* Forward declaration of the options class */
    class Options;
    /* Forward declaration of the MAVCommsLink class */
    class MAVCommsLink;
    
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
        /** Gimbal angle, 0 to 90 **/
        int gimbal;
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
            
            bool IsAutoMode();
            void Stop();

            //void SetLocalPosition(Coord4D pt);
            //void SetGlobalPosition(Coord4D pt);
            //void SetSpeed(Coord4D sp);
            //void SetAccel(Coord3D acc);

            void GetData(FlightData *d);
            void SetData(FlightData *d);
            void SetAileron(int speed);
            void SetElevator(int speed);
            void SetRudder(int speed);
            void SetGimbal(int pos);

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
            /** Holds current flight data **/
            FlightData m_currentData;
            /** The MAVLink data connection **/
            MAVCommsLink *m_link;
            /** The shutdown signal **/
            std::atomic<bool> m_shutdown;
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