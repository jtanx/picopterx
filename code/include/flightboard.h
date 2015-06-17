/**
 * @file flightboard.h
 * @brief Defines the FlightBoard class.
 */

#ifndef _PICOPTERX_FLIGHTBOARD_H
#define _PICOPTERX_FLIGHTBOARD_H

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
     * This class is not thread-safe.
     */
    class FlightBoard {
        public:
            FlightBoard();
            FlightBoard(Options *opts);
            virtual ~FlightBoard();
            bool IsAutoMode();
            void Stop();
            void GetData(FlightData *d);
            void SetData(FlightData *d);
            void SetAileron(int speed);
            void SetElevator(int speed);
            void SetRudder(int speed);
            void SetGimbal(int pos);
            
        private:
            /** Holds current flight data **/
            FlightData m_currentData;
            /** The MAVLink data connection **/
            MAVCommsLink *m_link;
            /** The shutdown signal **/
            std::atomic<bool> m_shutdown;
            /** Worker mutex **/
            std::mutex m_worker_mutex;
            /** Message receiving thread **/
            std::thread m_input_thread;
            /** The system ID of the flight board we connect to **/
            int m_system_id;
            /** The component ID of the flight board we connect to **/
            int m_component_id;
            /** Our component ID that identifies us **/
            int m_flightboard_id;
            /** Are we in auto (Guided) mode? **/
            std::atomic<bool> m_is_auto_mode;

            /** Loop to receive and dispatch MAVLink  messages **/
            void InputLoop();
            /** Copy constructor (disabled) **/
            FlightBoard(const FlightBoard &other);
            /** Assignment operator (disabled) **/
            FlightBoard& operator= (const FlightBoard &other);
            
            void Actuate();
            void FlushData();
            void SetChannel(int channel, int value);
    };
}

#endif // _PICOPTERX_FLIGHTBOARD_H