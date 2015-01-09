/**
 * @file flightboard.h
 * @brief Defines the FlightBoard class.
 */

#ifndef _PICOPTERX_FLIGHTBOARD_H
#define _PICOPTERX_FLIGHTBOARD_H

namespace picopter {
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
            virtual ~FlightBoard();
            void start();
            void stop();
            void getData(FlightData *d);
            void setData(FlightData *d);
            void setAileron(int speed);
            void setElevator(int speed);
            void setRudder(int speed);
            void setGimbal(int pos);
            
        private:
            /** Holds current flight data **/
            FlightData m_currentData;
            /** File handle to ServoBlaster **/
            FILE *m_fp;
            /** Determines if we should be actuating or not **/
            std::atomic<bool> m_activated;
            
            void actuate();
            void setChannel(int channel, int value);
    };
}

#endif // _PICOPTERX_FLIGHTBOARD_H