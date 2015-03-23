/**
 * @file flightboard.h
 * @brief Defines the FlightBoard class.
 */

#ifndef _PICOPTERX_FLIGHTBOARD_H
#define _PICOPTERX_FLIGHTBOARD_H

namespace picopter {
    /* Forward declaration of the options class */
    class Options;
    
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
            /** File handle to ServoBlaster **/
            FILE *m_fp;
            
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