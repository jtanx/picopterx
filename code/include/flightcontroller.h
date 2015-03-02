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

    /**
     * The base controller for the hexacopter.
     * It ties in all the actuators and sensors for access from a central point.
     */
    class FlightController {
        public:
            FlightController();
            FlightController(Options *opts);
            virtual ~FlightController();
            
            /** A pointer to the flight board controller instance. **/
            FlightBoard* const &fb;
            /** A pointer to the IMU instance, or nullptr if not present. **/
            IMU* const &imu;
            /** A pointer to the GPS instance. **/
            GPS* const &gps;
            /** A pointer to the Buzzer instance. **/
            Buzzer* const &buzzer;
        private:
            /** Holds the Buzzer instance. **/
            Buzzer *m_buzzer;
            /** Holds the GPS instance. **/
            GPS *m_gps;
            /** Holds the IMU instance. **/
            IMU *m_imu;
            /** Holds the flight board controller instance. **/
            FlightBoard *m_fb;
            
            /** Copy constructor (disabled) **/
            FlightController(const FlightController &other);
            /** Assignment operator (disabled) **/
            FlightController& operator= (const FlightController &other);
    };
}

#endif // _PICOPTERX_FLIGHTCONTROLLER_H