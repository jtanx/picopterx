/**
 * @file picopter.h
 * @brief The main include file
 */

#ifndef _PICOPTERX_PICOPTER_H
#define _PICOPTERX_PICOPTER_H

//Base includes
#include "config.h"
#include "common.h"
#include "log.h"
#include "opts.h"

//Modules
#include "gpio.h"
#include "buzzer.h"
#include "flightboard.h"
#include "gps_feed.h"
#include "imu_feed.h"

namespace picopter {
    template <typename T>
    inline T clamp(const T& n, const T& lower, const T& upper) {
        return std::max(lower, std::min(n, upper));
    }
    
    class FlightController {
        public:
            FlightBoard *fb;
            Buzzer *b;
            GPS *gps;
            IMU *imu;
            
            //Bit iffy on this...
            FlightController(Options *opts) {
                b = new Buzzer();
                fb = new FlightBoard(opts);
                gps = new GPS(opts);
                try {
                    imu = new IMU(opts);
                } catch (std::invalid_argument) {
                    imu = NULL;
                }
            };
            
            virtual ~FlightController() {
                delete b;
                delete fb;
                delete gps;
                delete imu;
            };
        private:
            /** Copy constructor (disabled) **/
            FlightController(const FlightController &other);
            /** Assignment operator (disabled) **/
            FlightController& operator= (const FlightController &other);
        
    };
}


#endif // _PICOPTERX_PICOPTER_H