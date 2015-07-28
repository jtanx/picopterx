/**
 * @file gps_mav.h
 * @brief GPS header for the MAVLink implementation.
 */

#ifndef _PICOPTERX_GPS_MAVLINK_H
#define _PICOPTERX_GPS_MAVLINK_H

/* For the Options class */
#include "opts.h"
#include "gps_feed.h"
#include "flightboard.h"

namespace picopter {
    /**
     * Class that interacts with the GPS.
     */
    class GPSMAV : public GPS {
        public:
            GPSMAV(picopter::FlightBoard *fb);
            GPSMAV(picopter::FlightBoard *fb, Options *opts);
            virtual ~GPSMAV() override;
        private:
            bool m_had_fix;
            DataLog m_log;
            
            /** Copy constructor (disabled) **/
            GPSMAV(const GPSMAV &other);
            /** Assignment operator (disabled) **/
            GPSMAV& operator= (const GPSMAV &other);
            void GPSInput(const mavlink_message_t *msg);
    };
}

#endif // _PICOPTERX_GPS_MAVLINK_H