/**
 * @file gps_gpsd.h
 * @brief GPS header for the gpsd implementation.
 */

#ifndef _PICOPTERX_GPS_GPSD_H
#define _PICOPTERX_GPS_GPSD_H

#include "gps_feed.h"

class gpsmm;

namespace picopter {
    /* Forward declaration of the options class */
    class Options;
    
    /**
     * Class that interacts with the GPS.
     */
    class GPSGPSD : public GPS {
        public:
            GPSGPSD();
            GPSGPSD(Options *opts);
            virtual ~GPSGPSD() override;
        private:
            /** The GPS read wait timeout (in us) **/
            static const int CYCLE_TIMEOUT_DEFAULT = 500000;

            int m_cycle_timeout;
            bool m_had_fix;
            std::thread m_worker;
            gpsmm *m_gps_rec;
            
            /** Copy constructor (disabled) **/
            GPSGPSD(const GPSGPSD &other);
            /** Assignment operator (disabled) **/
            GPSGPSD& operator= (const GPSGPSD &other);
            void GPSLoop();
    };
}

#endif // _PICOPTERX_GPS_GPSD_H