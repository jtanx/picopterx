/**
 * @file gps_feed.h
 * @brief GPS interaction code.
 */

#ifndef _PICOPTERX_GPS_H
#define _PICOPTERX_GPS_H

#include "navigation.h"

class gpsmm;

namespace picopter {
    /* Forward declaration of the options class */
    class Options;

    /** Holds uncertainty information for a GPS fix. 95% confidence levels. **/
    typedef navigation::Coord2D Uncertainty;
    
    /**
     * Stores information about the current GPS fix.
     * @todo Do we need the uncertainty in the timestamp too? 2D info enough?
     */
    typedef struct GPSData {
        /** The coordinates of the current GPS fix (in radians) **/
        navigation::Coord2D fix;
        /** The uncertainty in the current fix (in metres) **/
        Uncertainty err;
        /** The timestamp of the fix (Unix epoch in seconds w/ fractional) **/
        double timestamp;
    } GPSData;
    
    /**
     * Class that interacts with the GPS.
     */
    class GPS {
        public:
            GPS();
            GPS(Options *opts);
            virtual ~GPS();
            void GetLatest(GPSData *d);
            
            int TimeSinceLastFix();
        private:
            /** The GPS read wait timeout (in us) **/
            static const int CYCLE_TIMEOUT_DEFAULT = 500000;
            int m_cycle_timeout;
           
            std::atomic<GPSData> m_data;
            std::atomic<int> m_last_fix;
            std::atomic<bool> m_quit;
            std::thread m_worker;
            gpsmm *m_gps_rec;
            
            /** Copy constructor (disabled) **/
            GPS(const GPS &other);
            /** Assignment operator (disabled) **/
            GPS& operator= (const GPS &other);
            void GPSLoop();
    };
}

#endif // _PICOPTERX_GPS_H