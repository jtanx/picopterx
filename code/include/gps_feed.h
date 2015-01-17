/**
 * @file gps_feed.h
 * @brief GPS interaction code.
 */

#ifndef _PICOPTERX_GPS_H
#define _PICOPTERX_GPS_H

#include <cmath>

class gpsmm;

namespace picopter {
    /**
     * Generic structure to hold geographic coordinates.
     * @todo Figure out what is actually needed (moreover what is actually
     *       given from the GPS) and update as necessary. e.g. altitude/climb.
     */
    typedef struct Coordinates {
        /** Latitude, in degrees. For uncertainty: metres **/
        double lat = NAN;
        /** Longitude, in degrees. For uncertainty: metres **/
        double lon = NAN;
    } Coordinates;
    
    /** Holds uncertainty information for a GPS fix. 95% confidence levels. **/
    typedef Coordinates Uncertainty;
    
    /**
     * Stores information about the current GPS fix.
     * @todo Do we need the uncertainty in the timestamp too?
     */
    typedef struct GPSData {
        /** The coordinates of the current GPS fix **/
        Coordinates fix;
        /** The uncertainty in the current fix **/
        Uncertainty err;
        /** The timestamp of the fix (Unix epoch in seconds w/ fractional) **/
        double timestamp = NAN;
    } GPSData;
    
    /**
     * Class that interacts with the GPS.
     */
    class GPS {
        public:
            GPS();
            virtual ~GPS();
            void getLatest(GPSData *d);
            int timeSinceLastFix();
        private:
            static const int CYCLE_TIMEOUT = 500000;
           
            std::atomic<GPSData> m_data;
            std::atomic<int> m_last_fix;
            std::atomic<bool> m_quit;
            std::thread m_worker;
            gpsmm *m_gps_rec;
            
            /** Copy constructor (disabled) **/
            GPS(const GPS &other);
            /** Assignment operator (disabled) **/
            GPS& operator= (const GPS &other);
            void gpsLoop();
    };
}

#endif // _PICOPTERX_GPS_H