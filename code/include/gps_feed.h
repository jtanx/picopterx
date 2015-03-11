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
    
    typedef struct GPSFix {
        /** The latitude, in radians. Uncertainty in metres. **/
        double lat;
        /** The longitude, in radians. Uncertainty in metres. **/
        double lon;
        /** The speed, in m/s. Uncertainty in m/s. **/
        double speed;
        /** The heading (track angle), in radians. Uncertainty in radians. **/
        double heading;
    } GPSFix;
    
    /** Holds uncertainty information for a GPS fix. 95% confidence levels. **/
    typedef GPSFix Uncertainty;
    
    /**
     * Stores information about the current GPS fix.
     * @todo Do we need the uncertainty in the timestamp too? 2D info enough?
     */
    typedef struct GPSData {
        /** The coordinates of the current GPS fix **/
        GPSFix fix;
        /** The uncertainty in the current fix **/
        Uncertainty err;
        /** The timestamp of the fix (Unix epoch in seconds w/ fractional) **/
        double timestamp;
        
        operator navigation::Coord2D() {
            navigation::Coord2D ret{fix.lat, fix.lon};
            return ret;
        };
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
            bool HasFix();
            bool WaitForFix(int timeout=-1);
        private:
            /** The GPS read wait timeout (in us) **/
            static const int CYCLE_TIMEOUT_DEFAULT = 500000;
            /** The GPS fix timeout (in s) **/
            static const int FIX_TIMEOUT_DEFAULT = 2;
            /** The time checks waits when waiting for a fix (in ms) **/
            static const int WAIT_PERIOD = 200;
            int m_cycle_timeout;
            int m_fix_timeout;
            bool m_had_fix;
            
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