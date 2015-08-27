/**
 * @file gps_feed.h
 * @brief Base GPS header.
 */

#ifndef _PICOPTERX_GPS_H
#define _PICOPTERX_GPS_H

/* For the Options class */
#include "opts.h"
#include "navigation.h"

class gpsmm;

namespace picopter {
    typedef struct GPSFix {
        /** The latitude, in radians. Uncertainty in metres. **/
        double lat;
        /** The longitude, in radians. Uncertainty in metres. **/
        double lon;
        /** The GPS altitude, in metres. Uncertainty in metres.**/
        double alt;
        /** The estimated altitude of the ground, in metres. Uncertainty in metres. **/
        double groundalt;
        /** The speed, in m/s. Uncertainty in m/s. **/
        double speed;
        /** The heading (track angle), in radians. Uncertainty in radians. **/
        double heading;
        /** The magnetic bearing, in radians, if available. **/
        double bearing;
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
            virtual void GetLatest(GPSData *d);
            virtual double GetLatestRelAlt();
            
            int TimeSinceLastFix();
            bool HasFix();
            bool WaitForFix(int timeout=-1);
        protected:
            /** The GPS fix timeout (in s) **/
            static const int FIX_TIMEOUT_DEFAULT = 2;
            /** The time checks waits when waiting for a fix (in ms) **/
            static const int WAIT_PERIOD = 200;
            
            int m_fix_timeout;
            std::mutex m_worker_mutex;
            GPSData m_data;
            std::atomic<int> m_last_fix;
            std::atomic<bool> m_quit;
        private:
            /** Copy constructor (disabled) **/
            GPS(const GPS &other);
            /** Assignment operator (disabled) **/
            GPS& operator= (const GPS &other);
    };
}

#endif // _PICOPTERX_GPS_H
