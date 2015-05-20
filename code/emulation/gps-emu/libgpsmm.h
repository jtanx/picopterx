/**
 * @file libgpsmm.h
 * @brief A *very thin* emulation of libgpsmm.
 * Will always return that there is data, but with all values zeroed.
 * Data will randomly be marked as set or unset.
 */

#ifndef _LIBGPSMM_STUB_H
#define _LIBGPSMM_STUB_H

#include <config.h>

#ifdef IS_REAL_LIBGPS
#error The order of directory inclusion is broken. Do not include the folder containing this file when you have libgps installed.
#endif

#include <thread>
#include <chrono>
#include <cstdlib>

#define DEFAULT_GPSD_PORT ""
#define WATCH_ENABLE 1
#define WATCH_JSON 1

#define ONLINE_SET	(1llu<<1)
#define TIME_SET	(1llu<<2)
#define TIMERR_SET	(1llu<<3)
#define LATLON_SET	(1llu<<4)
#define ALTITUDE_SET	(1llu<<5)
#define SPEED_SET	(1llu<<6)
#define TRACK_SET	(1llu<<7)
#define CLIMB_SET	(1llu<<8)
#define STATUS_SET	(1llu<<9)
#define MODE_SET	(1llu<<10)
#define DOP_SET  	(1llu<<11)
#define HERR_SET	(1llu<<12)
#define VERR_SET	(1llu<<13)
#define ATTITUDE_SET	(1llu<<14)
#define SATELLITE_SET	(1llu<<15)
#define SPEEDERR_SET	(1llu<<16)
#define TRACKERR_SET	(1llu<<17)
#define CLIMBERR_SET	(1llu<<18)
#define DEVICE_SET	(1llu<<19)
#define DEVICELIST_SET	(1llu<<20)
#define DEVICEID_SET	(1llu<<21)
#define RTCM2_SET	(1llu<<22)
#define RTCM3_SET	(1llu<<23)
#define AIS_SET 	(1llu<<24)
#define PACKET_SET	(1llu<<25)
#define SUBFRAME_SET	(1llu<<26)
#define GST_SET 	(1llu<<27)
#define VERSION_SET	(1llu<<28)
#define POLICY_SET	(1llu<<29)
#define LOGMESSAGE_SET	(1llu<<30)
#define ERROR_SET	(1llu<<31)
#define TIMEDRIFT_SET	(1llu<<32)
#define EOF_SET		(1llu<<33)

struct gps_data_t {
    struct fix {
        double time;
        double latitude;
        double epy;
        double longitude;
        double epx;
        double speed;
        double eps;
        double track;
        double epd;
        double altitude;
        double epv;
    } fix;
    int set;
    double satellites_used;
};

class gpsmm {
    public:
        gpsmm(const char *host, const char *port) : m_dat{{},LATLON_SET,6} {
            srand(time(NULL));
        };
        virtual ~gpsmm() {};
        void *stream(int a) { return &m_dat;};
        bool waiting(int timeout) { 
            std::this_thread::sleep_for(std::chrono::microseconds(timeout));
            m_dat.set ^= rand() % 2; 
            return true;
        };
        struct gps_data_t * read() {return &m_dat;};
    private:
        struct gps_data_t m_dat;
};

#endif // _LIBGPSMM_STUB_H
