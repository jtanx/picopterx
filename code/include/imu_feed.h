/**
 * @file imu_feed.h
 * @brief Defines the IMU class.
 */

#ifndef _PICOPTERX_IMU_H
#define _PICOPTERX_IMU_H

#include "navigation.h"
#include "flightboard.h"

namespace picopter {
    /* Forward declaration of the options class */
    class Options;
    
    /**
     * Contains a unit of information from the IMU.
     */
    typedef navigation::EulerAngle IMUData;

    /**
     * Reads data from the IMU
     */
    class IMU {
        public:
            IMU(FlightBoard *fb);
            IMU(FlightBoard *fb, Options *opts);
            virtual ~IMU();
            void GetLatest(IMUData *d);
            
        private:
            /** Read timeout from the IMU in ms **/
            static const int IMU_TIMEOUT = 500;
            IMUData m_data;
            
            std::atomic<bool> m_quit;
            std::mutex m_mutex;
            std::thread m_worker;
            
            /** Copy constructor (disabled) **/
            IMU(const IMU &other);
            /** Assignment operator (disabled) **/
            IMU& operator= (const IMU &other);
            void ParseInput(const mavlink_message_t *msg);
    };
}

#endif // _PICOPTERX_IMU_H