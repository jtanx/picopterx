/**
 * @file imu_feed.h
 * @brief Defines the IMU class.
 */

#ifndef _PICOPTERX_IMU_H
#define _PICOPTERX_IMU_H

#include "navigation.h"

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
            IMU();
            IMU(Options *opts);
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
            void IMULoop();
    };
}

#endif // _PICOPTERX_IMU_H