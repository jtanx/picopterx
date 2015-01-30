/**
 * @file imu_feed.h
 * @brief Defines the IMU class.
 */

#ifndef _PICOPTERX_IMU_H
#define _PICOPTERX_IMU_H

#include "navigation.h"

//Forward declaration of Cmt3 from XSens
namespace xsens {class Cmt3;}

namespace picopter {
    /* Forward declaration of the options class */
    class Options;
    
    /**
     * Contains a unit of information from the IMU.
     */
    typedef EulerAngle IMUData;

    /**
     * Reads data from the IMU
     */
    class IMU {
        public:
            IMU();
            IMU(Options *opts);
            virtual ~IMU();
            void getLatest(IMUData *d);
            
        private:
            /** Path to the IMU device; e.g. /dev/ttyUSB0 **/
            static const char *IMU_DEVICE;
            /** Read timeout from the IMU in ms **/
            static const int IMU_TIMEOUT = 500;
            IMUData m_data;
            
            std::atomic<bool> m_quit;
            std::mutex m_mutex;
            std::thread m_worker;
            
            xsens::Cmt3 *m_device;
            
            /** Copy constructor (disabled) **/
            IMU(const IMU &other);
            /** Assignment operator (disabled) **/
            IMU& operator= (const IMU &other);
            void imuLoop();
    };
}

#endif // _PICOPTERX_IMU_H