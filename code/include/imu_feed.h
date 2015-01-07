/**
 * @file imu_feed.h
 * @brief Defines the IMU class.
 */

#ifndef _IMU_H
#define _IMU_H

//Forward declaration of Cmt3 from XSens
namespace xsens {class Cmt3;}

namespace picopter {
    /**
     * Contains a unit of information from the IMU.
     */
    typedef struct IMUData {
        /** Roll of the IMU, -pi to pi **/
        double roll = NAN;
        /** Pitch of the IMU, -pi to pi **/
        double pitch = NAN;
        /** Yaw of the IMU, -pi to pi **/
        double yaw = NAN;
    } IMUData;

    /**
     * Reads data from the IMU
     */
    class IMU {
        public:
            IMU();
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
            
            void imuLoop();
    };
}

#endif //_IMU_H