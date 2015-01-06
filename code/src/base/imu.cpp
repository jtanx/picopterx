/**
 * @file imu.cpp
 * @brief IMU interaction code.
 */

#include "picopter.h"
#include "cmt3.h"

using namespace xsens;
using picopter::IMU;
using picopter::IMUData;

const char *IMU::IMU_DEVICE = "/dev/ttyUSB0";

IMU::IMU()
: m_data{}
, m_quit(false)
{
    m_device = new Cmt3();
    if (m_device->openPort(IMU_DEVICE) != XRV_OK) {
        throw std::invalid_argument("Could not open IMU stream");
    } else if (m_device->setTimeoutMeasurement(IMU_TIMEOUT) != XRV_OK) {
        throw std::invalid_argument("Could not set IMU measurement timeout");
    }
    
    CmtDeviceMode mode(CMT_OUTPUTMODE_ORIENT, 
                       CMT_OUTPUTSETTINGS_TIMESTAMP_SAMPLECNT | 
                       CMT_OUTPUTSETTINGS_ORIENTMODE_EULER, 
                       10);
    if(m_device->setDeviceMode(mode, false, CMT_DID_BROADCAST)  != XRV_OK) {
        throw std::invalid_argument("Could not set the IMU device mode");
    } else if (m_device->gotoMeasurement() != XRV_OK) {
        throw std::invalid_argument("Could not begin IMU measurement");
    }
    
    m_worker = std::thread(&IMU::imuLoop, this);
}

IMU::~IMU() {
    m_quit = true;
    m_worker.join();
    m_device->closePort();
    delete m_device;
}

void IMU::getLatest(IMUData *d) {
    std::lock_guard<std::mutex> lock(m_mutex);
    *d = m_data;
}

void IMU::imuLoop() {
    Packet *msg = new Packet(1, 0);
    
    while (!m_quit) {
        if (m_device->waitForDataMessage(msg) != XRV_OK) {
            Log(LOG_WARNING, "Could not get IMU data");
        } else {
            CmtEuler angles = msg->getOriEuler();
            std::lock_guard<std::mutex> lock(m_mutex);
            
            m_data.pitch = angles.m_pitch;
            m_data.roll = angles.m_roll;
            m_data.yaw = -angles.m_yaw;
            Log(LOG_INFO, "Got IMU data: <%.2f, %.2f, %.2f>",
                m_data.pitch, m_data.roll, m_data.yaw);
        }
    }
    
    delete msg;
}