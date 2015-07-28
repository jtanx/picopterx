/**
 * @file imu.cpp
 * @brief IMU interaction code.
 */

#include "common.h"
#include "imu_feed.h"

using picopter::FlightBoard;
using picopter::IMU;
using picopter::IMUData;
using namespace std::placeholders;

/**
 * Constructor.
 * Once established, starts the worker thread to receive data from the IMU.
 * @param opts A pointer to options, if any (NULL for defaults)
 * @throws std::invalid_argument if IMU intialisation fails (e.g. disconnected)
 */
IMU::IMU(FlightBoard *fb, Options *opts)
: m_data{NAN,NAN,NAN}
{ 
    fb->RegisterHandler(MAVLINK_MSG_ID_ATTITUDE,
        std::bind(&IMU::ParseInput, this, _1));
}

/**
 * Constructor. Constructs IMU with default options.
 */
IMU::IMU(FlightBoard *fb) : IMU(fb, NULL) {}

/**
 * Destructor. Stops the worker thread and closes the connection to the IMU.
 */
IMU::~IMU() {
}

/**
 * Get the latest IMU data, if available.
 * Unavailable values are indicated with NaN.
 */
void IMU::GetLatest(IMUData *d) {
    std::lock_guard<std::mutex> lock(m_mutex);
    *d = m_data;
}

double IMU::GetLatestRoll() {
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_data.roll;
}

double IMU::GetLatestPitch() {
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_data.pitch;
}

double IMU::GetLatestYaw() {
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_data.yaw;
}

/**
 * Worker thread.
 * Receives data from the IMU and updates the latest information as necessary.
 */
void IMU::ParseInput(const mavlink_message_t *msg) {
    mavlink_attitude_t att;
    mavlink_msg_attitude_decode(msg, &att);
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        m_data.roll = RAD2DEG(att.roll);
        m_data.pitch = RAD2DEG(att.pitch);
        m_data.yaw = RAD2DEG(att.yaw);
    }
}