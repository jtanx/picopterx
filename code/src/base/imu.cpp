/**
 * @file imu.cpp
 * @brief IMU interaction code.
 */

#include "common.h"
#include "imu_feed.h"

using picopter::IMU;
using picopter::IMUData;

/**
 * Constructor.
 * Once established, starts the worker thread to receive data from the IMU.
 * @param opts A pointer to options, if any (NULL for defaults)
 * @throws std::invalid_argument if IMU intialisation fails (e.g. disconnected)
 */
IMU::IMU(Options *opts)
: m_data{NAN,NAN,NAN}
, m_quit(false)
{ 
    m_worker = std::thread(&IMU::IMULoop, this);
}

/**
 * Constructor. Constructs IMU with default options.
 */
IMU::IMU() : IMU(NULL) {}

/**
 * Destructor. Stops the worker thread and closes the connection to the IMU.
 */
IMU::~IMU() {
    m_quit = true;
    m_worker.join();
}

/**
 * Get the latest IMU data, if available.
 * Unavailable values are indicated with NaN.
 */
void IMU::GetLatest(IMUData *d) {
    std::lock_guard<std::mutex> lock(m_mutex);
    *d = m_data;
    d->roll = d->roll;
    d->pitch = d->pitch;
    d->yaw = d->yaw;
}

/**
 * Worker thread.
 * Receives data from the IMU and updates the latest information as necessary.
 */
void IMU::IMULoop() {
    while (!m_quit) {
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }
}