/**
 * @file lidar.cpp
 * @brief Lidar interaction code.
 * Based on code from:
 * https://github.com/PulsedLight3D/LIDARLite_Basics
 */

#include "common.h"
#include "lidar.h"
#include <wiringPiI2C.h>

#define    LIDARLITE_ADDRESS 0x62 // Default I2C Address of LIDAR-Lite.
#define    MEASURE_REGISTER  0x00 // Register to write to initiate ranging.
#define    MEASURE_VALUE     0x04 // Value to initiate ranging.
#define    READ_REGISTER     0x8f // Register to get both High and Low bytes in 1 call.

using picopter::Lidar;
using std::this_thread::sleep_for;
using std::chrono::milliseconds;

/**
 * Initiates the connection to the LIDAR sensor.
 * @param [in] opts A pointer to options, if any (NULL for defaults)
 * @throws std::invalid_argument If connection fails to the LIDAR.
 */
Lidar::Lidar(Options *opts)
: m_fd(-1)
, m_distance(-1)
{
    m_fd = wiringPiI2CSetup(LIDARLITE_ADDRESS);
    if (m_fd == -1) {
        throw std::invalid_argument("Cannot connect to LIDAR-Lite.");
    }
    m_worker = std::thread(&Lidar::Worker, this);
}

/**
 * Constructor. Shortcut to Lidar(NULL)
 */
Lidar::Lidar() : Lidar(NULL) {}

/**
 * Destructor.
 */
Lidar::~Lidar() {
    m_stop = true;
    m_worker.join();
}

/**
 * Gets the latest distance.
 * @return The distance (negative on error), in cm.
 */
int Lidar::GetLatest() {
    return m_distance;
}

void Lidar::Worker() {
    while (wiringPiI2CWriteReg8(LIDARLITE_ADDRESS, 
        MEASURE_REGISTER, MEASURE_VALUE) < 0 && !m_stop) {
        sleep_for(milliseconds(100));
    }
    
    while (!m_stop) {
        int distance = wiringPiI2CReadReg16(LIDARLITE_ADDRESS, READ_REGISTER);
        if (distance < 0) {
            Log(LOG_DEBUG, "Error reading from LIDAR.");
        } else {
            m_distance = distance; //Need to check endianness
        }
        
        sleep_for(milliseconds(50)); //20Hz update speed
    }
}