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
#define    READ_HIGH         0x0f // Register to get the high byte.
#define    READ_LOW          0x10 // Register to get the low byte.
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
, m_log("lidar")
, m_stop{false}
{
    m_fd = wiringPiI2CSetup(LIDARLITE_ADDRESS);
    if (m_fd == -1) {
        throw std::invalid_argument("Cannot connect to LIDAR-Lite.");
    }
    m_worker = std::thread(&Lidar::Worker, this);
    Log(LOG_INFO, "LIDAR intialised!");
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
    int counter = 0;
    while (!m_stop) {
        while (wiringPiI2CWriteReg8(m_fd, 
            MEASURE_REGISTER, MEASURE_VALUE) < 0 && !m_stop) {
            sleep_for(milliseconds(100));
            //Log(LOG_DEBUG, "WAITING FOR LIDARLITE");
        }

        int high = wiringPiI2CReadReg8(m_fd, READ_HIGH);
        int low = wiringPiI2CReadReg8(m_fd, READ_LOW);
        if (high < 0 || low < 0) {
            Log(LOG_DEBUG, "Error reading from LIDAR.");
        } else {
            low |= (high<<8);
            m_distance = low;
            if ((++counter % 20) == 0) { //Restrict log to ~1Hz.
                m_log.Write(": %d", low);
            }
            //Log(LOG_DEBUG, "DIST: %d", low));
        }
        
        sleep_for(milliseconds(50)); //20Hz update speed
    }
}
