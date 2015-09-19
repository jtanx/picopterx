/**
 * @file watchdog.cpp
 * @brief Watchdog routines.
 */

#include "common.h"
#include "watchdog.h"

using picopter::Watchdog;
using std::this_thread::sleep_for;
using std::chrono::milliseconds;

/**
 * Constructor. Creates a new watchdog.
 * @param [in] timeout The timeout in milliseconds.
 * @param [in] cb The callback to call if it times out.
 */
Watchdog::Watchdog(int timeout, std::function<void()> cb)
: m_stop{false}
, m_index{0}
, m_timeout(timeout)
, m_callback(cb)
{
}

/**
 * Destructor.
 */
Watchdog::~Watchdog() {
    Stop();
}

/**
 * Starts the watchdog. Not threadsafe.
 */
void Watchdog::Start() {
    if (!m_stop && !m_worker.joinable()) {
        m_worker = std::thread(&Watchdog::Worker, this);
    }
}

/**
 * Stops the watchdog.
 */
void Watchdog::Stop() {
    m_stop = true;
    if (m_worker.joinable()) {
        m_worker.join();
    }    
}

/**
 * Watchdog loop.
 */
void Watchdog::Worker() {
    int last_index = 0;
    while (true) {
        sleep_for(milliseconds(m_timeout));
        if (m_stop) {
            break;
        } else {
            int cur_index = m_index;
            if (cur_index <= last_index) {
                m_callback();
            }
            last_index = cur_index;
        }
    }
}

/**
 * Resets the watchdog timer.
 */
void Watchdog::Touch() {
    m_index++;
}