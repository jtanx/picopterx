/**
 * @file buzzer.cpp
 * @brief Buzzer manipulation code.
 * Uses wiringPi to control the GPIO pins on the rPi.
 * PWM (through the GPIO pins) is done through ServoBlaster.
 */

#include "picopter.h" 
#include <wiringPi.h>
 
using picopter::Buzzer;

using hrc = std::chrono::high_resolution_clock;
 
/**
 * Creates a new Buzzer instance.
 * Starts a worker thread that will play the sounds
 */
Buzzer::Buzzer()
: m_running(false)
, m_stop(false)
, m_quiet(false)
{
    wiringPiSetup();
    pinMode(BUZZER_PIN, OUTPUT);
    
    m_worker = std::thread(&Buzzer::soundLoop, this);
}

/**
 * Destructor method. Stops the worker thread.
 */
Buzzer::~Buzzer() {
    m_quiet = true;
    m_stop = true;
    
    m_signaller.notify_one();
    m_worker.join();
}

/**
 * Worker thread that controls the buzzer pin.
 * Blocks until it is signalled to either exit or play a sound.
 */
void Buzzer::soundLoop() {
    std::unique_lock<std::mutex> lock(m_mutex);
    
    while (!m_stop) {
        m_signaller.wait(lock, [this]{return m_running || m_stop;});
        Log(LOG_INFO, "Playing the sound! Count: %d, DS: %d, P: %d", m_count, m_dutyCycle, m_period);
        
        auto start = hrc::now();
        for (int n = 0; m_running && !m_stop && !m_quiet && n < m_count; n++) {
            digitalWrite(MODE_PIN, HIGH);
            delayMicroseconds(m_dutyCycle);
            digitalWrite(MODE_PIN, LOW);
            delayMicroseconds(m_period - m_dutyCycle);
        }
        
        std::chrono::duration<double> elapsed = hrc::now() - start;
        Log(LOG_INFO, "Play time: %lf", elapsed.count());
        
        digitalWrite(MODE_PIN, LOW);
        m_running = false;
        m_signaller.notify_one();
    }
}

/**
 * Plays a tone, non-blocking.
 * If a tone is already being played, then that is stopped and the new tone
 * is played instead.
 * @param duration The length of time to play the sound, in ms
 * @param frequency The frequency of the sound, in Hz (max 5000Hz)
 * @param volume The loudness of the sound, as a percentage (0 - 100%)
 */
void Buzzer::play(int duration, int frequency, int volume) {
    m_quiet = true;
    
    std::unique_lock<std::mutex> lock(m_mutex);
    m_period = 1000000 / picopter::clamp(frequency, 0, 5000);
    m_dutyCycle = (m_period * picopter::clamp(volume, 0, 100)) / 200;
    m_count = (1000 * duration) / m_period;
    
    m_quiet = false;
    m_running = true;
    lock.unlock();
    m_signaller.notify_one();
}

/**
 * Signals the buzzer to stop if it is running
 */
void Buzzer::stop() {
    m_quiet = true;
}

 