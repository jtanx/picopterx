/**
 * @file buzzer.cpp
 * @brief Buzzer manipulation code.
 * Uses wiringPi and a GPIO pin for driving the buzzer with software based PWM.
 */

#include "common.h"
#include "gpio.h"
#include "buzzer.h"
#include <wiringPi.h>
 
using picopter::Buzzer;
using picopter::gpio::SetBuzzer;
using hrc = std::chrono::high_resolution_clock;

/* 
 * We use a single mutex for all buzzer instances because
 * only one of them may be playing a sound through the buzzer at any
 * one time.
 */
std::mutex Buzzer::g_buzzer_mutex;
 
/**
 * Creates a new Buzzer instance.
 * Starts a worker thread that will play the sounds
 */
Buzzer::Buzzer()
: m_running(false)
, m_stop(false)
, m_quiet(false)
{
    picopter::gpio::Init();
    m_worker = std::thread(&Buzzer::SoundLoop, this);
}

/**
 * Destructor method. Stops the worker thread.
 */
Buzzer::~Buzzer() {
    m_quiet = true;
    m_stop = true;
    
    //Log(LOG_INFO, "DESTROYING");
    m_signaller.notify_one();
    //Log(LOG_INFO, "NOTIFIED");
    m_worker.join();
    //Log(LOG_INFO, "DESTROYED");
}

/**
 * The function that does the software PWM and actuates the GPIO sound pin.
 * @param blocking true iff this function is used with the playWait method.
 */
void Buzzer::SoundOutput(bool blocking) {
    //Log(LOG_INFO, "Playing the sound! Count: %d, DS: %d, P: %d", m_count, m_dutyCycle, m_period);
    //auto start = hrc::now();
    for (int n = 0; (blocking || m_running) && !m_stop && !m_quiet && n < m_count; n++) {
        SetBuzzer(HIGH);
        delayMicroseconds(m_dutyCycle);
        SetBuzzer(LOW);
        delayMicroseconds(m_period - m_dutyCycle);
    }
    //std::chrono::duration<double> elapsed = hrc::now() - start;
   // Log(LOG_INFO, "Play time: %lf", elapsed.count());
}

/**
 * Worker thread that controls the buzzer pin.
 * Blocks until it is signalled to either exit or play a sound.
 */
void Buzzer::SoundLoop() {
    std::unique_lock<std::mutex> lock(g_buzzer_mutex);
    
    while (!m_stop) {
        m_signaller.wait(lock, [this]{return m_running || m_stop;});
        if (!m_stop) {
            Buzzer::SoundOutput(false);
        }
        m_running = false;
    }
}

/**
 * Plays a tone, non-blocking.
 * If a tone is already being played, then that is stopped and the new tone
 * is played instead.
 * @param duration The length of time to play the sound, in ms
 * @param frequency The frequency of the sound, in Hz (10-5000Hz).
 * @param volume The loudness of the sound, as a percentage (0 - 100%)
 */
void Buzzer::Play(int duration, int frequency, int volume) {
    m_quiet = true;
    std::unique_lock<std::mutex> lock(g_buzzer_mutex);
    
    m_period = 1000000 / picopter::clamp(frequency, 10, 5000);
    m_dutyCycle = (m_period * picopter::clamp(volume, 0, 100)) / 200;
    m_count = (1000 * std::max(duration, 0)) / m_period;
    
    m_quiet = false;
    m_running = true;
    lock.unlock();
    m_signaller.notify_one();
}

/**
 * Plays a tone, blocking until it is complete.
 * If a tone is already being played, then that is stopped and the new tone
 * is played instead.
 * @param duration The length of time to play the sound, in ms
 * @param frequency The frequency of the sound, in Hz (10-5000Hz).
 * @param volume The loudness of the sound, as a percentage (0 - 100%)
 */
void Buzzer::PlayWait(int duration, int frequency, int volume) {
    m_quiet = true;
    std::lock_guard<std::mutex> lock(g_buzzer_mutex);
    
    m_period = 1000000 / picopter::clamp(frequency, 10, 5000);
    m_dutyCycle = (m_period * picopter::clamp(volume, 0, 100)) / 200;
    m_count = (1000 * std::max(duration, 0)) / m_period;
    
    m_quiet = false;
    Buzzer::SoundOutput(true);
}

/**
 * Signals the buzzer to stop if it is running
 */
void Buzzer::Stop() {
    m_quiet = true;
}

 
