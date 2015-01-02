/**
 * @file buzzer.h
 * @brief Methods to control the buzzer
 */

#ifndef _BUZZER_H
#define _BUZZER_H

namespace picopter {
    class Buzzer {
        public:
            Buzzer();
            virtual ~Buzzer();
            void play(int duration, int frequency, int volume);
            void stop();
        private:
            /** Internal mutex to interact with the worker thread **/
            static std::mutex g_buzzer_mutex;
            /** Condition to signal the worker thread to wake up **/
            std::condition_variable m_signaller;
            /** Indicates if the worker thread is currently playing a sound **/
            std::atomic<bool> m_running;
            /** Indicates that the thread should quit **/
            std::atomic<bool> m_stop;
            /** Indicates that the thread should stop playing any sound **/
            std::atomic<bool> m_quiet;
            /** The worker thread **/
            std::thread m_worker;
            /** The number of cycles to be played **/
            int m_count;
            /** The time of one period (cycle), in microseconds **/
            int m_period;
            /** The time per period that is spent ON **/
            int m_dutyCycle;
            
            void soundLoop();
    };
}

#endif // _BUZZER_H