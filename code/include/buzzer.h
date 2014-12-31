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
            void play(double duration, double frequency, double volume);
            void stop();
        private:
            static const int BUZZER_PIN = 2;
            static const int SLEEP_PERIOD = 5;
            std::mutex mutex;
            std::atomic<bool> running;
            std::atomic<bool> stopWorker;
            std::thread worker;
            
            int quantize(int value);
            void soundLoop(int duration, int period, int dutyCycle);
    };
}

#endif // _BUZZER_H