/**
 * @file buzzer.cpp
 * @brief Buzzer manipulation code.
 * Uses wiringPi to control the GPIO pins on the rPi.
 * PWM (through the GPIO pins) is done through ServoBlaster.
 */

#include "picopter.h" 
#include <wiringPi.h>
 
using namespace picopter;
 
Buzzer::Buzzer() : running(false), stopWorker(false) {
    wiringPiSetup();
    pinMode(BUZZER_PIN, OUTPUT);
}

int Buzzer::quantize(int value) {
    return (value / SLEEP_PERIOD) * SLEEP_PERIOD;
}

void Buzzer::soundLoop(int duration, int period, int dutyCycle) {
    int N = duration / period;
    int M = period / SLEEP_PERIOD;
    
    for(int n = 0; n < N && !stopWorker; n++) {
        for(int m = 0; m < M && !stopWorker; m++) {
            digitalWrite(BUZZER_PIN, (m < (dutyCycle / SLEEP_PERIOD)));
            std::this_thread::sleep_for(std::chrono::milliseconds(SLEEP_PERIOD));
        }
    }
    
    digitalWrite(BUZZER_PIN, LOW);
    running = false;
}
 
void Buzzer::play(double duration, double frequency, double volume) {
    std::lock_guard<std::mutex> lock(mutex);
    int durationQ = quantize((int)(duration * 1000));
	int period = quantize((int)(1000 / frequency));
	int dutyCycle = quantize(period * volume / 100);
    
    if (running) {
        stopWorker = true;
        worker.join();
    }
    
    running = true;
    stopWorker = false;
    worker = std::thread(&Buzzer::soundLoop, this, durationQ, period, dutyCycle);
}

void Buzzer::stop() {
    std::lock_guard<std::mutex> lock(mutex);
    
    if (running) {
        stopWorker = true;
        worker.join();
    }
    
    running = false;
    stopWorker  = false;
}
 
 