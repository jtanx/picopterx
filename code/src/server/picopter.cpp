/**
 * @file picopter.cpp
 * @brief The main entry point to the server.
 */
 
#include "picopter.h"
 
int main(int argc, char *argv[]) {
    LogInit();
    Log(LOG_NOTICE, "BUZZER TEST STARTED");
    picopter::Buzzer b;
    picopter::GPS gps;
    picopter::Options o;
    
    //o.Store("test", "JAJAJA");
    o.Save();
    o.SetFamily("GPS");
    o.Save();
    /*
    
    b.play(1000, 200, 80);
    std::this_thread::sleep_for(std::chrono::milliseconds(400));
    b.play(1000, 100, 80);
    std::this_thread::sleep_for(std::chrono::milliseconds(800));
    b.play(1000, 1200, 100);
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));
    
    if (argc > 1 && !strcmp(argv[1], "--loop")) {
        while(true) {
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }
    }
    
    std::cout << gps.timeSinceLastFix();*/
    return 0;
}