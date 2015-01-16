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
    const char *t = "AAA";
    
    o.Set("test21142", t);
    o.Set("test", "JAJAJA");
    o.Set("TEST", "JAJAJA");
    o.Set("TEST2", 1.233);
    o.Set("TEST3", true);
    o.Set("TEST3", 20);
    o.Save();
    o.SetFamily("GPS");
    std::cout << o.GetInt("NO") << std::endl;
    o.Set("NO", 20);
    std::cout << o.GetInt("NO") << std::endl;
    o.Set("NO", "JJAA");
    std::cout << o.GetInt("NO") << std::endl;
    std::cout << o.GetString("NO") << std::endl;
    std::cout << o.Remove("NO") << std::endl;
    std::cout << o.Remove("NO") << std::endl;
    
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