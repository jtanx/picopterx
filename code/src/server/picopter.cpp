/**
 * @file picopter.cpp
 * @brief The main entry point to the server.
 */
 
#include "picopter.h"
 
int main(int argc, char *argv[]) {
    LogInit();
    /*Log(LOG_NOTICE, "BUZZER TEST STARTED");
    picopter::Buzzer b;
    //picopter::GPS gps;
    picopter::Options o;
    const char *t = "AAA";
    
    o.Set("test21142", t);
    o.Set("test", "JAJAJA");
    o.Set("TEST", "JAJAJA");
    o.Set("TEST2", 1.233);
    o.Set("TEST3", true);
    o.Set("TEST3", 20);
    o.SetFamily("GPS");
    std::cout << o.GetInt("NO") << std::endl;
    o.Set("NO", 20);
    std::cout << o.GetInt("NO") << std::endl;
    o.Set("NO", "JJAA");
    std::cout << o.GetInt("NO") << std::endl;
    std::cout << o.GetString("NO") << std::endl;
    std::cout << o.Remove("NO") << std::endl;
    std::cout << o.Remove("NO") << std::endl;
    
    o.Save(stdout);
    b.Play(800,200,80);
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    b.PlayWait(1000,200,80);
    b.Play(1000,200,80);
    */
    std::unique_ptr<picopter::FlightController> fc;
    try {
        fc.reset(new picopter::FlightController());
    } catch (const std::invalid_argument &e) {
        //Fatal("Failed to initialise %s which is required, exiting.", e.what());
        Log(LOG_ERR, "Failed to initialise %s which is required, exiting (not really).", e.what());
    }
    
    if (fc) {
        fc->buzzer->Play(10,400,100);
        Log(LOG_INFO, "Flight controller started.");
        char buf[BUFSIZ];
        picopter::FlightData fd = {0};
        //picopter::Waypoints wp;
        
        while (fgets(buf, BUFSIZ, stdin) && strcmp(buf, "exit")) {
            int n = sscanf(buf, "%i,%i,%i,%i", &fd.elevator,&fd.aileron, &fd.rudder, &fd.gimbal);
            printf("UD:%d, LR: %d, RT: %d, GB: %d\n", fd.elevator, fd.aileron, fd.rudder, fd.gimbal);
            fc->fb->SetData(&fd);
        }
    }
    
    //picopter::navigation::Coord3D p = {2,3,4};
    //picopter::navigation::Coord2D e = p;
    
    //Log(LOG_INFO, "3D: (%.1f,%.1f,%.1f), 2D: (%.1f, %.1f)", p.lat, p.lon, p.alt, e.lat, e.lon);
    
    /*
    
    b.Play(1000, 200, 80);
    std::this_thread::sleep_for(std::chrono::milliseconds(400));
    b.Play(1000, 100, 80);
    std::this_thread::sleep_for(std::chrono::milliseconds(800));
    b.Play(1000, 1200, 100);
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));
    
    if (argc > 1 && !strcmp(argv[1], "--loop")) {
        while(true) {
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }
    }
    
    std::cout << gps.TimeSinceLastFix();*/
    return 0;
}
