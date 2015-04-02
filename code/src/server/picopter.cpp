/**
 * @file picopter.cpp
 * @brief The main entry point to the server.
 */
 
#include "picopter.h"
 
int main(int argc, char *argv[]) {
    LogInit();
    
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
    return 0;
}
