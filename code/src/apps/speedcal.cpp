/**
 * @file speedcal.cpp
 * @brief Application to determine relation between input and speed
 */

#include "picopter.h"
#include <signal.h>

using namespace picopter;

static volatile bool stop = false;

static void handler(int signum) {
    printf("\nQuit received.");
    stop = true;
}

int main(int argc, char *argv[]) {
    LogInit();
    
    FlightController fc;
    char spinners[] = {'-','\\','|','/'};
    
    signal(SIGINT, handler);
    printf("Waiting for GPS Authentication...\n");
    while (!fc.gps->WaitForFix()) {
        printf("Still waiting...\n");
    }
    
    for (int speed = 10; speed <= 100 && !stop; speed += 10) {
        printf("Will run at speed %d, waiting for authorisation...\n", speed);
        while (!fc.WaitForAuth()) {
            //Do nothing;
        }
        printf("Got auth, moving forward at speed %d\n", speed);
        fc.fb->SetElevator(speed);
        for (int i = 0; !fc.CheckForStop() && !stop; i = (i+1)%4) {
            GPSData d;
            fc.gps->GetLatest(&d);
            printf("[%c] Speed: %3d, GroundSpeed: %6.2f m/s\r", spinners[i], speed, d.fix.speed);
            fflush(stdout);
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
        }
        printf("\nAuth revoked, stopping...\n");
        fc.fb->Stop();
    }
    printf("Finished.\n");
}