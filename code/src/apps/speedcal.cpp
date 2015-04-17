/**
 * @file speedcal.cpp
 * @brief Application to determine relation between input and speed
 */

#include "picopter.h"

using namespace picopter;

int main(int argc, char *argv[]) {
    LogInit();
    
    FlightController fc;
    char spinners[] = {'-','\\','|','/'};
    
    for (int speed = 10; speed <= 100; speed += 10) {
        printf("Will run at speed %d, waiting for authorisation...\n", speed);
        while (!fc.WaitForAuth()) {
            //Do nothing;
        }
        printf("Got auth, moving forward at speed %d\n", speed);
        fc.fb->SetElevator(speed);
        for (int i = 0; !fc.CheckForStop(); i = (i+1)%4) {
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