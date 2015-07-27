/**
 * @file fbtest.cpp
 * @brief Small utility to test out the flight board.
 */

#include "picopter.h"

using namespace picopter;

int main(int argc, char *argv[]) {
    LogInit();
    FlightController fc;
    char buf[BUFSIZ];
    
    while (fgets(buf, BUFSIZ, stdin)) {
        if (!strcmp(buf, "exit")) {
            break;
        } else {
            int channel, pct;
            if (sscanf(buf, "%d %d", &channel, &pct) == 2) {
                switch(channel) {
                    case 0:
                        fc.fb->SetAileron(pct); break;
                    case 1:
                        fc.fb->SetElevator(pct); break;
                    case 2:
                        fc.fb->SetRudder(pct); break;
                }
            }
        }
    }
    
    return 0;
}