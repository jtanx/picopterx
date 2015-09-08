/**
 * @file fbtest.cpp
 * @brief Small utility to test out the flight board.
 */

#include "picopter.h"

using namespace picopter;
using namespace picopter::navigation;

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
                        fc.fb->SetBodyVel(Vec3D{pct/100.0*4, 0, 0}); break;
                    case 1:
                        fc.fb->SetBodyVel(Vec3D{0, pct/100.0*4, 0});break;
                    case 2:
                        fc.fb->SetBodyVel(Vec3D{0, 0, pct/100.0*4});break;
                    case 3:
                        fc.fb->SetYaw(pct*20/100, true); break;
                }
            }
        }
    }
    
    return 0;
}