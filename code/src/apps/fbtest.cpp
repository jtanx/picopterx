/**
 * @file fbtest.cpp
 * @brief Small utility to test out the flight board.
 */

#include "picopter.h"

using namespace picopter;
using namespace picopter::navigation;
using std::chrono::milliseconds;
using std::this_thread::sleep_for;

std::atomic<bool> shutdown{false};
std::atomic<int> x{0}, y{0}, z{0}, w{0};

void loop(FlightController *fc) {
    while (!shutdown) {
        Vec3D v = {x/100.0*4, y/100.0*4, z/100.0*4};
        fc->fb->SetBodyVel(v);
        if (w != 0) {
            fc->fb->SetYaw(w, false);
        }
        sleep_for(milliseconds(200));
    }
}

int main(int argc, char *argv[]) {
    LogInit();
    FlightController fc;
    char buf[BUFSIZ];
    std::thread t(loop, &fc);
    
    while (fgets(buf, BUFSIZ, stdin)) {
        if (!strcmp(buf, "exit")) {
            break;
        } else {
            int channel, pct;
            if (sscanf(buf, "%d %d", &channel, &pct) == 2) {
                switch(channel) {
                    case 0:
                        x = pct; break;
                    case 1:
                        y = pct; break;
                    case 2:
                        z = pct; break;
                    case 3:
                        w = pct; break;
                }
            }
        }
    }
    shutdown = true;
    t.join();
    
    return 0;
}