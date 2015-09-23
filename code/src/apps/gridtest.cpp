/**
 * @file gridtest.cpp
 * @brief voxel world testing code
 */

#include "picopter.h"
#include "gridspace.h"
using namespace picopter;
using namespace picopter::navigation;
using namespace std;

int main(int argc, char *argv[]) {
    //FlightController fc;
	GridSpace world;
	world.printToConsole(0, 8, 0);
	cout << "\n";
	world.raycast();world.raycast();
	world.printToConsole(0, 8, 0);
	return 0;
}
