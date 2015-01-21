/**
 * @file lawnmower.h
 * @brief Defines the lawnmower controls.
 */

#ifndef _PICOPTERX_LAWNMOWER_H
#define _PICOPTERX_LAWNMOWER_H

namespace picopter {
    /**
     * Class to fly the hexacopter in a lawnmower pattern.
     */
    class Lawnmower {
        public:
            Lawnmower(FlightController &fc, Options *opts);
            void Run(Coordinates p1, Coordinates p2);
        private:
    };
}

#endif // _PICOPTERX_LAWNMOWER_H