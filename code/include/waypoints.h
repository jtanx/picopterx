/**
 * @file waypoints.h
 * @brief The header file for the waypoints code.
 */

#ifndef _PICOPTERX_WAYPOINTS_H
#define _PICOPTERX_WAYPOINTS_H

#include "flightcontroller.h"

namespace picopter {
    /* Forward declaration of the options class */
    class Options;
    
    class Waypoints : FlightTask {
        public:
            Waypoints();
            Waypoints(Options *opts);
            virtual ~Waypoints() override;
            
            void Run(const FlightController *fc, void *opts) override;
        private:
            /** Copy constructor (disabled) **/
            Waypoints(const Waypoints &other);
            /** Assignment operator (disabled) **/
            Waypoints& operator= (const Waypoints &other);
    };
}

#endif // _PICOPTERX_WAYPOINTS_H