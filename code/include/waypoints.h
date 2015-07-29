/**
 * @file waypoints.h
 * @brief The header file for the waypoints code.
 */

#ifndef _PICOPTERX_WAYPOINTS_H
#define _PICOPTERX_WAYPOINTS_H

/* For the Options class */
#include "opts.h"
#include "flightcontroller.h"
#include "PID.h"

namespace picopter {
    /**
     * The method to be used to navigate to the waypoints
     */
    typedef enum WaypointMethod {
        /** Follow the waypoints directly. **/
        WAYPOINT_NORMAL,
        /** The waypoints list consists of two waypoints, specifying the corners for a lawnmower search pattern. **/
        WAYPOINT_LAWNMOWER
    } WaypointMethod;

    /**
     * Class for moving the hexacopter through waypoints.
     */
    class Waypoints : public FlightTask {
        public:
            Waypoints(std::deque<navigation::Coord2D> pts, WaypointMethod method);
            Waypoints(Options *opts, std::deque<navigation::Coord2D> pts, WaypointMethod method);
            virtual ~Waypoints() override;
            
            void Run(FlightController *fc, void *opts) override;
            bool Finished() override;
        private:
            std::deque<navigation::Coord2D> m_pts;
            WaypointMethod m_method;
            int m_update_interval;
            double m_waypoint_radius;
            int m_waypoint_idle;
            std::atomic<bool> m_finished;

            /** Copy constructor (disabled) **/
            Waypoints(const Waypoints &other);
            /** Assignment operator (disabled) **/
            Waypoints& operator= (const Waypoints &other);
    };
}

#endif // _PICOPTERX_WAYPOINTS_H