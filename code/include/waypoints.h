/**
 * @file waypoints.h
 * @brief The header file for the waypoints code.
 */

#ifndef _PICOPTERX_WAYPOINTS_H
#define _PICOPTERX_WAYPOINTS_H

#include "flightcontroller.h"
#include "PID.h"

namespace picopter {
    /* Forward declaration of the options class */
    class Options;
    
    /**
     * The method to be used to navigate to the waypoints
     */
    typedef enum WaypointMethod {
        WAYPOINT_SIMPLE
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
        private:
            std::deque<navigation::Coord2D> m_pts;
            WaypointMethod m_method;
            int m_update_interval;
            double m_waypoint_radius;
            double m_waypoint_control;
            int m_waypoint_idle;
            PID m_pid1;
            
            void SimpleCalculation(navigation::Coord2D c, navigation::Coord2D d);
            /** Copy constructor (disabled) **/
            Waypoints(const Waypoints &other);
            /** Assignment operator (disabled) **/
            Waypoints& operator= (const Waypoints &other);
    };
}

#endif // _PICOPTERX_WAYPOINTS_H