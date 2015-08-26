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
        WAYPOINT_NORMAL = 0,
        /** The waypoints list consists of two waypoints, specifying the corners for a lawnmower search pattern. **/
        WAYPOINT_LAWNMOWER = 1,
        /** Spiral mode (todo) **/
        WAYPOINT_SPIRAL = 2
    } WaypointMethod;

    /**
     * Class for moving the hexacopter through waypoints.
     */
    class Waypoints : public FlightTask {
        public:
            typedef struct Waypoint {
                navigation::Coord3D pt;
                navigation::Coord3D roi;
                bool has_roi;
                bool absolute_alt;
            } Waypoint;
            
            Waypoints(std::deque<Waypoint> pts, WaypointMethod method);
            Waypoints(Options *opts, std::deque<Waypoint> pts, WaypointMethod method);
            virtual ~Waypoints() override;
            
            void Run(FlightController *fc, void *opts) override;
            bool Finished() override;
        private:
            /** The list of waypoints to move through **/
            std::deque<Waypoint> m_pts;
            /** The waypoint movement method (e.g. manual or lawnmower) **/
            WaypointMethod m_method;
            /** How often to update the calculation **/
            int m_update_interval;
            /** The acceptance radius (in m) for being at a waypoint. **/
            double m_waypoint_radius;
            /** The idle time (in ms) at the waypoint **/
            int m_waypoint_idle;
            /** The spacing (in m) between sweeps for the lawnmower pattern. **/
            double m_sweep_spacing;
            /** The current image number for detected objects **/
            int m_image_counter;
            /** Flag to indicate if we're finished **/
            std::atomic<bool> m_finished;
            /** Log to store information about detected objects **/
            DataLog m_log;
            
            std::deque<Waypoint> GenerateLawnmowerPattern(Waypoint start, Waypoint end);
            std::deque<Waypoint> GenerateSpiralPattern(Waypoint centre, Waypoint edge);
            /** Copy constructor (disabled) **/
            Waypoints(const Waypoints &other);
            /** Assignment operator (disabled) **/
            Waypoints& operator= (const Waypoints &other);
    };
}

#endif // _PICOPTERX_WAYPOINTS_H
