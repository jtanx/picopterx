/**
 * @file navigation.h
 * @brief General navigation code.
 */

#ifndef _PICOPTERX_NAVIGATION_H
#define _PICOPTERX_NAVIGATION_H

#define _USE_MATH_DEFINES //For Windows compatibility
#include <cmath>

/** Radius of the earth (Australian tuned; in km) **/
#define RADIUS_OF_EARTH 6364.963
#define RAD2DEG(x)      ((x) * (180.0 / M_PI))
#define DEG2RAD(x)      ((x) * (M_PI / 180.0))

#define sin2(x) (sin(x) * (sin(x)))

namespace picopter {
    namespace navigation {
        /**
         * Holds 2-dimensional geographical coordinate position information.
         */
        typedef struct Coord2D {
            /** Latitude **/
            double lat;
            /** Longitud **/
            double lon;
        } Coord2D;
        
        /**
         * Holds 3-dimensional geographical coordinate position information.
         */
        typedef struct Coord3D {
            /** Latitude **/
            double lat;
            /** Longitude **/
            double lon;
            /** Altitude **/
            double alt;
            /** Implicit conversion to 2D coordinate **/
            operator Coord2D() { Coord2D ret = {lat,lon}; return ret; };
        } Coord3D;
        
        /**
         * Holds a 2-dimensional position in Cartesian space.
         */
        typedef struct Point2D {
            /** x-coordinate. **/
            double x;
            /** y-coordinate **/
            double y;
        } Point2D;
        
        /**
         * Holds a 3-dimensional position in Cartesian space.
         */
        typedef struct Point3D {
            /** x-coordinate. **/
            double x;
            /** y-coordinate **/
            double y;
            /** z-coordinate **/
            double z;
            /** Implicit conversion to 2D coordinate **/
            operator Point2D() { Point2D ret = {x, y}; return ret; }
        } Point3D;
        
        /**
         * Holds a set of Euler angles.
         */
        typedef struct EulerAngle {
            /** Roll of the IMU, -pi to pi **/
            double roll;
            /** Pitch of the IMU, -pi to pi **/
            double pitch;
            /** Yaw of the IMU, -pi to pi **/
            double yaw;
        } EulerAngle;
        
        /**
         * Determines if the given coordinate is within the bounded rectangle.
         * The bounds are inclusive.
         * @param here The current location (must have lat/lon members)
         * @param bl The bottom left coordinate (must have lat/lon members)
         * @param tr The top right coordinate (must have lat/lon members)
         * @return true iff the current location is within bounds.
         */
        template <typename Coord1, typename Coord2, typename Coord3>
        bool CoordInBounds(Coord1 here, Coord2 bl, Coord3 tr) {
            return (here.lat >= bl.lat && here.lat <= tr.lat) &&
                   (here.lon >= bl.lon && here.lon <= tr.lon);
        }
        
        /**
         * Converts the coordinate position from degrees to radians
         * @param a The coordinate, in degrees
         */
        template <typename Coord>
        void CoordInRadians(Coord &a) {
            a.lat = DEG2RAD(a.lat);
            a.lon = DEG2RAD(a.lon);
        }
        
        /**
         * Converts the coordinate position from radians to degrees.
         * @param a The coordinate, in radians
         */
        template <typename Coord>
        void CoordInDegrees(Coord &a) {
            a.lat = RAD2DEG(a.lat);
            a.lon = RAD2DEG(a.lon);
        }
        
        /**
         * Calculates the distance between two coordinates.
         * Uses the Haversine method (great-circle distance) with an 
         * Earth radius of 6364.963km. 
         * @see http://www.ga.gov.au/scientific-topics/positioning-navigation/geodesy/geodetic-techniques/distance-calculation-algorithms
         * @param from The first coordinate, in radians.
         * @param to The second coordinate,in radians.
         * @return The distance between the coordinates, in metres.
         */
        template <typename Coord1, typename Coord2>
        double CoordDistance(Coord1 from, Coord2 to) {
            CoordInRadians(from);
            CoordInRadians(to);
            
            double haversine = sin2((to.lat-from.lat)/2) + 
                               cos(from.lat) * cos(to.lat) * 
                               sin2((to.lon - from.lon)/2);
            return 2 * RADIUS_OF_EARTH * 1000 * asin(sqrt(haversine));
        }
        
        /**
         * Calculates the initial bearing (forward azimuth).
         * @param from The first coordinate, in radians.
         * @param to The second coordinate, in radians.
         * @param The bearing, in radians (-pi < x < pi; CW positive from N)
         */
        template <typename Coord1, typename Coord2>
        double CoordBearing(Coord1 from, Coord2 to) {
            CoordInRadians(from);
            CoordInRadians(to);
            
            double x = cos(from.lat) * sin(to.lat) -
                       sin(from.lat) * cos(to.lat) * cos(to.lon - from.lon);
            double y = sin(to.lon - from.lon) * cos(to.lat);
            double ret = RAD2DEG(atan2(y, x));
            return (ret < 0) ? (ret + 360) : ret;
        }
        
        const Coord2D PERTH_BL = {-33, 115};
        const Coord2D PERTH_TR = {-31, 117};
    }
}

#endif // _PICOPTERX_NAVIGATION_H
