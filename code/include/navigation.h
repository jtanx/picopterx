/**
 * @file navigation.h
 * @brief General navigation code.
 */

#ifndef _PICOPTERX_NAVIGATION_H
#define _PICOPTERX_NAVIGATION_H

#define _USE_MATH_DEFINES //For Windows compatibility
#include <cmath>

/** @todo Confirm correct radius (see http://www.ga.gov.au/scientific-topics/positioning-navigation/geodesy/geodetic-techniques/distance-calculation-algorithms) **/
#define RADIUS_OF_EARTH		6364.963
#define RAD2DEG(x) 			((x) * (180.0 / M_PI))
#define DEG2RAD(x)			((x) * (M_PI / 180.0))

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
         * @param here The current location (must have lat/lon members)
         * @param bl The bottom left coordinate (must have lat/lon members)
         * @param tr The top right coordinate (must have lat/lon members)
         * @return true iff the current location is within bounds.
         */
        template <typename Coord>
        bool InBounds(Coord here, Coord bl, Coord tr) {
            return (here.lat >= bl.lat && here.lat <= tr.lat) &&
                   (here.lon >= bl.lon && here.lat <= tr.lon);
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
         * Uses the Haversine method to calculate the distance.
         * @param from The first coordinate, in radians.
         * @param to The second coordinate,in radians.
         * @return The distance between the coordinates, in metres.
         */
        template <typename Coord>
        double CoordDistance(Coord from, Coord to) {
            double haversine = pow(sin((to.lat-from.lat)/2), 2) + 
                               cos(from.lat) * cos(to.lat) * 
                               pow(sin((to.lon - from.lon)/2), 2);
            return 2 * RADIUS_OF_EARTH * 1000 * asin(sqrt(haversine));
        }
        
        /**
         * Calculates the initial bearing (forward azimuth).
         * @param from The first coordinate, in radians.
         * @param to The second coordinate, in radians.
         * @param The bearing, in radians.
         */
         template <typename Coord>
         double CoordBearing(Coord from, Coord to) {
             double y = sin(to.lon - from.lon) * cos(to.lat);
             double x = cos(from.lat) * sin(to.lat) -
                        sin(from.lat) * cos(to.lon) * cos(to.lat - from.lat);
            return atan2(y, x);
         }
        
        const Point2D PERTH_BL = {-33, 115};
        const Point2D PERTH_TR = {-31, 117};
    }
}

#endif // _PICOPTERX_NAVIGATION_H