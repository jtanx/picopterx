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
            /** y-coordinate. **/
            double y;
            /** Returns the vector magnitude. **/
            double magnitude() { return std::sqrt(x*x + y*y); };
        } Point2D;
        
        /**
         * Holds a 3-dimensional position in Cartesian space.
         */
        typedef struct Point3D {
            /** x-coordinate. **/
            double x;
            /** y-coordinate. **/
            double y;
            /** z-coordinate. **/
            double z;
            /** Implicit conversion to 2D coordinate. **/
            operator Point2D() { Point2D ret = {x, y}; return ret; }
            /** Returns the vector magnitude. **/
            double magnitude() { return std::sqrt(x*x + y*y + z*z); };
        } Point3D;
        
        /**
         * Holds a 3-dimensional position in Cartesian space.
         */
        typedef struct Point4D {
            /** x-coordinate. **/
            double x;
            /** y-coordinate. **/
            double y;
            /** z-coordinate. **/
            double z;
            /** w-coordinate. **/
            double w;
            /** Implicit conversion to 3D coordinate. **/
            operator Point3D() { return Point3D{x, y, z}; }
            /** Returns the vector magnitude. **/
            double magnitude() { return std::sqrt(x*x + y*y + z*z); };
        } Point4D;
        
        /** 2D vector (alias) **/
        typedef struct Point2D Vec2D;
        
        /** 3D vector (alias) **/
        typedef struct Point3D Vec3D;
        
        /** 4D vector (alias) **/
        typedef struct Point4D Vec4D;
        
        /**
         * Holds a set of Euler angles.
         */
        typedef struct EulerAngle {
            /** Roll of the IMU, -180 to 180 **/
            double roll;
            /** Pitch of the IMU, -180 to 180 **/
            double pitch;
            /** Yaw of the IMU, -180 to 180 **/
            double yaw;
        } EulerAngle;
        
        /**
         * Defines a tile point.
         * @see http://wiki.openstreetmap.org/wiki/Slippy_map_tilenames#Zoom_levels
         */
        typedef struct TilePoint {
            /** The x-point **/
            int x;
            /** The y-point **/
            int y;
            /** The zoom level **/
            int zoom;
        } TilePoint;
        
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
         * @param from The first coordinate, in degrees.
         * @param to The second coordinate,in degrees.
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
         * @param from The first coordinate, in degrees.
         * @param to The second coordinate, in degrees.
         * @param The bearing, in degrees, 0 < ret < 360
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
        
        /**
         * Calculates the bearing as per CoordBearing, but translated to
         * be relative to the positive x-axis.
         * @param [in] from The first coordinate, in degrees.
         * @param [in] to The second coordinate, in degrees.
         * @return The bearing, in degrees, -180 < ret < 180, relative to the
         *         positive x-axis.
         */
        template <typename Coord1, typename Coord2>
        double CoordBearingX(Coord1 from, Coord2 to) {
            double ret = 90 - CoordBearing(from, to);
            return (ret < -180) ? (ret + 360) : ret;
        }
        
        /**
         * Adds an offset to a given coordinate.
         * This method uses a flat-earth approximation.
         * @param [in] c The coordinate to offset.
         * @param [in] radius The magnitude of the offset, in metres.
         * @param [in] angle The angle of the offset, in degrees, relative
         *                   to the positive x-axis (-180 < angle < 180).
         * @return The offset coordinate.
         */
        template <typename Coord>
        Coord CoordAddOffset(Coord c, double radius, double angle) {
            double offset_x = radius * cos(DEG2RAD(angle));
            double offset_y = radius * sin(DEG2RAD(angle));
            offset_x /= 1000 * RADIUS_OF_EARTH * cos(DEG2RAD(c.lat));
            offset_y /= 1000 * RADIUS_OF_EARTH;
            
            c.lat += RAD2DEG(offset_y);
            c.lon += RAD2DEG(offset_x);
            return c;
        }
        
        /**
         * Add a vector in NED frame to the coordinate.
         * @param [in] c The coordinate to offset.
         * @param [in] v The vector (units in metres).
         * @return The offset coordinate.
         */
        template <typename Coord, typename Vect>
        Coord CoordAddOffset(Coord c, Vect v) {
            double offset_x = v.x / (1000.0 * RADIUS_OF_EARTH * cos(DEG2RAD(c.lat)));
            double offset_y = v.y / (1000.0 * RADIUS_OF_EARTH);
            
            c.lat += RAD2DEG(offset_y);
            c.lon += RAD2DEG(offset_x);
            return c;
        }
        
        /**
         * Converts a coordinate/zoom level into the corresonding tile number.
         * @param [in] from The coordinate of the tile.
         * @see http://wiki.openstreetmap.org/wiki/Slippy_map_tilenames#Zoom_levels
         * @param [in] zoom The zoom level of the tile.
         * @return The tile number.
         */
        template <typename Coord>
        TilePoint CoordToTile(Coord from, int zoom) {
            TilePoint ret;
            int n =  1 << zoom;
            double rlat = DEG2RAD(from.lat);
            ret.x = static_cast<int>(((from.lon + 180.0) / 360.0) * n);
            ret.y = static_cast<int>((1.0 - log(tan(rlat) + (1/cos(rlat)))/M_PI)/2.0 * n);
            ret.zoom = zoom;
            return ret;
        }
        
        /**
         * Rotates from our body coordinates to NED coordinates.
         * @param [in] v The vector to rotate.
         * @param [in] yaw The yaw to rotate to.
         * @return The rotated coordinate in NED frame.
         */
        template <typename Vect>
        Vect RotateBodyToNED(Vect v, double yaw) {
            double cy = std::cos(DEG2RAD(yaw)), sy = std::sin(DEG2RAD(yaw));
            double x = v.x * cy - v.y * sy;
            double y = v.x * sy + v.y * cy;
            v.x = x;
            v.y = y;
            //v.z = -v.z;
            return v;
        }
        
        const Coord2D PERTH_BL = {-33, 115};
        const Coord2D PERTH_TR = {-31, 117};
    }
}

#endif // _PICOPTERX_NAVIGATION_H
