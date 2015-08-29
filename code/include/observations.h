/**
 * @file observations.h
 * @brief The header file for the observations code.
 */


/*
    a 3d gaussian elliptical structure: (treat as a probability density function PDF)
        p = e^-(ax^2 + by^2 + cz^2 + dyz + ezx + fxy + g)
    The semi-major axes are encoded with
        a,b,c, and d,e,f
    the centre of the ellipse the stationary point of the polynomial
        ax^2 + by^2 + cz^2 + dyz + ezx + fxy + g
    combining independent measurements of an object location is dine by multiplying PDFs, or in this case, summing polynomials.

    For the sake of sanity and legibility, converting to matrix definition.
        This increases the complexity of combining observations, but not by much.
        It also keeps the transforation matrices at 3x3 instead of 10x10 so the opencv API is friendlier

*/


#ifndef _PICOPTERX_OBSERVATIONS_H
#define _PICOPTERX_OBSERVATIONS_H

#include "opts.h"
#include "flightcontroller.h"
#include "camera_stream.h"
#include <opencv2/opencv.hpp>

namespace picopter {

    //a 3d gaussian elliptical structure

    const int _distrib_coeffs = 10;
    typedef struct Distrib {
        cv::Matx33d axes;   //eigenvectors are the semi-major axes, eigenvalues are the sigma=1 widths
        cv::Matx31d location;   //offset from origin column vector
        //double coeffs[10];
    } Distrib;

    //parameters to construct the elliptic structure above
    typedef struct DistribParams {
        double x,y,z;
        double sigma_x, sigma_y, sigma_z;
        double yaw, pitch, roll;
    } DistribParams;

    //where the observation came from
    typedef enum Source {   //should probably be a collection of booleans instead of splitting sensor values
        CAMERA_BLOB,    //blob detection has no sense of range, (include a picture)
        CAMERA_SIFT,    //SIFT doesn't resolve range (include a picture, and the SIFT characteristic data)
        CAMERA_FLOW,    //optical flow will resolve a range and blob (probably include two images and IMU velocity)
        LIDAR,          //lidar sensor has a small dot, but can't identify objects
        FLOW,           //flow sensor has a large width uncertainty
        TELEM           //objects transmitted from external sources
    } Source;
    //ObjectInfo    //detections from the camera stream (grouped into single objects)

    typedef struct Observation {
        //location of copter +uncertainty
        //time
        //IMU data (including velocity)
        //3D probability density function (elliptical normal)
        Distrib distrib;
        //sensor pack the detection came from
        Source source;

        //information from camera detection defined in camera_stream
        ObjectInfo* camDetection;
        //something for lidar data
        //something for

    } Observation;

    Distrib generatedistrib(DistribParams params);             //generate an distrib struct from a primitive and operators
    DistribParams getdistribParams(Distrib A);                  //calculate the centre and covariance widths of this object

    Distrib combineDistribs(Distrib A, Distrib B);         //combine two distrib distributions (as though statistically independent)
    Distrib translateDistrib(Distrib A, double x, double y, double z);      //translate an distrib struct from the origin
    Distrib rotateDistrib(Distrib A, double yaw, double pitch, double roll);       //translate an distrib struct about the origin
    Distrib stretchDistrib(Distrib A, double sx, double sy, double sz);         //stretch an distrib struct about the origin


    class Observations {
        public:
            Observations();
            double getSameProbability(Observation* observation);    //estimate the probability the given observation is of the same object
            void appendObservation(Observation* observation);     //add another sighting to this object
            void removeObservation(Observation* observation);     //remove an observation from this object

        private:
            Distrib distrib;    //cumulative distrib
            std::vector<Observation*> sightings;


    };
}



#endif  // _PICOPTERX_OBSERVATIONS_H