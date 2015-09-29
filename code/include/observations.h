/**
 * @file observations.h
 * @brief The header file for the observations code.
 */


/*
    I want a computationally efficient model for uncertainty.
    If I wanted an accurrate model for uncertainty, I'd use a voxel system with block density following probability density gradient
        Such a system (I expect) would chew on truly vast quantities of memory and computing power
    This code runs on an ellipsoidal gaussian structure

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


/*
Partial implementation and generalisation to 3D of:
    Representation and Estimation of Spatial Uncertainty
    Randall C. Smith and Peter Cheeseman
    The International Journal of Robotics Research 1986; 5; 56
    DOI: 10.1177/027836498600500404

Motivated by investigation into the monocular case of:
    IEEE JOURNAL OF ROBOTICS AND AUTOMATION, VOL. RA-3, NO. 3, JUNE 1987
    Error Modeling in Stereo Navigation


*/

#ifndef _PICOPTERX_OBSERVATIONS_H
#define _PICOPTERX_OBSERVATIONS_H

#include "opts.h"
#include "flightcontroller.h"
#include "camera_stream.h"
#include <opencv2/opencv.hpp>

#define TICKS_PER_SEC 1000000000l

//#define TIME_TYPE std::chrono::duration<long int, std::ratio<1l, TICKS_PER_SEC> >
#define TIME_TYPE std::chrono::steady_clock::duration

//#define CLOCK_TYPE std::chrono::time_point<std::chrono::_V2::steady_clock, std::chrono::duration<long int, std::ratio<1l, TICKS_PER_SEC> > >
#define CLOCK_TYPE std::chrono::time_point<std::chrono::steady_clock>
namespace picopter {

    //a 3d gaussian elliptical structure

    typedef struct Distrib {
        cv::Matx33d axes;   //eigenvectors are the semi-major axes, eigenvalues are the sigma=1 widths
        cv::Vec3d vect;   //offset from origin column vector
        //double coeffs[10];
    } Distrib;
    //empty measurements will just be zeroed out. A zero matrix represents infinite variance and covariance
    //zero coefficients in the polynomial case, zeroed matrix in the vector case

    //parameters to construct the elliptic structure above
    typedef struct DistribParams {
        double x,y,z;
        double sigma_x, sigma_y, sigma_z;
        double yaw, pitch, roll;
    } DistribParams;


    //where the observation came from
    typedef enum Source {   //should probably be a collection of booleans instead of splitting sensor values
        CAMERA_BLOB,    /** blob detection has no sense of range, (include a picture) **/
        CAMERA_SIFT,    /** SIFT doesn't resolve range (include a picture, and the SIFT characteristic data) **/
        CAMERA_FLOW,    /** optical flow will resolve a range and blob (probably include two images and IMU velocity) **/
        LIDAR,          /** lidar sensor has a small dot, but can't identify objects **/
        FLOW,           /** flow sensor has a large width uncertainty **/
        TELEM,          /** objects transmitted from external sources **/
        ASSUMPTION,     /** Fairy stories we tell our robots. **/
        INTERPOLATION   /** Current state derived from our model of the object's motion **/
    } Source;
    //ObjectInfo    //detections from the camera stream (grouped into single objects)

    //The specific measurement of the object.  These might get flushed to disk if we ever get that far.
    typedef struct Observation {
        //location of copter +uncertainty
        
        //time
        TIME_TYPE sample_time;

        //sensor pack the detection came from
        Source source;

        //IMU data (including velocity)
        //3D probability density function (ellipsoidal normal )
        Distrib location;
        //velocity information if it is available
        Distrib velocity;   
        //acceleration information if it is available
        Distrib acceleration;
        //information from camera detection defined in camera_stream
        ObjectInfo camDetection;

        //something for lidar data
        //something for

    } Observation;

    Distrib generatedistrib();                                                  //Generate an empty (sigma 1) distribution
    Distrib generatedistrib(DistribParams params);                              //generate an distrib struct from a primitive and operators
    DistribParams getdistribParams(Distrib A);                                  //calculate the centre and covariance widths of this object
    
    double sampleDistrib(Distrib &A, cv::Vec3d &B);

    Distrib combineDistribs(Distrib A, Distrib B);                              //combine two distrib distributions (as though statistically independent)

    Distrib translateDistrib(Distrib A, cv::Vec3d offset);
    inline Distrib translateDistrib(Distrib A, double x, double y, double z){   //translate a distrib struct from the origin
        return translateDistrib(A, cv::Vec3d(x,y,z));
    }
    Distrib rotateDistrib(Distrib A, cv::Matx33d Mrot);                         //rotate a distrib struct about the origin
    Distrib rotateDistribEuler(Distrib A, double roll, double pitch, double yaw);    //rotate a distrib struct about the origin
    
    cv::Matx33d rotationMatrix(double roll, double pitch, double yaw);

    Distrib stretchDistrib(Distrib A, double sx, double sy, double sz);         //stretch a distrib struct about the origin
    inline Distrib stretchDistrib(Distrib A, double s){                         //overload
        return stretchDistrib(A,s,s,s);}                                              
    Distrib vectorSum(Distrib A, Distrib B);                                    //move the centre of A by a distance described by B.
    Distrib changeStep(Distrib newLoc, Distrib oldLoc, TIME_TYPE timestep);     //find the step from one distrib to another.

    void rasterDistrib(cv::Mat *mat, Distrib *dist, cv::Vec4b colour, double scale);
    void storeDistrib(cv::Mat* mat, std::string filename);

    //one per distinct object.
    class Observations {
        public:
            Observations(Observation firstSighting);
            double getSameProbability(Observation observation);    //estimate the probability the given observation is of the same object
            void appendObservation(Observation observation);       //add another sighting to this object
            void removeObservation(Observation observation);       //remove an observation from this object
            void updateObject(TIME_TYPE timestep);                  //update the location and velocity with the time.
            TIME_TYPE lastObservation();
            Distrib getLocation();
        private:
            Observation accumulator;

            //TIME_TYPE last_sample;                                          //A timestamp for the last observation
            //characteristic data (colour, speckle histogram, glyph ID etc)
            std::vector<Observation> sightings;                     //the collection of sightings associated with this object
            //Distrib location;                                       //cumulative uncertainty distribution
            //Distrib velocity;                                       //the current distribution for velocity. distrib will be translated and inflated by this much per unit of time.
            //Distrib acceleration;                                   //the current distribution for acceleration. velocity will be translated and inflated by this much per unit of time.
            //acceleration won't be measured by most sensors, so we'll just leave this at some reasonable value
            //for chasing the buggy, we'll set this as a flat-ish circle on the origin

    };
}



#endif  // _PICOPTERX_OBSERVATIONS_H