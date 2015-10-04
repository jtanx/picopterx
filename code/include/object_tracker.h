/**
 * @file object_tracker.h
 * @brief The header file for the object tracking code.
 */

#ifndef _PICOPTERX_OBJECT_TRACKER_H
#define _PICOPTERX_OBJECT_TRACKER_H

#define FOCAL_LENGTH (3687.5/2592.0)
#define OVERLAP_CONFIDENCE 0.1

/* For the Options class */
#include "opts.h"
#include "flightcontroller.h"
#include "navigation.h"
#include "PID.h"
#include "observations.h"
#include <opencv2/opencv.hpp>
 
namespace picopter {
    /**
     * Class for moving the hexacopter through waypoints.
     */

    class ObjectTracker : public FlightTask {
        public:
            typedef enum {
                TRACK_STRAFE,
                TRACK_ROTATE
            } TrackMethod;
        
            ObjectTracker(TrackMethod method=TRACK_STRAFE);
            ObjectTracker(Options *opts, TrackMethod method=TRACK_STRAFE);
            virtual ~ObjectTracker() override;
            
            TrackMethod GetTrackMethod();
            void SetTrackMethod(TrackMethod method);
            void Run(FlightController *fc, void *opts) override;
            bool Finished() override;

            cv::Vec3d GroundFromGPS(navigation::Coord3D coord);
            navigation::Coord3D GPSFromGround(cv::Vec3d coord);



            Observation ObservationFromImageCoords(TIME_TYPE sample_time, GPSData *pos, navigation::EulerAngle *gimbal, IMUData *imu_data, ObjectInfo *object);
            Observation ObservationFromLidar(TIME_TYPE sample_time, GPSData *pos, navigation::EulerAngle *gimbal, IMUData *imu_data, double lidar_range);
            Observation AssumptionGroundLevel();
            Observation ObservationFromRemote(navigation::Coord3D &pos);

            //transformation matrices for gimbal and body
            cv::Matx33d GimbalToBody(navigation::EulerAngle *gimbal);
            cv::Matx33d BodyToGround(IMUData *imu_data);
            cv::Matx33d BodyToLevel(IMUData *imu_data);
            cv::Matx33d LevelToGround(IMUData *imu_data);

            
            navigation::Coord3D CalculateVantagePoint(GPSData *pos, Observations *object, bool has_fix);
            
            bool matchObsToObj(std::vector<Observation> &visibles, std::vector<Observations> &knownThings);
            bool ChooseObsToObj(std::vector<Observation> &visibles, std::vector<Observations> &knownThings);
            bool NoObjectMemory(std::vector<Observation> &visibles, std::vector<Observations> &knownThings);


        private:
            CLOCK_TYPE m_task_start;
            navigation::Coord3D launch_point;       //centre and orientation of the ground coordinate system

            bool m_observation_mode;
            bool m_demo_mode;
            int waypoint_seq = 0;
            PID m_pidw, m_pidx, m_pidy;//, m_pidz;
            std::atomic<TrackMethod> m_track_method;
            std::atomic<bool> m_finished;
            
            //int TRACK_TOL, 
            int SEARCH_GIMBAL_LIMIT;
            double TRACK_Kpw, TRACK_Kpx, TRACK_Kpy, TRACK_Kpz;
            double TRACK_TauIw, TRACK_TauIx, TRACK_TauIy, TRACK_TauIz;
            double TRACK_TauDw, TRACK_TauDx, TRACK_TauDy, TRACK_TauDz; 
            double TRACK_SETPOINT_W, TRACK_SETPOINT_X, TRACK_SETPOINT_Y, TRACK_SETPOINT_Z;
            int TRACK_SPEED_LIMIT_W, TRACK_SPEED_LIMIT_X, TRACK_SPEED_LIMIT_Y, TRACK_SPEED_LIMIT_Z;
            double desiredSlope;
            int observation_image_rows, observation_image_cols;
            bool print_observation_map;
            int observation_map_count = 0;


            //void EstimatePositionFromImageCoords(GPSData *pos, navigation::EulerAngle *gimbal, IMUData *imu_data, ObjectInfo *object, double lidar_range);

            //void AbsoluteFromRelative(GPSData *pos, IMUData *imu_data, ObjectInfo *object);
            //void CalculateTrackingTrajectory(FlightController *fc, navigation::Vec3D *current, ObjectInfo *object, bool has_fix);

            
            void CalculatePath(FlightController *fc, GPSData *pos,  IMUData *imu_data, navigation::Coord3D dest, navigation::Coord3D Poi, navigation::Vec3D *course);
            void PathWaypoint(FlightController *fc, GPSData *pos, IMUData *imu_data, navigation::Coord3D dest, navigation::Coord3D poi);
            bool UseLidar(ObjectInfo *object, double lidar_range);

            /** Copy constructor (disabled) **/
            ObjectTracker(const ObjectTracker &other);
            /** Assignment operator (disabled) **/
            ObjectTracker& operator= (const ObjectTracker &other);

    };
}

#endif // _PICOPTERX_OBJECT_TRACKER_H
