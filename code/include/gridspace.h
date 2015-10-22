/**
 * @file gridspace.h
 * @brief Obstacle describes obstacles in space using a voxel system.
 **/

#ifndef _PICOPTERX_GRID_SPACE_H
#define _PICOPTERX_GRID_SPACE_H

#include "waypoints.h"
#include <iostream>

namespace picopter {
    class GridSpace{

        public:
            typedef struct voxel {
                int observations;
                bool isFull;
            } Voxel;
            
            typedef struct index3D{
                double x;
                double y;
                double z;
            } index3D;
            
            GridSpace(FlightController *fc);
            
            void raycast(FlightController *fc);
            void printToConsole(int rangeMin, int rangeMax, int zDepth);
            void writeImage();
            navigation::Coord3D gridToWorld(index3D loc);
            
            std::vector< std::vector< std::vector<voxel> > > grid;
               
        private:
            double voxelLength;
            double voxelWidth;
            double voxelHeight;           
            navigation::Coord3D launchPoint;
            index3D launchIndex;
            std::mutex mutex;
            
            index3D findEndPoint(FlightController *fc);
            navigation::Coord3D getGPS();
            index3D worldToGrid(navigation::Coord3D GPSloc);
            double degToRad(double deg);
            
               
    };
}

#endif // _PICOPTERX_GRID_SPACE_H
