/**
* @file trackertest.cpp
* @brief Object tracker Code tests
*/
#include "picopter.h"
#include "object_tracker.h"
#include "observations.h"

using namespace picopter;
using namespace picopter::navigation;


void printMatrix(cv::Matx33d mat){
    for(int i=0;i<3;i++){
        std::cout << "[";
        for(int j=0;j<3;j++){
            std::cout << mat(i,j);
            if(j!=2) std::cout << ", ";
        }
        std::cout << "]" << std::endl;
    }
}

void printVector(cv::Matx31d mat){
    for(int i=0;i<3;i++){
        std::cout << "[" << mat(i,0) << "]" << std::endl;
    }
}

int main(int argc, char *argv[]) {

    std::cout << "Test Rotation Matrix: 30 deg roll" << std::endl;

    cv::Matx33d rot = rotationMatrix(30,0,0);
    printMatrix(rot);

    std::cout << "Test Base Distribution" << std::endl;
    Distrib A = generatedistrib();
    printMatrix(A.axes);
    printVector(A.vect);
    std::cout << "Translate Distrib by x=10" << std::endl;
    Distrib B = translateDistrib(A, 10, 0, 0);
    printMatrix(B.axes);
    printVector(B.vect);
    std::cout << "Stretch Distrib by y=5" << std::endl;
    Distrib C = stretchDistrib(B, 1, 5, 1);
    printMatrix(C.axes);
    printVector(C.vect);
    std::cout << "Rotate Distrib by yaw 30 deg" << std::endl;
    Distrib D = rotateDistrib(C, rotationMatrix(0,0,30));
    printMatrix(D.axes);
    printVector(D.vect);


    





    return 0;
}