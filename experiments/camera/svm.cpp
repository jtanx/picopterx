/**
 * @file svm.cpp
 * @brief Uses SVM machine learning to detect objects in OpenCV.
 */

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/ml/ml.hpp>

using namespace cv;

int main()
{
    CvSVM svm;
    if (true) {
        svm.load("training.xml");
    } else {
        // Set up training data
        float labels[10] = {1.0, 1.0, 1.0, 1.0, 1.0, -1.0, -1.0, -1.0, -1.0, -1.0};
        Mat labelsMat(10, 1, CV_32FC1, labels);

        Mat trainingDataMat(10, 800*600*3, CV_32FC1);

        // Set up SVM's parameters
        CvSVMParams params;
        params.svm_type    = CvSVM::C_SVC;
        params.kernel_type = CvSVM::LINEAR;
        params.term_crit   = cvTermCriteria(CV_TERMCRIT_ITER, 100, 1e-6);

        // Train the SVM
        for (int i = 0; i < 10; i++) {
            Mat mat = imread((std::string("img-") + std::to_string(i+1) + ".jpg").c_str(), 0);

            int jj = 0; // Current column in training_mat
            for (int j = 0; j<mat.rows; j++) {
                for (int k = 0; k < mat.cols; k++) {
                    trainingDataMat.at<float>(i,jj++) = mat.at<uchar>(j,k);
                }
            }
        }
        
        svm.train(trainingDataMat, labelsMat, Mat(), Mat(), params);
        svm.save("training.xml"); 
    }
    VideoCapture cap(-1);
    cap.set(CV_CAP_PROP_FRAME_WIDTH, 800);
    cap.set(CV_CAP_PROP_FRAME_HEIGHT, 600);
    while (true) {
        Mat mat;
        cap >> mat;
        imshow("Stream", mat);
        waitKey(1);
        Mat mat1d = mat.reshape(1, mat.channels()*mat.size().area());
        mat1d.convertTo(mat1d,CV_32FC1, 1.0);
        printf("Prediction: %f\n", svm.predict(mat1d));
        fflush(stdout);
    }

}