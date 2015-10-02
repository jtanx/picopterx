#include <opencv2/opencv.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

int main(int argc, char *argv[]) {
    cv::VideoCapture cap(-1);
    cap.set(CV_CAP_PROP_FRAME_WIDTH, 160);
    cap.set(CV_CAP_PROP_FRAME_HEIGHT, 120);
    
    cv::HOGDescriptor hog;
    hog.setSVMDetector(cv::HOGDescriptor::getDefaultPeopleDetector());
    
    cv::Mat img, grey;
    std::vector<cv::Rect> found;
    while(cv::waitKey(1) != 'q') {
        cap >> img;
        cv::cvtColor(img, grey, CV_BGR2GRAY);
        hog.detectMultiScale(grey, found);
        
        for (size_t i = 0; i < found.size(); i++) {
            cv::rectangle(img, found[i].tl(), found[i].br(), cv::Scalar(0, 0, 255), 2);
        }
        cv::imshow("HOG", img);
    }
    return 0;
}