/**
 * @file    camera_stream.h
 * @author	Michael Baxter	<20503664@student.uwa.edu.au>
 * @author  Jeremy Tan <20933708@student.uwa.edu.au>
 * 
 * Class used to start camera stream.
 * 
 * Wonderful omni-function camera streaming action fun!
 * 
 **/
 
#ifndef _PICOPTERX_CAMERA_STREAM_H
#define _PICOPTERX_CAMERA_STREAM_H

/* For the Options class */
#include "opts.h"
#include "common.h"
#include "navigation.h"
#include <opencv2/opencv.hpp>

#define STREAM_FILE "/mnt/ramdisk/out.jpg"

#define LOOKUP_SIZE 8
#define CHAR_SIZE 256

#define CAMERA_OK 0
 
namespace picopter {
    typedef struct {
        int x;
        int y;
        int l;
        int w;
    } CamWindow;

    class CameraStream {
        public:
            typedef enum {
                MODE_NO_PROCESSING = 0,
                MODE_COM = 1,
                MODE_CAMSHIFT = 2,
                MODE_CONNECTED_COMPONENTS = 3,
                MODE_LEARN_COLOUR = 999
            } CameraMode;
            
            CameraStream();
            CameraStream(Options *opts);
            virtual ~CameraStream(void);
            
            bool Start(void);
            void Stop(void);
            
            int GetInputWidth();
            int GetInputHeight();
            
            CameraMode GetMode(void);
            void SetMode(CameraMode mode);

            void SetLearningSize(bool decrease);
            void ShowLearningThreshold(bool show);
            int GetLearningHue();
            void DoAutoLearning(std::map<std::string, int32_t> *ret);
            void DoManualLearning(const std::map<std::string, int32_t> & values, std::map<std::string, int32_t> *ret);
            
            void GetDetectedObjects(std::vector<navigation::Point2D>*);
            
            void SetArrow(navigation::Point2D vec);
            
            double GetFramerate(void);
            bool TakePhoto(std::string);
        private:
            int MIN_HUE, MAX_HUE, MIN_SAT, MAX_SAT, MIN_VAL, MAX_VAL, PIXEL_THRESHOLD;
            int DILATE_ELEMENT, ERODE_ELEMENT;
            int INPUT_WIDTH, INPUT_HEIGHT, PROCESS_WIDTH, PROCESS_HEIGHT, STREAM_WIDTH;
            int PIXEL_SKIP;
            int LEARN_SIZE, LEARN_HUE_WIDTH, LEARN_MIN_HUE, LEARN_AVG_HUE, LEARN_MAX_HUE,
                LEARN_MIN_SAT, LEARN_AVG_SAT, LEARN_MAX_SAT,
                LEARN_MIN_VAL, LEARN_AVG_VAL, LEARN_MAX_VAL;
            int THREAD_SLEEP_TIME;
            double BOX_SIZE;
            
            std::atomic<bool> m_stop;
            CameraMode m_mode;
            
            std::mutex m_worker_mutex;
            std::future<void> m_worker_thread;
            
            std::mutex m_aux_mutex;
            
            navigation::Point2D m_arrow_vec;
            
            cv::VideoCapture m_capture;
            
            bool m_learning_show_threshold;
            
            int frame_counter;
            double m_fps;
            
            bool m_save_photo;
            std::string m_save_filename;
            
            /** Demo mode **/
            bool m_demo;
            
            uchar lookup_threshold[LOOKUP_SIZE][LOOKUP_SIZE][LOOKUP_SIZE];
            uchar lookup_reduce_colourspace[CHAR_SIZE];
            
            void ProcessImages(void);
            bool centerOfMass(cv::Mat& Isrc);
            bool CamShift(cv::Mat& src);
            bool camShift(cv::Mat& Isrc);
            int connectComponents(cv::Mat& Isrc);
            int ConnectedComponents(cv::Mat& src);
            void Threshold(cv::Mat& src, cv::Mat&out);
            void LearnHue(cv::Mat& src, cv::Point &left, cv::Point &right);
            
            std::vector<navigation::Point2D> redObjectList;
            std::vector<CamWindow> windowList;

            std::vector<cv::Scalar> windowColours;


            void RGB2HSV(int r, int g, int b, int *h, int *s, int *v);
            void build_lookup_threshold(uchar lookup_threshold[][LOOKUP_SIZE][LOOKUP_SIZE], int minHue, int maxHue, int minSat, int maxSat, int minVal, int maxVal);
            void build_lookup_reduce_colourspace(uchar lookup_reduce_colourspace[]);
            int unreduce(int x);
            
            void drawCrosshair(cv::Mat& img);
            void drawObjectMarker(cv::Mat& img, cv::Point centre, cv::Scalar colour);
            void drawBox(cv::Mat& img, cv::Point topLeft, cv::Point bottomRight, cv::Scalar colour);
            void drawFramerate(cv::Mat& img);
            void drawArrow(cv::Mat& img, cv::Point from, cv::Point to);
            
            void buildColours(std::vector<cv::Scalar>*);
            
            /** Copy constructor (disabled) **/
            CameraStream(const CameraStream &other);
            /** Assignment operator (disabled) **/
            CameraStream& operator= (const CameraStream &other);
    };
}

#endif // _PICOPTERX_CAMERA_STREAM_H
