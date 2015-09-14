/**
 * @file camera_stream.h
 * @brief Camera interaction declarations.
 **/

#ifndef _PICOPTERX_CAMERA_STREAM_H
#define _PICOPTERX_CAMERA_STREAM_H

/* For the Options class */
#include "opts.h"
#include "common.h"
#include "navigation.h"
#include "flightboard.h" //For HUDInfo
#include <opencv2/opencv.hpp>
#ifdef IS_ON_PI
#  include "omxcv.h"
 #endif

#define STREAM_FILE "/mnt/ramdisk/out.jpg"
/** The size of the threshold lookup (number of colour bins per channel) **/
#define THRESH_SIZE 16
/** The scaling factor for reducing a value into its respective colour bin **/
#define THRESH_DIV (256/THRESH_SIZE)
/** Un-reduces a threshold value into the midpoint of the colour bin. **/
#define UNREDUCE(x) (((x)*255 + 127) / THRESH_SIZE)

namespace picopter {

    /**
     * Structure to hold hue thresholding information.
     */
    typedef struct HueThresholds {
        int hue_min, hue_max;
        int sat_min, sat_max;
        int val_min, val_max;

        cv::Scalar Min() {return cv::Scalar(hue_min, sat_min, val_min);}
        cv::Scalar Max() {return cv::Scalar(hue_max, sat_max, val_max);}
    } HueThresholds;

    /**
     * Holds information about a detected object.
     */
    typedef struct ObjectInfo {
        /** The object ID **/
        int id;
        /** The size of the image used for this object detection. **/
        int image_width, image_height;
        /** Position in the detected frame **/
        navigation::Point2D position;
        /** The relative location (in body-frame coordinates; real units, e.g. metres) **/
        navigation::Point3D offset;
        /** Bounding rectangle of the object */
        cv::Rect bounds;
        /** Real-world location (lat/lon/alt) **/
        navigation::Coord3D location;
    } ObjectInfo;
    
    /**
     * Holds information about a glyph.
     */
    typedef struct CameraGlyph {
        /** The glyph ID. Should be unique. **/
        int id;
        /** The path to the glyph file, if any. **/
        std::string path;
        /** Glyph description. **/
        std::string description;
        /** The actual glyph image. **/
        cv::Mat image;
    } CameraGlyph;

    /**
     * Camera class. Uses OpenCV to interact with the camera.
     */
    class CameraStream {
        public:
            typedef enum {
                MODE_NO_PROCESSING = 0,
                MODE_COM = 1,
                MODE_CAMSHIFT = 2,
                MODE_CONNECTED_COMPONENTS = 3,
                MODE_CANNY_GLYPH = 4,
                MODE_THRESH_GLYPH = 5,
                MODE_LEARN_COLOUR = 999
            } CameraMode;

            CameraStream();
            CameraStream(Options *opts);
            virtual ~CameraStream(void);

            CameraMode GetMode(void);
            CameraMode SetMode(CameraMode mode);

            int GetInputWidth(void);
            int GetInputHeight(void);

            void GetConfig(Options *config);
            void SetConfig(Options *config);
            
            void SetHUDInfo(HUDInfo *hud);

            void DoAutoLearning(void);

            void GetDetectedObjects(std::vector<ObjectInfo>* objects);
            double GetFramerate(void);
            bool TakePhoto(std::string filename);
            void SetTrackingArrow(navigation::Point3D arrow);
        private:
            /** A list of distinct colours **/
            static const std::vector<cv::Scalar> m_colours;
            /** The OpenCV video capture handle **/
            cv::VideoCapture m_capture;
            /** Flag to indicate that the worker thread should be stopped **/
            std::atomic<bool> m_stop;
            /** The current camera mode **/
            CameraMode m_mode;

            /** The main mutex to interact with the thread **/
            std::mutex m_worker_mutex;
            /** Secondary mutex to interact with worker **/
            std::mutex m_aux_mutex;
            /** The video processing thread **/
            std::future<void> m_worker_thread;

            /** The colour thresholding parameters **/
            HueThresholds m_thresholds;
            /** The colour auto-learning thresholding parameters **/
            HueThresholds m_learning_thresholds;
            /** The processing rate (FPS) **/
            double m_fps;
            /** Demo mode (displays camera stream in GTK window) **/
            bool m_demo_mode;
            /** Show the working copy (e.g. thresholded image) **/
            bool m_show_backend;
            /** Indicates that a snapshot should be taken **/
            bool m_save_photo;
            /** The path to store the snapshot to **/
            std::string m_save_filename;
            /** The current HUD info. **/
            HUDInfo m_hud;
            /** Arrow indicating movement **/
            navigation::Point3D m_arrow;
            /** Detected objects **/
            std::vector<ObjectInfo> m_detected;
            /** List of glyphs **/
            std::vector<CameraGlyph> m_glyphs;
            /** Colour lookup thresholding table **/
            uint8_t m_lookup_threshold[THRESH_SIZE][THRESH_SIZE][THRESH_SIZE];

            int INPUT_WIDTH, INPUT_HEIGHT, PROCESS_WIDTH, PROCESS_HEIGHT;
            int STREAM_WIDTH, STREAM_HEIGHT, PIXEL_SKIP, PIXEL_THRESHOLD;
            int LEARN_SIZE;

#ifdef IS_ON_PI
            omxcv::OmxCv *m_enc;
#endif

            void LoadGlyphs(Options *opts);

            void ProcessImages(void);
            void DrawHUD(cv::Mat& img);
            void DrawCrosshair(cv::Mat& img, cv::Point centre, const cv::Scalar& colour, int size);
            void DrawTrackingArrow(cv::Mat& img);

            void RGB2HSV(uint8_t r, uint8_t g, uint8_t b, uint8_t *h, uint8_t *s, uint8_t *v);
            void BuildThreshold(uint8_t lookup[][THRESH_SIZE][THRESH_SIZE], HueThresholds thresh);
            void Threshold(const cv::Mat& src, cv::Mat &out, int width);
            void LearnThresholds(cv::Mat& src, cv::Mat& threshold, cv::Rect roi);
            bool CentreOfMass(cv::Mat& src, cv::Mat& threshold);
            int ConnectedComponents(cv::Mat& src, cv::Mat& threshold);
            bool CamShift(cv::Mat& src, cv::Mat& threshold);
            bool CannyGlyphDetection(cv::Mat& src, cv::Mat& proc);
            bool ThresholdingGlyphDetection(cv::Mat& src, cv::Mat& proc);
            bool GlyphContourDetection(cv::Mat& src, std::vector<std::vector<cv::Point>> contours) ;

            /** Copy constructor (disabled) **/
            CameraStream(const CameraStream &other);
            /** Assignment operator (disabled) **/
            CameraStream& operator= (const CameraStream &other);
    };
}

#endif // _PICOPTERX_CAMERA_STREAM_H