/**
 * @file camera_stream.cpp
 * @brief Camera interaction functions. Monster class for camera processing.
 */

#include "common.h"
#include "camera_stream.h"
#include "flightboard.h"

#define BLACK 0
#define WHITE 255

using namespace picopter;
using namespace picopter::navigation;
using std::this_thread::sleep_for;
using std::chrono::steady_clock;
using std::chrono::duration_cast;
using std::chrono::seconds;
using std::chrono::milliseconds;
using std::chrono::microseconds;

#ifdef IS_ON_PI
    using namespace omxcv;
#endif

const std::vector<cv::Scalar> CameraStream::m_colours {
    cv::Scalar(255, 0, 0), cv::Scalar(0, 255, 0), cv::Scalar(0, 0, 255),
    cv::Scalar(255, 255, 0), cv::Scalar(0, 255, 255), cv::Scalar(255, 0, 255)
};

/**
 * Constructor. Creates a new camera stream.
 * @param [in] opts A pointer to options, if any (NULL for defaults).
 */
CameraStream::CameraStream(Options *opts)
: m_capture(-1)
, m_stop{false}
, m_mode(MODE_NO_PROCESSING)
, m_pool(4)
, m_fps(-1)
, m_show_backend(false)
, m_save_photo(false)
, m_hud{}
, m_arrow{}
{
    Options clear;
    if (!opts) {
        opts = &clear;
    }

    //Load the glyphs, if any.
    LoadGlyphs(opts);

    //Set the input/processing/streaming width parameters
    opts->SetFamily("CAMERA_STREAM");
    INPUT_WIDTH   = opts->GetInt("INPUT_WIDTH", 320);
    INPUT_HEIGHT  = opts->GetInt("INPUT_HEIGHT", 240);
    PROCESS_WIDTH = opts->GetInt("PROCESS_WIDTH", 160);
    STREAM_WIDTH  = opts->GetInt("STREAM_WIDTH", 320);
    LEARN_SIZE    = picopter::clamp(opts->GetInt("LEARN_SIZE", 50), 20, 100);

    //Set the default hue thresholds
    m_thresholds.p1_min = opts->GetInt("MIN_HUE", -10);
    m_thresholds.p1_max = opts->GetInt("MAX_HUE", 10);
    m_thresholds.p2_min = opts->GetInt("MIN_SAT", 95);
    m_thresholds.p2_max = opts->GetInt("MAX_SAT", 255);
    m_thresholds.p3_min = opts->GetInt("MIN_VAL", 127);
    m_thresholds.p3_max = opts->GetInt("MAX_VAL", 255);
    m_thresholds.colourspace = THRESH_HSV;
    m_learning_thresholds.colourspace = THRESH_HSV;

   if (!m_capture.isOpened()) {
        Log(LOG_WARNING, "cv::VideoCapture failed.");
        throw std::invalid_argument("Could not open camera stream.");
    }
    m_capture.set(CV_CAP_PROP_FRAME_WIDTH, INPUT_WIDTH);
    m_capture.set(CV_CAP_PROP_FRAME_HEIGHT, INPUT_HEIGHT);
    m_capture.set(CV_CAP_PROP_FPS, 30);

    INPUT_WIDTH  = m_capture.get(CV_CAP_PROP_FRAME_WIDTH);
    INPUT_HEIGHT = m_capture.get(CV_CAP_PROP_FRAME_HEIGHT);

    //If process resolution is larger than input resolution, don't resize
    if (PROCESS_WIDTH > INPUT_WIDTH) {
        PROCESS_WIDTH = INPUT_WIDTH;
    }

    //Resolution dependent parameters
    STREAM_HEIGHT = (INPUT_HEIGHT * STREAM_WIDTH) / INPUT_WIDTH;
    PROCESS_HEIGHT = (INPUT_HEIGHT * PROCESS_WIDTH) / INPUT_WIDTH;
    PIXEL_THRESHOLD	= opts->GetInt("PIXEL_THRESHOLD", (30 * INPUT_WIDTH) / 320);
    PIXEL_SKIP = INPUT_WIDTH / PROCESS_WIDTH;

    //Initialise the thresholding lookup table
    BuildThreshold(m_lookup_threshold, m_thresholds);

    //Initialise the HOG detector
    m_hog.setSVMDetector(cv::HOGDescriptor::getDefaultPeopleDetector());

    //Determine if we're running in demo mode.
    opts->SetFamily("GLOBAL");
    m_demo_mode = opts->GetBool("DEMO_MODE", false);

#ifdef IS_ON_PI
    std::string f = GenerateFilename(
        PICOPTER_HOME_LOCATION "/videos", "save", ".mkv");
    Log(LOG_INFO, "Saving video to %s", f.c_str());
    try {
        m_enc = new OmxCv(f.c_str(), INPUT_WIDTH, INPUT_HEIGHT, (700 * INPUT_WIDTH)/320);
    } catch (std::invalid_argument e) {
        Log(LOG_WARNING, "Cannot save video: %s", e.what());
        m_enc = nullptr;
    }
#endif

    //Start the worker thread.
    m_worker_thread = std::async(std::launch::async,
        &CameraStream::ProcessImages, this);
}

/**
 * Destructor.
 * Stops the worker thread and closes the camera stream.
 */
CameraStream::~CameraStream() {
    m_stop = true;
    if (m_worker_thread.valid()) {
        m_worker_thread.wait();
    }

#ifdef IS_ON_PI
    delete m_enc;
#endif

    if (m_demo_mode) {
        //Gtk is crap so this doesn't actually do much.
        cv::destroyWindow("Thresholded image");
        cv::destroyWindow("Camera stream");
    }
}

/**
 * Retrieves a list of detected objects in the current field of view.
 * @param [in,out] objects The list of detected objects.
 */
void CameraStream::GetDetectedObjects(std::vector<ObjectInfo> *objects) {
    std::lock_guard<std::mutex> lock(m_worker_mutex);
    *objects = m_detected;
}

/**
 * Retrieves the current mode of the camera.
 * @return The current camera mode.
 */
CameraStream::CameraMode CameraStream::GetMode(void) {
    std::lock_guard<std::mutex> lock(m_worker_mutex);
    return m_mode;
}

/**
 * Sets the mode of the camera.
 * @param [in] mode The mode to set the camera to.
 */
CameraStream::CameraMode CameraStream::SetMode(CameraMode mode) {
    std::lock_guard<std::mutex> lock(m_worker_mutex);
    m_mode = mode;
    return m_mode;
}

/**
 * Retrieves the input image width.
 * @return The input width.
 */
int CameraStream::GetInputWidth() {
    return INPUT_WIDTH;
}

/**
 * Retrieves the input image height.
 * @return The input height.
 */
int CameraStream::GetInputHeight() {
    return INPUT_HEIGHT;
}

/**
 * Retrieves the current camera configuration.
 * @param [out] config The location to store the camera configuration.
 */
void CameraStream::GetConfig(Options *config) {
    std::lock_guard<std::mutex> lock(m_worker_mutex);
    config->SetFamily("CAMERA_STREAM");

    config->Set("THRESH_COLOURSPACE", m_thresholds.colourspace);
    if (m_thresholds.colourspace == THRESH_HSV) {
        config->Set("MIN_HUE", m_thresholds.p1_min);
        config->Set("MAX_HUE", m_thresholds.p1_max);
        config->Set("MIN_SAT", m_thresholds.p2_min);
        config->Set("MAX_SAT", m_thresholds.p2_max);
        config->Set("MIN_VAL", m_thresholds.p3_min);
        config->Set("MAX_VAL", m_thresholds.p3_max);
    } else if (m_thresholds.colourspace == THRESH_YCbCr) {
        config->Set("MIN_Y", m_thresholds.p1_min);
        config->Set("MAX_Y", m_thresholds.p1_max);
        config->Set("MIN_Cb", m_thresholds.p2_min);
        config->Set("MAX_Cb", m_thresholds.p2_max);
        config->Set("MIN_Cr", m_thresholds.p3_min);
        config->Set("MAX_Cr", m_thresholds.p3_max);
    }
    config->Set("SHOW_BACKEND", m_show_backend);
}

/**
 * Sets the current camera configuration.
 * @param [in] config The configuration to use.
 */
void CameraStream::SetConfig(Options *config) {
    std::lock_guard<std::mutex> lock(m_worker_mutex);
    bool refresh = false, decrease = false;
    int colourspace = m_thresholds.colourspace;

    config->SetFamily("CAMERA_STREAM");
    config->GetBool("SHOW_BACKEND", &m_show_backend);


    config->GetInt("THRESH_COLOURSPACE", &colourspace);
    switch(colourspace) {
        case THRESH_HSV:
            m_thresholds.colourspace = THRESH_HSV;
            refresh |= config->GetInt("MIN_HUE", &m_thresholds.p1_min, -180, 180);
            refresh |= config->GetInt("MAX_HUE", &m_thresholds.p1_max, 0, 180);
            refresh |= config->GetInt("MIN_SAT", &m_thresholds.p2_min, 0, 255);
            refresh |= config->GetInt("MAX_SAT", &m_thresholds.p2_max, 0, 255);
            refresh |= config->GetInt("MIN_VAL", &m_thresholds.p3_min, 0, 255);
            refresh |= config->GetInt("MAX_VAL", &m_thresholds.p3_max, 0, 255);
            break;
        case THRESH_YCbCr:
            m_thresholds.colourspace = THRESH_YCbCr;
            refresh |= config->GetInt("MIN_Y", &m_thresholds.p1_min, 0, 255);
            refresh |= config->GetInt("MAX_Y", &m_thresholds.p1_max, 0, 255);
            refresh |= config->GetInt("MIN_Cb", &m_thresholds.p2_min, 0, 255);
            refresh |= config->GetInt("MAX_Cb", &m_thresholds.p2_max, 0, 255);
            refresh |= config->GetInt("MIN_Cr", &m_thresholds.p3_min, 0, 255);
            refresh |= config->GetInt("MAX_Cr", &m_thresholds.p3_max, 0, 255);
            break;
    }

    m_learning_thresholds.colourspace = m_thresholds.colourspace;

    if (refresh) {
        BuildThreshold(m_lookup_threshold, m_thresholds);
    }

    if (config->GetBool("SET_LEARNING_SIZE", &decrease)) {
        if (decrease) {
            LEARN_SIZE = picopter::clamp(LEARN_SIZE-10, 10, 100);
        } else {
            LEARN_SIZE = picopter::clamp(LEARN_SIZE+10, 10, 100);
        }
    }
}

/**
 * Sets the heads-up display info.
 * @param [in] hud The HUD info.
 */
void CameraStream::SetHUDInfo(HUDInfo *hud) {
    std::lock_guard<std::mutex> lock(m_aux_mutex);
    m_hud = *hud;
}

/**
 * Perform camera auto learning.
 */
void CameraStream::DoAutoLearning() {
    std::lock_guard<std::mutex> lock(m_worker_mutex);
    if (m_mode == CameraMode::MODE_LEARN_COLOUR) {
        if (m_learning_thresholds.colourspace == THRESH_HSV) {
            m_thresholds.p1_min = m_learning_thresholds.p1_min;
            m_thresholds.p1_max = m_learning_thresholds.p1_max;
        } else if (m_learning_thresholds.colourspace == THRESH_YCbCr) {
            m_thresholds.p2_min = m_learning_thresholds.p2_min;
            m_thresholds.p2_max = m_learning_thresholds.p2_max;
            m_thresholds.p3_min = m_learning_thresholds.p3_min;
            m_thresholds.p3_max = m_learning_thresholds.p3_max;
        }
        BuildThreshold(m_lookup_threshold, m_thresholds);
    }
}

/**
 * Take a single photo.
 * @param [in] filename Location to store the photo.
 * @return true iff the photo will be taken. This function may return false if
 *         it is in the process of taking another photo.
 */
bool CameraStream::TakePhoto(std::string filename) {
    std::lock_guard<std::mutex> lock(m_worker_mutex);
    if (!m_save_photo && !filename.empty()) {
        m_save_filename = filename;
        m_save_photo = true;
        return true;
    }
    return false;
}

/**
 * Set an arrow to be displayed from the centre of the image.
 * @param [in] arrow The arrow vector, as percentage ([0-1] range for x,y,z)
 *             E.g. 100% vec.x draws an arrow from centre to the right side
 *             of the image.
 */
void CameraStream::SetTrackingArrow(navigation::Point3D arrow) {
    std::lock_guard<std::mutex> lock(m_aux_mutex);
    m_arrow = arrow;
}

//Private methods

/**
 * Worker thread. Processes the images as necessary.
 */
void CameraStream::ProcessImages() {
    static const std::vector<int> saveparams = {CV_IMWRITE_JPEG_QUALITY, 90};
    static const std::vector<int> streamparams {CV_IMWRITE_JPEG_QUALITY, 75};
    int frame_counter = 0, frame_duration = 0, skip_factor = 5;
    auto sampling_start = steady_clock::now();
#ifdef IS_ON_PI
    OmxCvJpeg *streamer = nullptr, *saver = nullptr;
    try {
        streamer = new OmxCvJpeg(STREAM_WIDTH, STREAM_HEIGHT, 75);
        saver = new OmxCvJpeg(INPUT_WIDTH, INPUT_HEIGHT, 90);
        skip_factor = 2;
    } catch (std::invalid_argument e) {
        Log(LOG_WARNING, "Cannot start hardware JPEG encoders: %s", e.what());
    }
#endif

    while (!m_stop) {
        std::unique_lock<std::mutex> lock(m_worker_mutex, std::defer_lock);
        cv::Mat image, backend;

        //Grab image
        m_capture >> image;

        //Acquire the mutex
        lock.lock();

        //Save it, if requested to.
        if (m_save_photo) {
#ifdef IS_ON_PI
            if (saver) {
                saver->Encode(m_save_filename.c_str(), image, true);
            } else
#endif
            cv::imwrite(m_save_filename, image, saveparams);
            m_save_photo = false;
         }

        //Process image
        switch(m_mode) {
            case MODE_NO_PROCESSING:	//No image processing
            default:
                break;
            case MODE_LEARN_COLOUR: {
                int lwidth = (LEARN_SIZE*image.cols)/100, lheight = (LEARN_SIZE*image.rows)/100;
                cv::Rect roi((image.cols - lwidth)/2, (image.rows - lheight)/2,
                    lwidth, lheight);
                Threshold(image, backend, PROCESS_WIDTH);
                LearnThresholds(image, backend, roi);

                cv::rectangle(image,roi.tl(), roi.br(), cv::Scalar(255, 255, 255));
            }   break;
            case MODE_COM:
                if (CentreOfMass(image, backend)) {
                    DrawCrosshair(image,
                        cv::Point(m_detected[0].position.x + image.cols/2,
                            -m_detected[0].position.y + image.rows/2),
                            cv::Scalar(0,0,0), 100);
                }
                break;
            case MODE_CAMSHIFT:
                if (CamShift(image, backend)) {
                    cv::rectangle(image, m_detected[0].bounds.tl(),
                            m_detected[0].bounds.br(), m_colours[0]);
                }
                break;
            case MODE_CONNECTED_COMPONENTS:
                if(ConnectedComponents(image, backend) > 0) {
                    for(size_t i=0; i < m_detected.size(); i++) {
                        cv::rectangle(image, m_detected[i].bounds.tl(),
                            m_detected[i].bounds.br(),
                            m_colours[i%m_colours.size()], 2);
                    }
                }
                break;
            case MODE_CANNY_GLYPH:
                if (CannyGlyphDetection(image, backend)) {
                    for(size_t i=0; i < m_detected.size(); i++) {
                        cv::rectangle(image, m_detected[i].bounds.tl(),
                            m_detected[i].bounds.br(),
                            m_colours[i%m_colours.size()], 2);
                    }
                }
            break;
            case MODE_THRESH_GLYPH:
                if (ThresholdingGlyphDetection(image, backend)) {
                    for(size_t i=0; i < m_detected.size(); i++) {
                        cv::rectangle(image, m_detected[i].bounds.tl(),
                            m_detected[i].bounds.br(),
                            m_colours[i%m_colours.size()], 2);
                    }
                }
            break;
            case MODE_HOUGH:
                if (HoughDetection(image, backend)) {
                    //??????
                }
            break;
            case MODE_HOG_PEOPLE:
                if (HOGPeople(image, backend)) {
                    for (size_t i = 0; i < m_detected.size(); i++) {
                        cv::rectangle(image, m_detected[i].bounds.tl(),
                            m_detected[i].bounds.br(),
                            m_colours[i%m_colours.size()], 2);
                    }
                }
           break;
        }

        DrawCrosshair(image, cv::Point(image.cols/2, image.rows/2),
            cv::Scalar(255, 255, 255), 20);
        // Draw an arrow on the image (for displaying where it wants to go for object tracking)
        DrawTrackingArrow(image);

        //Overlay the HUD
        DrawHUD(image);

        //Are we in demo mode? If so, display the image on the screen.
        //Trying to do this when no X server is available will crash the program.
        if (m_demo_mode) {
            cv::imshow("Camera stream", image);
            cv::waitKey(1);
        }

#ifdef IS_ON_PI
        if (m_enc) {
            m_enc->Encode(image);
        }
#endif
        //Stream image
        //Only write the image out for web streaming every 5th frame
        if ((frame_counter % skip_factor) == 0) {
            if (m_show_backend) {
                cv::imwrite(STREAM_FILE, backend, streamparams);
            } else {
                if (STREAM_WIDTH < INPUT_WIDTH) {
                    cv::resize(image, image,
                        cv::Size(STREAM_WIDTH, STREAM_HEIGHT));
                }
#ifdef IS_ON_PI
                if (streamer) {
                    streamer->Encode(STREAM_FILE, image);
                } else
#endif
                cv::imwrite(STREAM_FILE, image, streamparams);
            }
        }

        //Update frame rate
        frame_counter++;
        frame_duration = duration_cast<milliseconds>(steady_clock::now() - sampling_start).count();
        if (frame_duration > 1000) {
            m_fps = (frame_counter * 1000.0) / frame_duration;
            frame_counter = 0;
            sampling_start = steady_clock::now();
            //printf("%f\n", m_fps);
            //Log(LOG_INFO, "FPS: %.2f", m_fps);
        }
    }
#ifdef IS_ON_PI
    delete streamer;
    delete saver;
#endif
}

/**
 * Retrieves the current frame rate.
 * @return The current frame rate, in frames per second, or -1.0 if unknown.
 */
double CameraStream::GetFramerate() {
    std::lock_guard<std::mutex> lock(m_worker_mutex);
    return m_fps;
}

/**
 * Draws a heads-up display onto the frame.
 * @param [in] img The image to draw the HUD onto.
 */
void CameraStream::DrawHUD(cv::Mat& img) {
    std::unique_lock<std::mutex> lock(m_aux_mutex);
    HUDInfo hud = m_hud;
    lock.unlock();

    char string_buf[128];
    time_t ts = time(NULL) + hud.unix_time_offset;
    struct tm tsp;

    //Enter the time
    localtime_r(&ts, &tsp);
    strftime(string_buf, sizeof(string_buf), "%H:%M:%S", &tsp);
    cv::putText(img, string_buf, cv::Point(70*img.cols/100, 5*img.rows/100),
        cv::FONT_HERSHEY_SIMPLEX, 0.34, cv::Scalar(255, 255, 255), 1, 8);
    strftime(string_buf, sizeof(string_buf), "%d-%m-%Y", &tsp);
    cv::putText(img, string_buf, cv::Point(70*img.cols/100, 10*img.rows/100),
        cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255, 255, 255), 1, 8);
    //Enter the FPS
    sprintf(string_buf, "%3.4f fps", m_fps);
    cv::putText(img, string_buf, cv::Point(70*img.cols/100, 15*img.rows/100),
        cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255, 255, 255), 1, 8);
    //Enter the LIDAR range
    sprintf(string_buf, "L: %.2fm", hud.lidar);
    cv::putText(img, string_buf, cv::Point(70*img.cols/100, 20*img.rows/100),
        cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255, 255, 255), 1, 8);
    sprintf(string_buf, "P: %.1f, R: %.1f", hud.gimbal.pitch, hud.gimbal.roll);
    cv::putText(img, string_buf, cv::Point(70*img.cols/100, 25*img.rows/100),
        cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255, 255, 255), 1, 8);

    //Enter the position
    sprintf(string_buf, "%.7f, %.7f", hud.pos.lat, hud.pos.lon);
    cv::putText(img, string_buf, cv::Point(5*img.cols/100, 5*img.rows/100),
        cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255, 255, 255), 1, 8);
    //Enter the altitude and climb rate
    sprintf(string_buf, "%.1fm, %.1fm/s", hud.pos.alt, hud.climb);
    cv::putText(img, string_buf, cv::Point(5*img.cols/100, 10*img.rows/100),
        cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255, 255, 255), 1, 8);
    //Enter the heading and throttle
    sprintf(string_buf, "%03d deg, %d%%", hud.heading, hud.throttle);
    cv::putText(img, string_buf, cv::Point(5*img.cols/100, 15*img.rows/100),
        cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255, 255, 255), 1, 8);
    //Enter the ground speed and air speed.
    sprintf(string_buf, "GS: %.1fm/s AS:%.1f m/s", hud.ground_speed, hud.air_speed);
    cv::putText(img, string_buf, cv::Point(5*img.cols/100, 20*img.rows/100),
        cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255, 255, 255), 1, 8);
    //Enter the battery statistics
    sprintf(string_buf, "%.2fV, %.1fA (%3d %%)",
        hud.batt_voltage, hud.batt_current, hud.batt_remaining);
    cv::putText(img, string_buf, cv::Point(5*img.cols/100, 25*img.rows/100),
        cv::FONT_HERSHEY_SIMPLEX, 0.32, cv::Scalar(255, 255, 255), 1, 8);

    if (hud.status2.size()) {
        cv::putText(img, hud.status2,  cv::Point(5*img.cols/100, 87*img.rows/100),
        cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255, 255, 255), 1, 8);
    }
    if (hud.status1.size()) {
        cv::putText(img, hud.status1, cv::Point(5*img.cols/100, 92*img.rows/100),
        cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255, 255, 255), 1, 8);
    }
}

/**
 * Draws a crosshair on the image.
 * @param [in] img The imge to draw on.
 * @param [in] centre The centre of the crosshair.
 * @param [in] colour The colour of the crosshair.
 * @param [in] size The size (radius) of the crosshair (in pixels).
 */
void CameraStream::DrawCrosshair(cv::Mat& img, cv::Point centre, const cv::Scalar& colour, int size) {
    cv::line(img, cv::Point(centre.x - size, centre.y),
        cv::Point(centre.x + size, centre.y), colour, 2);
    cv::line(img, cv::Point(centre.x, centre.y - size),
        cv::Point(centre.x, centre.y + size), colour, 2);
}

/**
 * Draws the image tracking arrow (if present).
 * @param [in] img The image to draw on.
 */
void CameraStream::DrawTrackingArrow(cv::Mat& img) {
    std::lock_guard<std::mutex> lock(m_aux_mutex);
    if (m_arrow.x != 0 || m_arrow.y != 0 ||  m_arrow.z != 0) {
        cv::Point centre(img.cols/2, img.rows/2);
        cv::Point end(img.cols/2 * (1 + m_arrow.x), img.rows/2 * (1 + m_arrow.y));
        //Draw strafing x-y movement
        cv::line(img, centre, end, cv::Scalar(255), 2);
        //Draw rotation z movement
        cv::ellipse(img, centre, cv::Size(10, 10), 0, 0, 180.0*m_arrow.z,
            cv::Scalar(255), 2);
    }
}

/**
 * Converts from RGB to HSV colourspace.
 * Uses the OpenCV convention of 0-180 for hue.
 *
 * @param [in] r The red value.
 * @param [in] g The green value.
 * @param [in] b The blue value;
 * @param [out] h The hue value (0-180).
 * @param [out] s The saturation value (0-255).
 * @param [out] v The value value (0-255).
 */
void CameraStream::RGB2HSV(uint8_t r, uint8_t g, uint8_t b, uint8_t *h, uint8_t *s, uint8_t *v) {
    uint8_t rgb_max = std::max(r, std::max(g, b));
    uint8_t rgb_min = std::min(r, std::min(g, b));
    uint8_t delta = rgb_max - rgb_min;

    *v = rgb_max;
    if (rgb_max != 0 && delta != 0) {
        *s = ((int)255*delta)/rgb_max;
    } else {
        *s = 0;
        *h = 0;
        return;
    }

    if(r == rgb_max) {
        *h = 43 * (g-b)/delta;
    } else if(g == rgb_max) {
        *h = 85 + 43 * (b-r)/delta;
    } else {
        *h = 171 + 43 * (r-g)/delta;
    }

    *h = (uint8_t)(((int)180*(*h))/255);
}

/**
 * Converts to the Y'CbCr colourspace.
 * @param [in] r The red value.
 * @param [in] g The green value.
 * @param [in] b The blue value.
 * @param [in] y The luma component (0-255).
 * @param [in] cb The blue difference component (0-255).
 * @param [in] cr The red difference component (0-255).
 * @return Return_Description
 */
void CameraStream::RGB2YCbCr(uint8_t r, uint8_t g, uint8_t b, uint8_t *y, uint8_t *cb, uint8_t *cr) {
    *y = 0.299 * r + 0.587 * g + 0.114 * b;
    *cb = -0.168736 * r - 0.331264 * g + 0.500 * b + 128;
    *cr = 0.500 * r - 0.418688 * g - 0.081312 * b + 128;
}

/**
 * Builds the threshold lookup table from the given thresholding parameters.
 * @param [out] lookup The lookup table.
 * @param [in] thresh The thresholding parameters.
 */
void CameraStream::BuildThreshold(uint8_t lookup[][THRESH_SIZE][THRESH_SIZE], ThresholdParams thresh) {
    uint8_t r, g, b;
    for(r = 0; r < THRESH_SIZE; r++) {
        for(g = 0; g < THRESH_SIZE; g++) {
            for(b = 0; b < THRESH_SIZE; b++) {
                lookup[r][g][b] = BLACK;

                if (thresh.colourspace == THRESH_HSV) {
                    uint8_t h, s, v;
                    RGB2HSV(UNREDUCE(r), UNREDUCE(g), UNREDUCE(b), &h, &s, &v);

                    if (v >= thresh.p3_min && v <= thresh.p3_max &&
                        s >= thresh.p2_min && s <= thresh.p2_max) {
                        if (thresh.p1_min < 0) {
                            if ((h >= thresh.p1_min+180 && h <= 180) ||
                                (h >= 0 && h <= thresh.p1_max)) {
                                    lookup[r][g][b] = WHITE;
                                }
                        } else if (h >= thresh.p1_min && h <= thresh.p1_max) {
                            lookup[r][g][b] = WHITE;
                        }
                    }
                } else if (thresh.colourspace == THRESH_YCbCr) {
                    uint8_t y, cb, cr;
                    RGB2YCbCr(UNREDUCE(r), UNREDUCE(g), UNREDUCE(b), &y, &cb, &cr);

                    if (y >= thresh.p1_min && y <= thresh.p1_max &&
                        cb >= thresh.p2_min && cb <= thresh.p2_max &&
                        cr >= thresh.p3_min && cr <= thresh.p3_max) {
                        lookup[r][g][b] = WHITE;
                    }
                }
            }
        }
    }
}

/**
 * Threshold a slice of a frame.
 * @param [in] src The source frame.
 * @param [in] out The destination frame.
 * @param [in] skip The pixel skip factor.
 * @param [in] offset The starting offset.
 * @param [in] slice_height The number of destination rows to process.
 */
void CameraStream::ThresholdSlice(const cv::Mat &src, cv::Mat &out, int skip, int offset, int slice_height) {
    const uint8_t *srcp;
    uint8_t *destp;
    int nChannels = src.channels();
    int i, j, k;
    
    for(j=offset; j < offset+slice_height; j++) {
        srcp = src.ptr<const uint8_t>(j*skip);
        destp = out.ptr<uint8_t>(j);
        for (i=0; i < out.cols; i++) {
            k = i*nChannels*skip;
            destp[i] = m_lookup_threshold[srcp[k+2]/THRESH_DIV][srcp[k+1]/THRESH_DIV][srcp[k]/THRESH_DIV];
        }
    }
}

/**
 * Thresholds the image using the preset LUT.
 * @param [in] src The input image.
 * @param [out] out The output image.
 * @param [in] width The output processing width.
 */
void CameraStream::Threshold(const cv::Mat& src, cv::Mat &out, int width) {
    //auto start = steady_clock::now();
    int skip = src.cols/width;
    out.create((src.rows * width) / src.cols, width, CV_8UC1);
    
    ThresholdSlice(src, out, skip, 0, out.rows);
    
    //GDB really does not like thread pooling and GDB will segfault
    //if threaded thresholding is enabled. Probably fixed by upgrading
    //GDB.
    /*
    if ((out.rows%4)) {
        ThresholdSlice(src, out, skip, 0, out.rows);
    } else {
        //Log(LOG_DEBUG, "SLICING");
        std::vector<std::future<void>> ret;
        for (int i = 0; i < 4; i++) {
            ret.emplace_back(m_pool.enqueue(&CameraStream::ThresholdSlice,
                this, src, out, skip, (i*out.rows)/4, out.rows/4));
        }
        for(auto &&result : ret){
            result.get();
        }
    }
    */
    //printf("%d\n", (int)duration_cast<microseconds>(steady_clock::now()-start).count());
}

/**
 * Do auto threshold learning.
 * @param [in] src The source image.
 * @param [in] threshold Location to store thresholded image.
 * @param [in] roi Region of interest to calculate thresholds.
 */
void CameraStream::LearnThresholds(cv::Mat& src, cv::Mat& threshold, cv::Rect roi) {
    cv::Mat sroi(src, roi);
    cv::medianBlur(sroi, sroi, 7);
    if (m_learning_thresholds.colourspace == THRESH_HSV) {
        cv::cvtColor(sroi, sroi, CV_BGR2HSV);
        std::vector<cv::Mat> channels;
        cv::split(sroi, channels);

        cv::Scalar mean_hue = cv::mean(channels[0]);
        m_learning_thresholds.p1_min = mean_hue[0]-10;
        m_learning_thresholds.p1_max = mean_hue[0]+10;
        if (m_learning_thresholds.p1_max > 180) {
            m_learning_thresholds.p1_max -= 180;
        }
        if (m_learning_thresholds.p1_min > m_learning_thresholds.p1_max) {
            m_learning_thresholds.p1_min -= 180;
        }
    } else if (m_learning_thresholds.colourspace == THRESH_YCbCr) {
        cv::cvtColor(sroi, sroi, CV_BGR2YCrCb);
        std::vector<cv::Mat> channels;
        cv::split(sroi, channels);

        cv::Scalar mean_cr = cv::mean(channels[1]);
        cv::Scalar mean_cb = cv::mean(channels[2]);

        m_learning_thresholds.p2_min = std::max((int)mean_cb[0]-15, 0);
        m_learning_thresholds.p2_max = std::min((int)mean_cb[0]+15, 255);

        m_learning_thresholds.p3_min = std::max((int)mean_cr[0]-15, 0);
        m_learning_thresholds.p3_max = std::min((int)mean_cr[0]+15, 255);
    }
    if (m_demo_mode) {
        cv::imshow("Thresholded image", threshold);
    }
}

/**
 * Simple centre of mass thresholding calculation.
 * @param [in] src The image to compute from.
 * @param [out] threshold The location to store the thresholded image.
 * @return true iff an object was detected.
 */
bool CameraStream::CentreOfMass(cv::Mat& src, cv::Mat& threshold) {
    cv::Moments m;
    Threshold(src, threshold, PROCESS_WIDTH);
    if (m_demo_mode) {
        cv::imshow("Thresholded image", threshold);
        cv::waitKey(1);
    }
    m_detected.clear();
    m = cv::moments(threshold, true);
    if(m.m00 > PIXEL_THRESHOLD) {
        ObjectInfo object = {0};
        object.image_width = INPUT_WIDTH;
        object.image_height = INPUT_HEIGHT;
        object.position.x = PIXEL_SKIP*m.m10/m.m00 - src.cols/2;
        object.position.y = -(PIXEL_SKIP*m.m01/m.m00 - src.rows/2);
        m_detected.push_back(object);
        return true;
    }
    return false;
}

/**
 * Connected components V2
 * Computes position of 4 largest blobs on the image.
 * @param [in] src The image to compute from.
 * @param [out] threshold The location to store the thresholded image. NB:
 *                        Due to the call to `findContours`, this will likely
 *                        contain a meaningless image, and will not accurately
 *                        display the thresholded image used. To view this,
 *                        switch to 'Centre of Mass' mode.
 * @return The number of objects detected, sorted by order of decreasing size.
 */
int CameraStream::ConnectedComponents(cv::Mat& src, cv::Mat& threshold) {
    Threshold(src, threshold, PROCESS_WIDTH);

    //Blur, dilate and erode the image
    cv::Mat elementDilate(8, 8, CV_8U, cv::Scalar(255));
    cv::Mat elementErode(8, 8, CV_8U, cv::Scalar(255));

    cv::dilate(threshold, threshold, elementDilate);
    cv::erode(threshold, threshold, elementErode);

    if (m_demo_mode) {
        cv::imshow("Thresholded image", threshold);
        cv::waitKey(1);
    }

    //Find the contours (connected components)
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(threshold, contours, cv::RETR_LIST, cv::CHAIN_APPROX_SIMPLE);

    //Calculate the contour moments
    typedef std::pair<std::vector<cv::Point>*, cv::Moments> ctm_t;
    std::vector<ctm_t> comps(contours.size());
    for(size_t i = 0; i < contours.size(); i++) {
        comps[i] = ctm_t(&contours[i], cv::moments(contours[i], true));
    }
    //Sort moments in order of decreasing size (largest first)
    std::sort(comps.begin(), comps.end(),
    [] (const ctm_t &a, const ctm_t &b) {
        return (int)b.second.m00 < (int)a.second.m00;
    });

    m_detected.clear();

    ObjectInfo object = {0};
    object.image_width = INPUT_WIDTH;
    object.image_height = INPUT_HEIGHT;

    int nCols = threshold.cols * PIXEL_SKIP;
    int nRows = threshold.rows * PIXEL_SKIP;

    //Calculate the locations on the original image (first 4 only)
    for(int k=0; k < 4 && k < (int)comps.size(); k++) {
        if(comps[k].second.m00 > PIXEL_THRESHOLD) {
            comps[k].second.m01 *= PIXEL_SKIP;
            comps[k].second.m10 *= PIXEL_SKIP;

            object.id = k;
            object.position.x = comps[k].second.m10/comps[k].second.m00 - nCols/2;
            object.position.y = -(comps[k].second.m01/comps[k].second.m00 - nRows/2);

            object.bounds = cv::boundingRect(*(comps[k].first));
            object.bounds.x *= PIXEL_SKIP;
            object.bounds.y *= PIXEL_SKIP;
            object.bounds.width *= PIXEL_SKIP;
            object.bounds.height *= PIXEL_SKIP;
            m_detected.push_back(object);
        }
    }
    return m_detected.size();
}

/**
 * An attempt at the Camshift algorithm.
 * @param [in] src The image to compute from.
 * @param [out] threshold The location to store the thresholded image.
 * @return true iff an object was detected.
 */
bool CameraStream::CamShift(cv::Mat& src, cv::Mat& threshold) {
    static cv::Rect roi_bounds(0,0,0,0); //FIXME make non-static
    static cv::Mat hist;
    static int capcount = 0;
    static int smin = 100, vmin = 130, vmax = 256;
    // we compute the histogram from the 0-th and 1-st channels
    int channels[] = {0, 1};
    // hue varies from 0 to 179, see cvtColor
    // saturation varies from 0 (black-gray-white) to
    // 255 (pure spectrum color)
    float hranges[] = {0,180}, sranges[] = {0,256};
    const float *ranges[] = {hranges, sranges};
    cv::TermCriteria tc(cv::TermCriteria::EPS|cv::TermCriteria::COUNT, 10, 1);

    if (capcount < 10 || roi_bounds.width <= 1 || roi_bounds.height <= 1) {
        if (ConnectedComponents(src, threshold)) {
            roi_bounds = m_detected[0].bounds;
            cv::Mat roi = src(roi_bounds), mask;
            cv::cvtColor(roi, roi, CV_BGR2HSV);
            cv::inRange(roi, cv::Scalar(0, smin, std::min(vmin, vmax)),
                        cv::Scalar(180, 256, std::max(vmin, vmax)), mask);
            // Quantize the hue to 30 levels and the saturation to 32 levels
            int histSize[] = {10,30};
            cv::calcHist(&roi, 1, channels, mask, hist, 1, histSize, ranges,
                true, false);
            if (m_demo_mode) {
                cv::imshow("Histogram", hist);
            }
            //cv::normalize(hist, hist, 0, 255, cv::NORM_MINMAX);
            capcount++;
        } else {
            capcount = 0;
        }
    } else {
        cv::Mat dst, mask;
        cv::cvtColor(src, dst, CV_BGR2HSV);
        cv::inRange(dst, cv::Scalar(0, smin, std::min(vmin, vmax)),
                        cv::Scalar(180, 256, std::max(vmin, vmax)), mask);
        cv::calcBackProject(&dst, 1, channels, hist, threshold, ranges);
        threshold &= mask;
        
        if (m_demo_mode) {
            cv::imshow("Thresholded image", threshold);
        }
        cv::RotatedRect rr = cv::CamShift(threshold, roi_bounds, tc);
        ObjectInfo object = {0};

        object.image_width = INPUT_WIDTH;
        object.image_height = INPUT_HEIGHT;
        object.position.x = rr.center.x - src.cols/2;
        object.position.y = -rr.center.y + src.rows/2;
        object.bounds = rr.boundingRect();

        //LogSimple(LOG_DEBUG, "X: %.1f, Y: %.1f, W: %d, H: %d", rr.center.x, rr.center.y, object.bounds.width, object.bounds.height);
        m_detected.clear();
        m_detected.push_back(object);
        return true;
    }

    return false;

}

/**
 * Uses the HOG descriptor to detect people.
 * @param [in] src The image to compute from.
 * @param [in] process The process buffer.
 * @return true iff people were detected.
 */
bool CameraStream::HOGPeople(cv::Mat &src, cv::Mat& process) {
    std::vector<cv::Rect> found;

    cv::resize(src, process, cv::Size(PROCESS_WIDTH, PROCESS_HEIGHT));
    cv::cvtColor(process, process, CV_BGR2GRAY);
    m_hog.detectMultiScale(process, found);
    m_detected.clear();

    if (m_demo_mode) {
        cv::imshow("Thresholded image", process);
    }

    for (size_t i = 0; i < found.size(); i++) {
        ObjectInfo object{};
        cv::Rect r(found[i].x*PIXEL_SKIP, found[i].y*PIXEL_SKIP,
                   found[i].width*PIXEL_SKIP, found[i].height*PIXEL_SKIP);

        object.image_width = INPUT_WIDTH;
        object.image_height = INPUT_HEIGHT;
        object.position.x = r.x + r.width/2 - src.cols/2;
        object.position.y = -(r.y + r.height/2) + src.rows/2;
        object.bounds = r;
        m_detected.push_back(object);
        Log(LOG_DEBUG, "DETECTED HOG");
    }

    return m_detected.size() > 0;
}
