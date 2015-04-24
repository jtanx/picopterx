/**
 * @file    camera_stream.cpp
 * @author	Michael Baxter	<20503664@student.uwa.edu.au>
 * @author  Jeremy Tan <20933708@student.uwa.edu.au>
 * 
 * Camera functions
 **/
 
#include "common.h"
#include "camera_stream.h"
#include <queue>

using namespace picopter;
using picopter::navigation::Point2D;
using std::this_thread::sleep_for;
using std::chrono::steady_clock;
using std::chrono::duration_cast;
using std::chrono::seconds;
using std::chrono::milliseconds;

#define BLACK 0
#define WHITE 255

typedef struct {
    int M00;
    int M01;
    int M10;
} CamComponent;

typedef struct {
    int x;
    int y;
} vec2;


CameraStream::CameraStream(Options *opts)
: m_stop{false}
, m_mode(MODE_NO_PROCESSING)
{
    Options clear;
    if (!opts) {
        opts = &clear;
    }

    opts->SetFamily("CAMERA_STREAM");
    this->MIN_HUE		= opts->GetInt("MIN_HUE", 340);
    this->MAX_HUE		= opts->GetInt("MAX_HUE", 20);
    this->MIN_SAT		= opts->GetInt("MIN_SAT", 95);
    this->MAX_SAT		= opts->GetInt("MAX_SAT", 255);
    this->MIN_VAL		= opts->GetInt("MIN_VAL", 127);
    this->MAX_VAL		= opts->GetInt("MAX_VAL", 255);
    this->PIXEL_THRESHOLD	= opts->GetInt("PIXEL_THRESHOLD", 30);
    
    this->PROCESS_IMAGE_REDUCE	= opts->GetReal("PROCESS_IMAGE_REDUCE", 0.5);
    this->STREAM_IMAGE_REDUCE	= opts->GetReal("STREAM_IMAGE_REDUCE", 0.5);
    this->PIXEL_SKIP = (int)(1/PROCESS_IMAGE_REDUCE);
    
    this->BOX_SIZE		= opts->GetReal("BOX_SIZE", 1.5);
    
    this->THREAD_SLEEP_TIME = opts->GetInt("THREAD_SLEEP_TIME", 5);
    
    this->DILATE_ELEMENT = opts->GetInt("DILATE_ELEMENT", 8);
    this->ERODE_ELEMENT = opts->GetInt("ERODE_ELEMENT", 8);
    
    build_lookup_reduce_colourspace(lookup_reduce_colourspace);
    build_lookup_threshold(lookup_threshold, MIN_HUE, MAX_HUE, MIN_SAT, MAX_SAT, MIN_VAL, MAX_VAL);
    buildColours(&windowColours);
    
    m_capture = cvCreateCameraCapture(-1);
    if (m_capture == NULL) {
        Log(LOG_WARNING, "cvCreateCameraCapture failed.");
        throw std::invalid_argument("Could not open camera stream.");
    }
    cvSetCaptureProperty(m_capture, CV_CAP_PROP_FRAME_WIDTH, CAMERA_WIDTH);
    cvSetCaptureProperty(m_capture, CV_CAP_PROP_FRAME_HEIGHT, CAMERA_HEIGHT);
    cvSetCaptureProperty(m_capture, CV_CAP_PROP_FPS, 30);
    
    this->frame_counter = -1;
    this->m_fps = -1;
    
    this->m_save_photo = false;
    this->m_save_filename = "image.jpg";
    
    opts->SetFamily("GLOBAL");
    this->m_demo = opts->GetBool("DEMO_MODE", false);
    //cv::namedWindow("TEST", cv::WINDOW_AUTOSIZE);
}

CameraStream::CameraStream() : CameraStream(NULL) {}

CameraStream::~CameraStream() {
    Stop();
    if (m_worker_thread.valid()) {
        m_worker_thread.wait();
    }
    cvReleaseCapture(&m_capture);
}

bool CameraStream::Start() {
    //Check if we're already running
    std::lock_guard<std::mutex> lock(m_worker_mutex);
    
    if(m_worker_thread.valid() && 
       m_worker_thread.wait_for(milliseconds(0)) != std::future_status::ready) {
    
        return false;
    }
    
    m_stop = false;
    m_worker_thread = std::async(std::launch::async, &CameraStream::ProcessImages, this);
    return true;
}

void CameraStream::Stop() {
    m_stop = true;
}

CameraStream::CameraMode CameraStream::GetMode() {
    std::lock_guard<std::mutex> lock(m_worker_mutex);
    return m_mode;
}


void CameraStream::SetMode(CameraMode mode) {
    std::lock_guard<std::mutex> lock(m_worker_mutex);
    m_mode = mode;
}
    
void CameraStream::GetDetectedObjects(std::vector<Point2D> *list) {
    std::lock_guard<std::mutex> lock(m_worker_mutex);
    *list = redObjectList;
}

double CameraStream::GetFramerate() {
    std::lock_guard<std::mutex> lock(m_worker_mutex);
    return m_fps;
}

void CameraStream::TakePhoto(std::string filename) {
    std::lock_guard<std::mutex> lock(m_worker_mutex);
    if(!filename.empty()) {
        m_save_filename = filename;
    }
    m_save_photo = true;
}

void CameraStream::ProcessImages() {
    auto start_time = steady_clock::now();
    int frame_duration;
    frame_counter = 0;
    
    while(!m_stop) {
        std::unique_lock<std::mutex> lock(m_worker_mutex);
        
        /*----------------------*
         *      Load image      *
         *----------------------*/

        IplImage* image_raspi = cvQueryFrame(m_capture);
        cv::Mat image = cv::Mat(image_raspi);

        /*----------------------*
         *     Process image    *
         *----------------------*/
        switch(m_mode) {
            case MODE_NO_PROCESSING:	//No image processing
            default:
                drawCrosshair(image);
                break;
            
            case MODE_COM:	//Center of mass detection
                if(centerOfMass(image)) {
                    drawObjectMarker(image, cv::Point(redObjectList[0].x+image.cols/2, -redObjectList[0].y+image.rows/2), cv::Scalar(0, 0, 0));
                }
                drawCrosshair(image);
                break;
            
            case MODE_CAMSHIFT:	//Center of mass detection
                if(camShift(image)) {
                    drawObjectMarker(image, cv::Point(redObjectList[0].x+image.cols/2, -redObjectList[0].y+image.rows/2), cv::Scalar(0, 0, 0));
                }
                drawBox(image, cv::Point(windowList[0].x, windowList[0].y), cv::Point(windowList[0].x+windowList[0].w, windowList[0].y+windowList[0].l), cv::Scalar(255, 255, 255));
                drawCrosshair(image);
                break;
                
            case MODE_CONNECTED_COMPONENTS:
                if(ConnectedComponents(image) > 0) {
                    for(std::vector<Point2D>::size_type i=0; i<redObjectList.size(); i++) {
                        cv::Scalar colour = cv::Scalar(255, 255, 255);
                        if(i < windowColours.size()) {
                            colour = windowColours[i];
                        }
                        drawBox(image, cv::Point(windowList[i].x, windowList[i].y), cv::Point(windowList[i].x+windowList[i].w, windowList[i].y+windowList[i].l), colour);
                    }
                }
                drawCrosshair(image);
                break;
        }
        
        /*----------------------*
         *     Stream image     *
         *----------------------*/
        
        //cv::Mat frame;
        //cv::resize(image, frame, cv::Size(), STREAM_IMAGE_REDUCE, STREAM_IMAGE_REDUCE, cv::INTER_NEAREST);
        //drawFramerate(frame);
        //cv::VideoWriter outStream(STREAM_FILE, CV_FOURCC('M','J','P','G'), 4, frame.size(), true);
        //if(outStream.isOpened()) {
        //	outStream.write(frame);
        //}
        
        drawFramerate(image);
        // Only write the image out for web streaming every 5th frame
        if ((frame_counter % 5) == 0) {
            cv::VideoWriter outStream(STREAM_FILE, CV_FOURCC('M','J','P','G'), 4, image.size(), true);
            if(outStream.isOpened()) {
                outStream.write(image);
            }
        }
        
        //Are we in demo mode? If so, display the image on the screen.
        //Trying to do this when no X server is available will crash the program.
        if (m_demo) {
            cv::imshow("Camera stream", image);
            cv::waitKey(1);
        }
        
        frame_counter++;
        frame_duration = duration_cast<milliseconds>(steady_clock::now() - start_time).count();
        if (frame_duration > 1000) {
            m_fps = (frame_counter * 1000.0) / frame_duration;
            frame_counter = 0;
            start_time = steady_clock::now();
        }
        
        /*----------------------*
         *         Sleep        *
         *----------------------*/
        
        lock.unlock();
        
        if(THREAD_SLEEP_TIME > 0) {
            sleep_for(milliseconds(THREAD_SLEEP_TIME));
        }
    }
}

bool CameraStream::centerOfMass(cv::Mat& Isrc) {
    redObjectList.clear();
    Point2D object;
    
    int nRows = Isrc.rows;
    int nCols = Isrc.cols;
    int nChannels = Isrc.channels();
    
    int M00 = 0;		//Mxy
    int M01 = 0;
    int M10 = 0;

    int i, j, k;		//k = 3*i
    uchar* p;
    for(j=0; j<nRows; j += PIXEL_SKIP) {
        p = Isrc.ptr<uchar>(j);
        for (i=0; i<nCols; i += PIXEL_SKIP) {
            k = i*nChannels;
            if(lookup_threshold[lookup_reduce_colourspace[p[k+2]]][lookup_reduce_colourspace[p[k+1]]][lookup_reduce_colourspace[p[k]]]) {
                M00 += 1;
                M01 += j;
                M10 += i;
            }
        }
    }
    if(M00 > PIXEL_THRESHOLD) {
        object.x = M10/M00 - nCols/2;
        object.y = -(M01/M00 - nRows/2);
        redObjectList.push_back(object);
        return true;
    } else {
        return false;
    }
}

bool CameraStream::camShift(cv::Mat& Isrc) {
    Point2D object;
    CamWindow window;
    
    if(windowList.empty()) {
        window.x = 0;
        window.y = 0;
        window.l = Isrc.rows;
        window.w = Isrc.cols;
    } else {
        window = windowList.front();
    }
    
    redObjectList.clear();
    windowList.clear();
    
    int rowStart = window.y;
    int rowEnd = rowStart + window.l;
    int colStart = window.x;
    int colEnd = colStart + window.w;
    int nChannels = Isrc.channels();
    
    
    int M00 = 0;	//Mxy
    int M01 = 0;
    int M10 = 0;

    int i, j, k;
    uchar* p;
    for(j=rowStart; j<rowEnd; j += PIXEL_SKIP) {
        p = Isrc.ptr<uchar>(j);
        for (i=colStart; i<colEnd; i += PIXEL_SKIP) {
            k = i*nChannels;
            if(lookup_threshold[lookup_reduce_colourspace[p[k+2]]][lookup_reduce_colourspace[p[k+1]]][lookup_reduce_colourspace[p[k]]]) {
                M00 += 1;
                M01 += j;
                M10 += i;
            }
        }
    }

    if(M00 > PIXEL_THRESHOLD) {
        
        object.x = M10/M00 - Isrc.cols/2;
        object.y = -(M01/M00 - Isrc.rows/2);
        
        int l = (int)(BOX_SIZE*PIXEL_SKIP*sqrt(M00));
        int w = l;
        
        window.x = std::max(M10/M00 - w/2, 0);
        window.w = std::min(w, Isrc.cols-window.x);
        window.y = std::max(M01/M00 - l/2, 0);
        window.l = std::min(l, Isrc.rows-window.y);
        
        redObjectList.push_back(object);
        windowList.push_back(window);
        
        return true;
    } else {
        windowList.push_back(window);
        return false;
    }
}

int CameraStream::connectComponents(cv::Mat& Isrc) {
    int nChannels = Isrc.channels();
    
    cv::Mat BW(Isrc.rows*PROCESS_IMAGE_REDUCE, Isrc.cols*PROCESS_IMAGE_REDUCE, CV_8U, cv::Scalar(BLACK));
    int nRows = BW.rows;
    int nCols = BW.cols;


    int i, j, k;		//k = 3*i
    uchar* p_src;
    uchar* p_BW;
    for(j=0; j<nRows; j++) {
        p_src = Isrc.ptr<uchar>(j*PIXEL_SKIP);
        p_BW = BW.ptr<uchar>(j);
        for (i=0; i<nCols; i++) {
            k = i*nChannels*PIXEL_SKIP;
            if(lookup_threshold[lookup_reduce_colourspace[p_src[k+2]]][lookup_reduce_colourspace[p_src[k+1]]][lookup_reduce_colourspace[p_src[k]]]) {
                p_BW[i] = WHITE;
            }
        }
    }
    
    cv::Mat elementDilate(DILATE_ELEMENT, DILATE_ELEMENT, CV_8U, cv::Scalar(255));
    cv::Mat elementErode(ERODE_ELEMENT, ERODE_ELEMENT, CV_8U, cv::Scalar(255));
    cv::dilate(BW, BW, elementDilate);
    cv::erode(BW, BW, elementErode);
   
    cv::Mat CC(nRows, nCols, CV_8U, cv::Scalar(0));
    std::queue<vec2> q;
    uchar label = 0;
    
    vec2 temp;
    
    int x, y;
    uchar* p_CC;
    for(j=0; j<nRows; j++) {
        p_BW = BW.ptr<uchar>(j);
        p_CC = CC.ptr<uchar>(j);
        for (i=0; i<nCols; i++) {
            if(p_BW[i] == WHITE && p_CC[i] == 0) {
                label++;	//new component
                p_CC[i] = label;
                temp.x = i;
                temp.y = j;
                q.push(temp);
            }
            while(!q.empty()) {
                x = q.front().x;
                y = q.front().y;
                
                //check pixel above
                if(y > 0 && BW.ptr<uchar>(y-1)[x] == WHITE && CC.ptr<uchar>(y-1)[x] == 0) {
                    CC.ptr<uchar>(y-1)[x] = label;
                    temp.x = x;
                    temp.y = y-1;
                    q.push(temp);
                }
                
                //check pixel left
                if(x > 0 && BW.ptr<uchar>(y)[x-1] == WHITE && CC.ptr<uchar>(y)[x-1] == 0) {
                    CC.ptr<uchar>(y)[x-1] = label;
                    temp.x = x-1;
                    temp.y = y;
                    q.push(temp);
                }
                
                //check pixel right
                if(x < nCols-1 && BW.ptr<uchar>(y)[x+1] == WHITE && CC.ptr<uchar>(y)[x+1] == 0) {
                    CC.ptr<uchar>(y)[x+1] = label;
                    temp.x = x+1;
                    temp.y = y;
                    q.push(temp);
                }
                
                //check pixel below
                if(y < nRows-1 && BW.ptr<uchar>(y+1)[x] == WHITE && CC.ptr<uchar>(y+1)[x] == 0) {
                    CC.ptr<uchar>(y+1)[x] = label;
                    temp.x = x;
                    temp.y = y+1;
                    q.push(temp);
                }
                
                q.pop();
            }
        }
    }
    int numComponents = (int)label;

    std::vector<CamComponent> comps(numComponents);
    for(k=0; k<numComponents; k++) {
        comps[k].M00 = 0;
        comps[k].M01 = 0;
        comps[k].M10 = 0;
    }
    

    uchar* p;
    for(j=0; j<nRows; j++) {
        p = CC.ptr<uchar>(j);
        for (i=0; i<nCols; i++) {
            if(p[i] > 0) {
                comps[p[i] -1].M00 += 1;
                comps[p[i] -1].M01 += j;
                comps[p[i] -1].M10 += i;
            }
        }
    }
    
    
    redObjectList.clear();
    windowList.clear();
    
    Point2D object;
    CamWindow window;
    
    nCols*= PIXEL_SKIP;
    nRows*= PIXEL_SKIP;
    
    for(k=0; k<numComponents; k++) {
        if(comps[k].M00 > PIXEL_THRESHOLD) {
            comps[k].M01 *= PIXEL_SKIP;
            comps[k].M10 *= PIXEL_SKIP;
            
            object.x = comps[k].M10/comps[k].M00 - nCols/2;
            object.y = -(comps[k].M01/comps[k].M00 - nRows/2);
            
            int w = (int)(BOX_SIZE*PIXEL_SKIP*sqrt(comps[k].M00));
            int l = w;
            
            window.x = std::max(comps[k].M10/comps[k].M00 - w/2, 0);
            window.w = std::min(w, nCols-window.x);
            window.y = std::max(comps[k].M01/comps[k].M00 - l/2, 0);
            window.l = std::min(l, nRows-window.y);
            
            redObjectList.push_back(object);
            windowList.push_back(window);
        }
    }
    return (int)redObjectList.size();
}

/**
 * Connected components V2
 */
int CameraStream::ConnectedComponents(cv::Mat& src) {
    int nChannels = src.channels();
    
    cv::Mat BW(src.rows*PROCESS_IMAGE_REDUCE, src.cols*PROCESS_IMAGE_REDUCE, CV_8U, cv::Scalar(BLACK));
    int nRows = BW.rows;
    int nCols = BW.cols;
    
    //Reduce (point resize) and threshold the image
    int i, j, k;		//k = 3*i
    uchar* p_src;
    uchar* p_BW;
    for(j=0; j<nRows; j++) {
        p_src = src.ptr<uchar>(j*PIXEL_SKIP);
        p_BW = BW.ptr<uchar>(j);
        for (i=0; i<nCols; i++) {
            k = i*nChannels*PIXEL_SKIP;
            if(lookup_threshold[lookup_reduce_colourspace[p_src[k+2]]][lookup_reduce_colourspace[p_src[k+1]]][lookup_reduce_colourspace[p_src[k]]]) {
                p_BW[i] = WHITE;
            }
        }
    }
    
    //Blur, dilate and erode the image
    cv::Mat elementDilate(DILATE_ELEMENT, DILATE_ELEMENT, CV_8U, cv::Scalar(255));
    cv::Mat elementErode(ERODE_ELEMENT, ERODE_ELEMENT, CV_8U, cv::Scalar(255));
    cv::blur(BW,BW,cv::Size(3,3)); //MOAR BLUR
    cv::dilate(BW, BW, elementDilate);
    cv::erode(BW, BW, elementErode);
    
    //Find the contours (connected components)
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(BW, contours, cv::RETR_LIST, cv::CHAIN_APPROX_SIMPLE);
    
    //Calculate the contour moments
    std::vector<cv::Moments> comps(contours.size());
    for(size_t i = 0; i < contours.size(); i++) {
        comps[i] = cv::moments( contours[i], true);
    }
    //Sort moments in order of decreasing size (largest first)
    std::sort(comps.begin(), comps.end(), 
    [] (const cv::Moments &a, const cv::Moments &b) {
        return (int)b.m00 < (int)a.m00;
    });
    
    redObjectList.clear();
    windowList.clear();
    
    Point2D object;
    CamWindow window;
    
    nCols*= PIXEL_SKIP;
    nRows*= PIXEL_SKIP;
    
    //Calculate the locations on the original image
    for(k=0; k < (int)comps.size(); k++) {
        if(comps[k].m00 > PIXEL_THRESHOLD) {
            comps[k].m01 *= PIXEL_SKIP;
            comps[k].m10 *= PIXEL_SKIP;
            
            object.x = comps[k].m10/comps[k].m00 - nCols/2;
            object.y = -(comps[k].m01/comps[k].m00 - nRows/2);
            
            int w = (int)(BOX_SIZE*PIXEL_SKIP*sqrt(comps[k].m00));
            int l = w;
            
            window.x = std::max((int)(comps[k].m10/comps[k].m00 - w/2), 0);
            window.w = std::min(w, nCols-window.x);
            window.y = std::max((int)(comps[k].m01/comps[k].m00 - l/2), 0);
            window.l = std::min(l, nRows-window.y);
            
            redObjectList.push_back(object);
            windowList.push_back(window);
        }
    }
    return (int)redObjectList.size();
}

void CameraStream::RGB2HSV(int r, int g, int b, int *h, int *s, int *v) {
    int Vmax = std::max(r, std::max(g, b));
    int Vmin = std::min(r, std::min(g, b));

    *v = Vmax;
    
    int delta = Vmax - Vmin;
    
    if (Vmax != 0) {
        *s = CHAR_SIZE*delta/Vmax;
    } else {
        *s = 0;
        *h = -1;
    }
    
    if(delta == 0) delta = 1;
    
    if(r == Vmax) {
        *h = 60 * (g-b)/delta;
    } else if(g == Vmax) {
        *h = 120 + 60 * (b-r)/delta;
    } else {
        *h = 240 + 60 * (r-g)/delta;
    }
    
    if (*h < 0) *h += 360;
}

void CameraStream::build_lookup_threshold(uchar lookup_threshold[][LOOKUP_SIZE][LOOKUP_SIZE], int minHue, int maxHue, int minSat, int maxSat, int minVal, int maxVal) {
    int r, g, b, h, s, v;
    for(r=0; r<LOOKUP_SIZE; r++) {
        for(g=0; g<LOOKUP_SIZE; g++) {
            for(b=0; b<LOOKUP_SIZE; b++) {
                RGB2HSV(unreduce(r), unreduce(g), unreduce(b), &h, &s, &v);
                
                if(v < minVal || v > maxVal) {
                    lookup_threshold[r][g][b] = 0;
                } else if(s < minSat || s > maxSat) {
                    lookup_threshold[r][g][b] = 0;
                } else if(minHue < maxHue && (h > minHue && h < maxHue)) {
                    lookup_threshold[r][g][b] = 1;
                } else if(minHue > maxHue && (h > minHue || h < maxHue)) {
                    lookup_threshold[r][g][b] = 1;
                } else {
                    lookup_threshold[r][g][b] = 0;
                }
            }
        }
    }
}


void CameraStream::build_lookup_reduce_colourspace(uchar lookup_reduce_colourspace[]) {
    for (int i=0; i<CHAR_SIZE; i++) {
        lookup_reduce_colourspace[i] = (uchar)(i*(LOOKUP_SIZE-1)/(CHAR_SIZE-1));
    }
}

int CameraStream::unreduce(int x) {
    return (x*(CHAR_SIZE-1) + (CHAR_SIZE-1)/2) / LOOKUP_SIZE;
}

void CameraStream::drawCrosshair(cv::Mat& img) {
    int thickness = 2;
    int lineType = 8;
    int size = 20;
    cv::Point cross_points[4];
    cross_points[0] = cv::Point(img.cols/2 - size,	img.rows/2);
    cross_points[1] = cv::Point(img.cols/2 + size,	img.rows/2);
    cross_points[2] = cv::Point(img.cols/2,	img.rows/2 - size);
    cross_points[3] = cv::Point(img.cols/2,	img.rows/2 + size);
    for(int i=0; i<4; i+=2) {
        cv::line(img, cross_points[i], cross_points[i+1], cv::Scalar(255, 255, 255), thickness, lineType);
    }
}

void CameraStream::drawObjectMarker(cv::Mat& img, cv::Point centre, cv::Scalar colour) {
    int thickness = 2;
    int lineType = 8;
    cv::Point cross_points[4];
    cross_points[0] = cv::Point(centre.x,	0);
    cross_points[1] = cv::Point(centre.x,	img.rows);
    cross_points[2] = cv::Point(0,			centre.y);
    cross_points[3] = cv::Point(img.cols,	centre.y);
    for(int i=0; i<4; i+=2) {
        cv::line(img, cross_points[i], cross_points[i+1], colour, thickness, lineType);
    }
}

void CameraStream::drawBox(cv::Mat& img, cv::Point topLeft, cv::Point bottomRight, cv::Scalar colour) {
    //Thickness 2, lineType 8
    cv::rectangle(img, topLeft, bottomRight, colour, 2, 8);
}

void CameraStream::drawFramerate(cv::Mat& img) {
    //time(&end_time);
    char string_buf[128];
    sprintf(string_buf, "%3.4f fps", m_fps);
    //printf("%3.4f\n", fps);
    
    int thickness = 1;
    int lineType = 8;
    cv::Point TL_corner(30*img.cols/100, 90*img.rows/100);
    cv::putText(img, string_buf, TL_corner, CV_FONT_HERSHEY_PLAIN, 1, cv::Scalar(255, 255, 255), thickness, lineType);
}

void CameraStream::buildColours(std::vector<cv::Scalar> *colourList) {
    colourList->clear();
    colourList->push_back(cv::Scalar(0, 0, 255));
    colourList->push_back(cv::Scalar(255, 0, 0));
    colourList->push_back(cv::Scalar(0, 255, 255));
    colourList->push_back(cv::Scalar(255, 0, 255));
    colourList->push_back(cv::Scalar(255, 255, 0));
    colourList->push_back(cv::Scalar(0, 255, 0));
    colourList->push_back(cv::Scalar(0, 255, 0));
}
