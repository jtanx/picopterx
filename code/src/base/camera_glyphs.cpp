/**
 * @file camera_glyphs.cpp
 * @brief Glyph detection code.
 * @author Jeremy Tan
 * Loosely based on:
 * https://rdmilligan.wordpress.com/2015/07/19/glyph-recognition-using-opencv-and-python/
 */

#include "common.h"
#include "camera_stream.h"
#include <rapidjson/document.h>

using picopter::CameraStream;
using picopter::CameraGlyph;
using picopter::Options;
using namespace rapidjson;

#define GLYPH_BLACK_THRESHOLD 140

/**
 * Unpacks a glyph from the given options entry.
 * @param [in] val The entry to decode.
 * @param [in] closure Pointer to the glyph list.
 */
static void GlyphUnpickler(const void *val, void *closure) {
    Value *v = const_cast<Value*>(static_cast<const Value*>(val)); //...
    auto *glyphs = static_cast<std::vector<CameraGlyph>*>(closure);
    CameraGlyph g{};
    Value *vv;
    
    vv = static_cast<Value*>(Options::GetValue(v, "ID"));
    if (!vv || !vv->IsInt()) {
        Log(LOG_WARNING, "Ignoring glyph with unknown ID!");
    } else {
        g.id = vv->GetInt();
        vv = static_cast<Value*>(Options::GetValue(v, "PATH"));
        if (!vv || !vv->IsString()) {
            Log(LOG_WARNING, "Ignoring glyph %d with invalid path!", g.id);
        } else {
            g.path = vv->GetString();
            vv = static_cast<Value*>(Options::GetValue(v, "DESCRIPTION"));
            if (vv && vv->IsString()) {
                g.description = vv->GetString();
            }
            
            g.image = cv::imread(g.path);
            cv::cvtColor(g.image, g.image, CV_BGR2GRAY);
            cv::inRange(g.image, cv::Scalar(0), cv::Scalar(GLYPH_BLACK_THRESHOLD), g.image);
            if (g.image.data == NULL) {
                Log(LOG_WARNING, "Glyph %s does not exist! Skipping!", g.path.c_str());
            } else {
                glyphs->push_back(g);
                Log(LOG_INFO, "Added glyph %d[%s]!", g.id, g.path.c_str());
            }
        }
    }
}

/**
 * Load the glyphs from the given options.
 * @param [in] opts The instance to load glyphs from.
 */
void CameraStream::LoadGlyphs(Options *opts) {
    if (opts) {
        opts->SetFamily("CAMERA_GLYPHS");
        opts->GetList("GLYPH_LIST", (void*)&m_glyphs, GlyphUnpickler);
    }
}

/**
 * Descending order comparator function for sorting contours.
 * @param [in] a Contour 1.
 * @param [in] b Contour 2.
 * @return Comparator value.
 */
static bool ContourSort(const std::vector<cv::Point> &a, const std::vector<cv::Point> &b) {
    return cv::contourArea(b) < cv::contourArea(a);
}

/**
 * Order the four corners of the quadrilateral.
 * Note that this doesn't always work, but it's fast and good enough for
 * trying to detect a square's points.
 * A better method probably involves computing the convex hull of the points
 * but OpenCV's function doesn't seem to return them in any particular order.
 * @param [in] pts The points to be ordered.
 * @return The ordered points (top-left, top-right, bottom-left, bottom-right).
 */
static std::vector<cv::Point2f> OrderPoints(std::vector<cv::Point>& pts) {
    assert(pts.size() == 4);
    std::vector<cv::Point2f> ret(4);
    int p1=0,p2=0,p3=0,p4=0;
    
    for (int i = 0; i < 4; i++) {
        int sum = pts[i].x + pts[i].y;
        int diff = pts[i].y - pts[i].x;
        
        if (sum < (pts[p1].x + pts[p1].y))
            p1 = i;
        if (sum > (pts[p3].x + pts[p3].y))
            p3 = i;
        if (diff < (pts[p2].y - pts[p2].x))
            p2 = i;
        if (diff > (pts[p4].y - pts[p4].x))
            p4 = i;
    }
    
    ret[0] = pts[p1]; ret[1] = pts[p2]; ret[2] = pts[p3]; ret[3] = pts[p4];
    return ret;
}

/**
 * Use the contour points to warp the quad into a top-down perspective.
 * This function will only warp at least somewhat square quads.
 * @param [in] in The input frame.
 * @param [out] out The output frame.
 * @param [in] src The four defining points of the quad, ordered by:
 *                 (tl, tr, bl, br). (Use OrderPoints).
 * @return true iff it was warped.
 */
static bool WarpPerspective(cv::Mat &in, cv::Mat &out, std::vector<cv::Point2f> &src) {
    cv::Rect b = std::move(cv::boundingRect(src));
    //float ratio = static_cast<float>(b.width)/b.height;
    
    //Discard largely non-square images
    //if (ratio < 0.4 || ratio > 1.6)
    //    return false;
    
    std::vector<cv::Point2f> dst{cv::Point2f(0,0), cv::Point2f(b.width-1, 0),
        cv::Point2f(b.width-1,b.height-1), cv::Point2f(0, b.height-1)};
    cv::Mat trn = std::move(cv::getPerspectiveTransform(src, dst));
    cv::warpPerspective(in, out, trn, b.size());
    
    return true;
}

/**
 * Attempts to match the provided image with a known glyph.
 * @param [in] src The source image. Only used to get image bounds.
 * @param [in] roi The image to match to a glpyh.
 * @param [in] bounds The bounding rectangle of the glyph.
 * @return true iff glyphs were matched.
 */
bool CameraStream::GlyphDetection(cv::Mat &src, cv::Mat& roi, cv::Rect bounds) {
    bool ret = false;
    cv::Mat rquad;
    ObjectInfo obj{};
    
    //Perform initial resize.
    if (m_glyphs.size() > 0) {
        cv::resize(roi, rquad, 
            cv::Size(m_glyphs[0].image.cols, m_glyphs[0].image.rows));
        cv::cvtColor(rquad, rquad, CV_BGR2GRAY);
        cv::inRange(rquad, cv::Scalar(0), cv::Scalar(GLYPH_BLACK_THRESHOLD), rquad);
    }
    
    for(size_t i = 0; i < m_glyphs.size(); i++) {
        //Resize if necessary
        if (rquad.cols != m_glyphs[i].image.cols || rquad.rows != m_glyphs[i].image.rows) {
            cv::resize(roi, rquad, 
                cv::Size(m_glyphs[i].image.cols, m_glyphs[i].image.rows));
            cv::cvtColor(rquad, rquad, CV_BGR2GRAY);
            cv::inRange(rquad, cv::Scalar(0), cv::Scalar(GLYPH_BLACK_THRESHOLD), rquad);
        }
        
        //Display the warped and resized image
        if (m_demo_mode) {
            cv::imshow("Test", rquad);
            //cv::imwrite("test.png", rquad);
            //cv::waitKey(0);
        }
        
        //Perform template matching. This is the bottleneck if the template image is large.
        cv::Mat result(1, 1, CV_32FC1);
        cv::matchTemplate(rquad, m_glyphs[i].image, result, CV_TM_CCORR_NORMED);

        //Get the correlation value (no need for minMaxLoc - one result only)
        //double minVal; double maxVal;
        //cv::minMaxLoc(result, &minVal, &maxVal, NULL, NULL, cv::Mat());
        
        //Check goodness of fit. If good, add it.
        if (result.at<float>(0,0) > 0.8) {
            Log(LOG_DEBUG, "DETECTED %d<%s>! %.2f", 
                m_glyphs[i].id, m_glyphs[i].description.c_str(),
                result.at<float>(0,0));
            obj.id = m_glyphs[i].id;
            obj.image_width = src.cols;
            obj.image_height = src.rows;
            obj.bounds = bounds;
            obj.position = navigation::Point2D{
                obj.bounds.x + obj.bounds.width/2.0,
                obj.bounds.y + obj.bounds.height/2.0};
            m_detected.push_back(obj);
            
            ret = true;
        }
    }
    return ret;
}

/**
 * Performs glyph detection on the contour list.
 * @param [in] src The source image to detect glyphs from.
 * @param [in] contours The corresponding contour list.
 * @return true iff detected.
 */
bool CameraStream::GlyphContourDetection(cv::Mat& src, std::vector<std::vector<cv::Point>> contours) {
    bool ret = false;
    m_detected.clear();
    
    for (size_t i = 0; i < contours.size(); i++) {
        double area = cv::contourArea(contours[i]);
        if (area > 10) {
            double perimiter = cv::arcLength(contours[i], true);
            std::vector<cv::Point> approx;
            cv::approxPolyDP(contours[i], approx, 0.01*perimiter, true);
            
            if (approx.size() == 4 && cv::isContourConvex(approx)) { //We probably have a quad.
                cv::Mat quad;
                std::vector<cv::Point2f> pts = std::move(OrderPoints(approx));
                
                if (WarpPerspective(src, quad, pts)) {
                    ret |= GlyphDetection(src, quad, cv::boundingRect(pts));
                }
            }
        }
    }
    return ret;
}

/**
 * Perform glyph detection. Potential glyphs are first found using Canny
 * line detection to detect square objects.
 * @param [in] src The source image to search for a glyph.
 * @param [in] proc The process buffer.
 * @return true iff glyph was detected.
 */
bool CameraStream::CannyGlyphDetection(cv::Mat& src, cv::Mat& proc) {
    cv::Mat gray;
    //Downscale
    cv::resize(src, gray, cv::Size(PROCESS_WIDTH, PROCESS_HEIGHT));
    //Convert to grayscale
    cv::cvtColor(gray, gray, CV_BGR2GRAY);
    //Blur it a bit
    cv::GaussianBlur(gray, gray, cv::Size(5,5), 0);
    //Apply Canny detection
    cv::Canny(gray, proc, 100, 200);
    
    if (m_demo_mode) {
        cv::imshow("Thresholded image", proc);
        //cv::imwrite("glyph_canny.png", proc);
        cv::waitKey(1);
    }
    
    //Find the contours in the image and sort in descending order of contour area.
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(proc, contours, cv::RETR_LIST, cv::CHAIN_APPROX_SIMPLE);
    std::sort(contours.begin(), contours.end(), ContourSort);
    if (contours.size() > 10) {
        contours.resize(10);
    }
    
    //Translate from threshold point space back into source point space.
    //I'm sure there's an OpenCV way to do this with matrices, but whatever.
    for (size_t i = 0; i < contours.size(); i++) {
        for (size_t j = 0; j < contours[i].size(); j++) {
            contours[i][j].x *= PIXEL_SKIP;
            contours[i][j].y *= PIXEL_SKIP;
        }
    }
    
    //Get the detected glyphs.
    return GlyphContourDetection(src, contours);
}

/**
 * Perform glyph detection. Potential glyphs are found by using colour
 * thresholding to detect square objects. (Contours are detected in the
 * same manner used for the connected components algorithm).
 * 
 * @param [in] src The source image to search for a glyph.
 * @param [in] threshold The process buffer.
 * @return true iff glyph was detected.
 */
bool CameraStream::ThresholdingGlyphDetection(cv::Mat& src, cv::Mat& threshold) {
    //Threshold the image.
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
    std::sort(contours.begin(), contours.end(), ContourSort);
    if (contours.size() > 10) {
        contours.resize(10);
    }
    
    //Translate from threshold point space back into source point space.
    //I'm sure there's an OpenCV way to do this with matrices, but whatever.
    for (size_t i = 0; i < contours.size(); i++) {
        for (size_t j = 0; j < contours[i].size(); j++) {
            contours[i][j].x *= PIXEL_SKIP;
            contours[i][j].y *= PIXEL_SKIP;
        }
    }
    
    //Get the detected glyphs.
    return GlyphContourDetection(src, contours);
}

/**
 * Glyph detection using Hough circles.
 * @param [in] src The source image to search for a glyph.
 * @param [in] threshold The process buffer.
 * @return true iff glyph was detected.
 */
bool CameraStream::HoughDetection(cv::Mat& src, cv::Mat& proc) {
    bool ret = false;
    cv::Mat gray;
    //Downscale
    cv::resize(src, gray, cv::Size(PROCESS_WIDTH, PROCESS_HEIGHT));
    //Convert to grayscale
    cv::cvtColor(gray, gray, CV_BGR2GRAY);
    //Blur it a bit
    cv::GaussianBlur(gray, gray, cv::Size(9,9), 0);
    
    //Apply the Hough circle transform
    std::vector<cv::Vec3f> circles;
    cv::HoughCircles(gray, circles, CV_HOUGH_GRADIENT, 1, 30, 200, 50, 0, 0 );

    //Clear the detection list.
    m_detected.clear();
    
    //Draw the circles detected
    for(size_t i = 0; i < circles.size(); i++) {
        cv::Point centre(cvRound(circles[i][0])*PIXEL_SKIP, cvRound(circles[i][1])*PIXEL_SKIP);
        int radius = cvRound(circles[i][2])*PIXEL_SKIP;
        
        //Calculate ROI
        cv::Rect roi(centre.x-radius, centre.y-radius, radius*2, radius*2);
        if (roi.x >= 0 && roi.y >= 0 && roi.width > 0) {
            if (roi.x + roi.width > src.cols) {
                roi.width = src.cols - roi.x;
            }
            if (roi.y + roi.height > src.rows) {
                roi.height = src.rows - roi.y;
            }
            cv::Mat sroi(src, roi);
            ret = GlyphDetection(src, sroi, roi);
        }

        //Draw circles on the image...
        cv::circle(src, centre, 3, cv::Scalar(0, 255, 0), -1, 8, 0);
        cv::circle(src, centre, radius, cv::Scalar(0, 0, 255), 3, 8, 0);
    }

    return ret;
}
