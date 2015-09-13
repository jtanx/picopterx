/**
 * @file camera_glyphs.cpp
 * @brief Glyph detection code.
 * @author Jeremy Tan
 * Loosely based on:
 * https://rdmilligan.wordpress.com/2015/07/19/glyph-recognition-using-opencv-and-python/
 */

#include "common.h"
#include "camera_stream.h"

using picopter::CameraStream;

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
 * @param [in] pts The four points that define the quad.
 * @return true iff it was warped.
 */
static bool WarpPerspective(cv::Mat &in, cv::Mat &out, std::vector<cv::Point> &pts) {
    std::vector<cv::Point2f> src = std::move(OrderPoints(pts));
    cv::Rect b = std::move(cv::boundingRect(src));
    float ratio = static_cast<float>(b.width)/b.height;
    
    //Discard largely non-square images
    if (ratio < 0.4 || ratio > 1.6)
        return false;
    
    std::vector<cv::Point2f> dst{cv::Point2f(0,0), cv::Point2f(b.width-1, 0),
        cv::Point2f(b.width-1,b.height-1), cv::Point2f(0, b.height-1)};
    cv::Mat trn = std::move(cv::getPerspectiveTransform(src, dst));
    cv::warpPerspective(in, out, trn, b.size());
    
    return true;
}

/**
 * Performs glyph detection on the contour list.
 * @param [in] src The source image to detect glyphs from.
 * @param [in] contours The corresponding contour list.
 * @param [in] templ The glyph template to match against.
 * @return true iff detected.
 */
bool CameraStream::GlyphContourDetection(cv::Mat& src, std::vector<std::vector<cv::Point>> contours, cv::Mat &templ) {
    for (size_t i = 0; i < 2 && i < contours.size(); i++) {
        double perimiter = cv::arcLength(contours[i], true);
        std::vector<cv::Point> approx;
        cv::approxPolyDP(contours[i], approx, 0.01*perimiter, true);
        
        if (approx.size() == 4) { //We probably have a quad.
            cv::Mat quad;
            if (WarpPerspective(src, quad, approx)) {
                cv::resize(quad, quad, cv::Size(templ.cols, templ.rows));
                if (m_demo_mode) {
                    cv::imshow("Test", quad);
                }
                
                //Perform template matching
                cv::Mat result(1, 1, CV_32FC1);
                cv::matchTemplate(quad, templ, result, CV_TM_CCORR_NORMED);

                //Get the correlation value
                double minVal; double maxVal;
                cv::minMaxLoc(result, &minVal, &maxVal, NULL, NULL, cv::Mat());
                
                //Log(LOG_DEBUG, "%.4f", maxVal);
                //Check goodness of fit.
                if (maxVal > 0.8)
                    Log(LOG_DEBUG, "DETECTED! %.2f", maxVal);
            }
        }
    }
    return false;
}

/**
 * Perform glyph detection. Potential glyphs are first found using Canny
 * line detection to detect square objects.
 * @param [in] src The source image to search for a glyph.
 * @param [in] proc The process buffer.
 * @param [in] templ The template glyph.
 * @return true iff glyph was detected.
 */
bool CameraStream::CannyGlyphDetection(cv::Mat& src, cv::Mat& proc, cv::Mat& templ) {
    cv::Mat gray;
    //Downscale
    cv::resize(src, gray, cv::Size(PROCESS_WIDTH, PROCESS_HEIGHT));
    //Convert to grayscale
    cv::cvtColor(gray, gray, CV_BGR2GRAY);
    //Blur it a bit
    cv::GaussianBlur(gray, gray, cv::Size(5,5), 0);
    //Apply Canny detection
    cv::Canny(gray, proc, 100, 200);
    
    //Find the contours in the image and sort in descending order of contour area.
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(proc, contours, cv::RETR_LIST, cv::CHAIN_APPROX_SIMPLE);
    std::sort(contours.begin(), contours.end(), ContourSort);
    
    return GlyphContourDetection(gray, contours, templ);
}

/**
 * Perform glyph detection. Potential glyphs are found by using colour
 * thresholding to detect square objects. (Contours are detected in the
 * same manner used for the connected components algorithm).
 * 
 * @param [in] src The source image to search for a glyph.
 * @param [in] threshold The process buffer.
 * @param [in] templ The template glyph.
 * @return true iff glyph was detected.
 */
bool CameraStream::ThresholdingGlyphDetection(cv::Mat& src, cv::Mat& threshold, cv::Mat& templ) {
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
    //Translate from threshold point space back into source point space.
    //I'm sure there's an OpenCV way to do this with matrices, but whatever.
    for (size_t i = 0; i < 2 && i < contours.size(); i++) {
        for (size_t j = 0; j < contours[i].size(); j++) {
            contours[i][j].x *= PIXEL_SKIP;
            contours[i][j].y *= PIXEL_SKIP;
        }
    }
    return GlyphContourDetection(src, contours, templ);
}
