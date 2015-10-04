/**
 * @file camtest.cpp
 * @brief Camera testing routine for thresholding
 */

#include <cstdio>
#include <cstdlib>
#include "common.h"
#include "camera_stream.h"
 
using picopter::CameraStream;
using picopter::Options;
using picopter::ThresholdParams;
using picopter::ThresholdColourspace;

#define BLACK 0
#define WHITE 255

cv::Mat g_src, g_proc;
ThresholdParams g_thresh;
uint8_t g_lut[THRESH_SIZE][THRESH_SIZE][THRESH_SIZE];

void RGB2HSV(uint8_t r, uint8_t g, uint8_t b, uint8_t *h, uint8_t *s, uint8_t *v) {
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

void RGB2YCbCr(uint8_t r, uint8_t g, uint8_t b, uint8_t *y, uint8_t *cb, uint8_t *cr) {
    *y = 0.299 * r + 0.587 * g + 0.114 * b;
    *cb = -0.168736 * r - 0.331264 * g + 0.500 * b + 128;
    *cr = 0.500 * r - 0.418688 * g - 0.081312 * b + 128;
}

void BuildThreshold(uint8_t lookup[][THRESH_SIZE][THRESH_SIZE], ThresholdParams thresh) {
    uint8_t r, g, b;
    for(r = 0; r < THRESH_SIZE; r++) {
        for(g = 0; g < THRESH_SIZE; g++) {
            for(b = 0; b < THRESH_SIZE; b++) {
                lookup[r][g][b] = 0;

                if (thresh.colourspace == ThresholdColourspace::THRESH_HSV) {
                    uint8_t h, s, v;
                    RGB2HSV(UNREDUCE(r), UNREDUCE(g), UNREDUCE(b), &h, &s, &v);

                    if (v >= thresh.p3_min && v <= thresh.p3_max &&
                        s >= thresh.p2_min && s <= thresh.p2_max) {
                        if (thresh.p1_min < 0) {
                            if ((h >= thresh.p1_min+180 && h <= 180) ||
                                (h >= 0 && h <= thresh.p1_max)) {
                                    lookup[r][g][b] = 1;
                                }
                        } else if (h >= thresh.p1_min && h <= thresh.p1_max) {
                            lookup[r][g][b] = 1;
                        }
                    }
                } else if (thresh.colourspace == ThresholdColourspace::THRESH_YCbCr) {
                    uint8_t y, cb, cr;
                    RGB2YCbCr(UNREDUCE(r), UNREDUCE(g), UNREDUCE(b), &y, &cb, &cr);

                    if (y >= thresh.p1_min && y <= thresh.p1_max &&
                        cb >= thresh.p2_min && cb <= thresh.p2_max &&
                        cr >= thresh.p3_min && cr <= thresh.p3_max) {
                        lookup[r][g][b] = 1;
                    }
                }
            }
        }
    }
}

void Threshold(uint8_t thresh[][THRESH_SIZE][THRESH_SIZE], const cv::Mat& src, cv::Mat &out, int width) {
    int i, j, k;
    const uint8_t* srcp;
    uint8_t* destp;
    int skip = src.cols / width;
    int nChannels = src.channels();

    out.create((src.rows * width) / src.cols, width, CV_8UC1);
    for(j=0; j < out.rows; j++) {
        srcp = src.ptr<const uint8_t>(j*skip);
        destp = out.ptr<uint8_t>(j);
        for (i=0; i < out.cols; i++) {
            k = i*nChannels*skip;
            if(thresh[srcp[k+2]/THRESH_DIV][srcp[k+1]/THRESH_DIV][srcp[k]/THRESH_DIV]) {
                destp[i] = WHITE;
            } else {
                destp[i] = BLACK;
            }
        }
    }
}
 
void on_call(int p, void* unused) {
    cv::Mat tmp;
    if (g_thresh.colourspace == ThresholdColourspace::THRESH_HSV && g_thresh.p1_min > 180) {
        g_thresh.p1_min = 180 - g_thresh.p1_min;
    }
    BuildThreshold(g_lut, g_thresh);
    Threshold(g_lut, g_src, g_proc, g_src.cols);
    cv::resize(g_proc, tmp, cv::Size(320, 240));
    cv::imshow("Threshold", tmp);
}
 
int main(int argc, char *argv[]) {
    if (argc < 3) {
        printf("Usage: %s options file1 [files...]\n", argv[0]);
        return 1;
    }
    
    Options *opts = new Options(argv[1]);
    cv::namedWindow("Threshold", CV_WINDOW_AUTOSIZE);
    
    opts->SetFamily("CAMERA_STREAM");
    g_thresh.colourspace = ThresholdColourspace::THRESH_HSV;
    g_thresh.p1_min = opts->GetInt("MIN_HUE", -10);
    g_thresh.p1_max = opts->GetInt("MAX_HUE", 10);
    g_thresh.p2_min = opts->GetInt("MIN_SAT", 95);
    g_thresh.p2_max = opts->GetInt("MAX_SAT", 255);
    g_thresh.p3_min = opts->GetInt("MIN_VAL", 127);
    g_thresh.p3_max = opts->GetInt("MAX_VAL", 255);
    
    cv::createTrackbar("LowH/LowY", "Threshold", &g_thresh.p1_min, 255, on_call); //Hue (0 - 179)
    cv::createTrackbar("HighH/HighY", "Threshold", &g_thresh.p1_max, 255, on_call);
    cv::createTrackbar("LowS/LowCr", "Threshold", &g_thresh.p2_min, 255, on_call); //Saturation (0 - 255)
    cv::createTrackbar("HighS/HighCr", "Threshold", &g_thresh.p2_max, 255, on_call);
    cv::createTrackbar("LowV/LowCb", "Threshold", &g_thresh.p3_min, 255, on_call);//Value (0 - 255)
    cv::createTrackbar("HighV/HighCb", "Threshold", &g_thresh.p3_max, 255, on_call);
    
    for (int i = 2; i < argc; i++) {
        char c;
        printf("Processing %s\n", argv[i]);
        fflush(stdout);
        g_src = cv::imread(argv[i], 1);
        do {
            on_call(0, NULL);
            c = cv::waitKey(0);
        } while (c != 'n' && c != 'q');
        std::string n = std::string("thresh/") + std::string(argv[i]) + "." + 
            std::to_string(g_thresh.p1_min) + "." + std::to_string(g_thresh.p1_max) + "." +
            std::to_string(g_thresh.p2_min) + "." + std::to_string(g_thresh.p2_max) + "." +
            std::to_string(g_thresh.p3_min) + "." + std::to_string(g_thresh.p3_max) + "." +
            "thresh.png";
            
        printf("Name: %s\n", n.c_str());
        cv::imwrite(n, g_proc);
        if (c == 'q') {
            break;
        }
    }
}