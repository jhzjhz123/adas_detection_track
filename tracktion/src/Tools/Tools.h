#ifndef TOOLS_H_
#define TOOLS_H_

#include <string>
#include <iostream>
#include <fstream>
#include <opencv2/opencv.hpp>

// #define TIME_COUNT
#ifdef TIME_COUNT
#include <chrono>
#endif
// #define DEBUG_INFO
#define AISDK_INTERFACE

class errMsg
{
    errMsg();
    ~errMsg();
    static errMsg* instance;

public:
    static errMsg* getInstance();
    void out(std::string file, std::string func, std::string msg, bool pause = true);
};

float getdata(std::string &s);

void GetData(const char* s, float& Velocity, float& StirAngle, float AngleOffset);

void GetInputData(std::ifstream &inputfile, float &Velocity, float &StirAngle, const float AngleOffset);

cv::Mat image_add(cv::Mat image, cv::Mat image_add1, cv::Mat image_add2, float factor);

#ifdef AISDK_INTERFACE
#else
float* get_hog_feature(cv::Mat img, cv::Point2f center_point);
#endif

#endif
