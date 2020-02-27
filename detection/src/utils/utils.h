#pragma once
#ifndef UTILS_H
#define UTILS_H

#include <iostream>
#include <fstream>
#include <iomanip>
#include <string>
#include <vector>
#include <string.h>
#include <math.h>
#include <sys/stat.h>
#include <dirent.h>
#include <sys/time.h>
#include "dnndk/dnndk.h"
#include <thread>
#include <mutex>
#include <future>

// #define USE_RESIZE
// #define DEBUG_INFO
// #define TIME_COUNT

using namespace std;
using namespace cv;

#define YUV420OPEN 1
static std::string medstr = "meyuv";

unsigned long get_current_time(void);
void swapPointerUsePointer(int8_t **p, int8_t **q);
void swapPointerUseReference(int8_t *&p, int8_t *&q);
void ListPath(string const &path, vector<string> &paths);
void ListImages(string const &path, vector<string> &images);
float overlap(float x1, float w1, float x2, float w2);
float cal_iou(vector<float> box, vector<float>truth);
void GetSigMatrix(float *SigMatrix, float *ExpMatrix, float scale);
void GetSigMatrix(float *SigMatrix, float *ExpMatrix, int8_t *ImgScaleMatrix, float scale, float img_scale);
float sigmoid(float p);
void CPUCalcSoftmax(const float *data, size_t size, float *result);
Mat convertTo3Channels(const Mat& binImg);
vector<cv::Mat> splitImage(cv::Mat image, int num, int type);
cv::Mat catImage(vector<cv::Mat> v, int type);

#endif