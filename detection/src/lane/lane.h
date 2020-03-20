
#pragma once
#ifndef LANE_H
#define LANE_H

#include "../utils/utils.h"
#include "json11/json11.hpp"
#include <chrono>

#define HWC 1
#define CHW 0
#define MERGED_RES_MAT
// #define MULTI_CHANNEL_RES_MAT
#define NEON_NORM

struct LaneInputData{
    char *elf_model_name;
    char *input_node;
	char **output_node;
	unsigned int crop_size_left_top_x;
	unsigned int crop_size_left_top_y;
	unsigned int crop_size_width;
	unsigned int crop_size_height;
};

class LaneProcess{
public:
    LaneProcess(const char* config_file);
    ~LaneProcess();
    void PreProcess(cv::Mat& img);
    cv::Mat GetResult();
    void PostProcess(std::string imagename);
    cv::Mat thread_func(cv::Mat& img);
private:
    void getParams(const char* file_name);
    void showParams();
    void setInputImageForLane();
    void make_empty();

    DPUKernel* lane_kernel;
    DPUTask* lane_task;
    DPUTensor *input_tensor, *output_tensor;
    cv::Mat img_init, img_input, img_post;
    LaneInputData lane_params;
    std::vector<std::string> paramSet = {"elf_model_name",
                                        "input_node", 
                                        "output_node", 
                                        "crop_size_left_top_x",
                                        "crop_size_left_top_y",
                                        "crop_size_width",
                                        "crop_size_height"};
    int output_num;

    int input_width, input_height, input_channel, input_size;
    float input_scale;
    int8_t* input_data;

    int output_width, output_height, output_channel, output_size;
    float output_scale;
    int8_t* output_data;
    cv::Mat lane_mat;

    // float mean[3] = {103.939, 116.779, 123.68}; // 20200304
    float mean[3] = {46.18, 47.61, 48.78}; // 20200313
    // float mean[3] = {47.61, 47.61, 47.61}; // 20200313

    unsigned long time1, time2;

    #define channels_num 5
    uint8_t colorB[channels_num] = {128, 232, 70, 156, 153};
    uint8_t colorG[channels_num] = {64, 35, 70, 102, 153};
    uint8_t colorR[channels_num] = {128, 244, 70, 102, 190};
};

#endif