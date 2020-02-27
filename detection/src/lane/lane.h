
#pragma once
#ifndef LANE_H
#define LANE_H

#include "../utils/utils.h"
#include "json11/json11.hpp"
#include <chrono>

#define HWC 1
#define CHW 0

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

    float mean[3] = {0.f, 0.f, 0.f};
    // float mean[3] = {103.939f, 116.779f, 123.68f};

    unsigned long time1, time2;
};

#endif