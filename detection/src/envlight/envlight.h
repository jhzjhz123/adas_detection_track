#pragma once
#ifndef ENVLIGHT_H_
#define ENVLIGHT_H_

#include "../utils/utils.h"
#include "json11/json11.hpp"

struct EnlightInputData
{
	char *elf_model_name;
    char *input_node;
	char *output_node;
	unsigned int crop_size_left_top_x;
	unsigned int crop_size_left_top_y;
	unsigned int crop_size_width;
	unsigned int crop_size_height;
};

class EnvlightProcess{
public:
    EnvlightProcess(const char* config_file);
    ~EnvlightProcess();
    void PreProcess(cv::Mat& img);
    std::vector<float> GetResult();
    void PostProcess(std::string imagename);

private:
    void getParams(const char* file_name);
    void showParams();
    void setInputImageForEnvLight();
    void make_empty();

    DPUKernel* envlight_kernel;
    DPUTask* envlight_task;
    cv::Mat img_init, img_input;
    EnlightInputData envlight_params;
    std::vector<std::string> paramSet = {"elf_model_name",
                                        "input_node", 
                                        "output_node", 
                                        "crop_size_left_top_x", 
                                        "crop_size_left_top_y", 
                                        "crop_size_width", 
                                        "crop_size_height", 
                                        "resize_size_x", 
                                        "resize_size_y"}; 
    int input_width, input_height, input_size;
    int output_channel;
    std::vector<float> fcn_out, softmax_out;
};



#endif
