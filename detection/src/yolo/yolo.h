#pragma once
#ifndef YOLO_H
#define YOLO_H

#include "../utils/utils.h"
#include "json11/json11.hpp"
#include <chrono>
#include "opencv2/core/core.hpp"
#include <arm_neon.h>

#define USER_DEFINE 0
#define DPU_SCALE 0
#define MY_SCALER 0
#define MULTI_THREAD_SCALE 0
#define THREAD_NUMS 3
#define ARM_NEON_SCALE 1

struct YOLOInputData{
    char *elf_model_name;
    char *input_node;             /*!< input node name*/
	char **output_node;           /*!< output node name*/
	unsigned int num_classes;     /*!< classes numbers*/
	unsigned int anchorCnt;       /*!< anchor numbers per output*/
	float *conf_threshold;        /*!< confidence threshold*/
	float nms_threshold;          /*!< nms threshold*/
	float *biases;                /*!< biases pointer*/
    unsigned int conf_box;        /*!< the channel size per box*/
    float Sig_Matrix[256];        /*!< initial map for sigmoid*/
    float Exp_Matrix[256];        /*!< initial map for exp*/
    unsigned int crop_size_left_top_x;
	unsigned int crop_size_left_top_y;
	unsigned int crop_size_width;
	unsigned int crop_size_height;
};

/*dataThread用于传递线程需要的参数值*/
struct dataThread
{
	int w;
	int h;
	uchar * data;
};

class YoLoProcess{
public:
    YoLoProcess(const char* config_file);
    ~YoLoProcess();
    void PreProcess(cv::Mat& img);
    std::vector<std::vector<float>> GetResult();
    void PostProcess(std::string imagename);
    std::vector<std::vector<float>> thread_func(cv::Mat& img);
private:
    void getParams(const char* file_name);
    void showParams();
    void setInputImageForYOLO();
    void detect(vector<vector<float>> &boxes, int8_t* input, int channel, int height, int width, int num, int sHeight, int sWidth, int sizeOut);
    vector<vector<float>> applyNMS(vector<vector<float>>& boxes, const float thres);
    void refine(std::vector<std::vector<float>>& nms_results_, std::vector<std::vector<float>>& refine_results_);
    void make_empty();

    DPUKernel* yolo_kernel;
    DPUTask* yolo_task;
    cv::Mat img_init, img_input, img_post;
    
    YOLOInputData yolo_params;
    std::vector<std::string> paramSet = {"elf_model_name",
                                        "input_node", 
                                        "output_node", 
                                        "num_classes", 
                                        "anchorCnt", 
                                        "conf_threshold", 
                                        "nms_threshold", 
                                        "biases",
                                        "crop_size_left_top_x",
                                        "crop_size_left_top_y",
                                        "crop_size_width",
                                        "crop_size_height"};
    int output_num, conf_num, biases_num;

    int input_width, input_height, input_channel, input_size;
    float input_scale;
    int8_t* input_data;

    int output_width, output_height, output_channel, output_size;
    float output_scale;    
    int8_t* output_data;
    
    std::vector<std::vector<float>> det_results, nms_results, final_results;

    float ws = 1.0, hs =1.0;

    int box_cnt = 0;

    unsigned long time1, time2;

    float mean[3]={0.f, 0.f, 0.f};

    int8_t ImgScaleMatrix[256];
    int8_t data[1966080]; // input_width*input_height*input_channel

    #if MY_SCALER
    std::string elf_file = "scale";

    DPUKernel* scaler_kernel;
    DPUTask* scaler_task;

    std::string sinput_node = "preprocess";
    std::string soutput_node = "preprocess";

    int sinput_width, sinput_height, sinput_channel, sinput_size;
    float sinput_scale;
    int8_t* sinput_data;

    int soutput_width, soutput_height, soutput_channel, soutput_size;
    float soutput_scale;    
    int8_t* soutput_data;
    #endif

    #if MULTI_THREAD
    /********************************************************
    *	@brief          : 多线程处理函数
    *	@param  pt_args : 多线程传入的参数
    *	@return         : void*
    ********************************************************/
    static void* threadProcess(void* args);
    #endif
};


#endif