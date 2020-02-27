#include "lane.h"


LaneProcess::LaneProcess(const char* config_file){
    getParams(config_file);
    #ifdef DEBUG_INFO
    showParams();
    #endif

    dpuOpen();
    lane_kernel = dpuLoadKernel(lane_params.elf_model_name);
	lane_task = dpuCreateTask(lane_kernel, 0);
}

LaneProcess::~LaneProcess(){
    dpuDestroyTask(lane_task);
    dpuDestroyKernel(lane_kernel);
    dpuClose();
        
    make_empty();
}

void LaneProcess::PreProcess(cv::Mat& img){
    #ifdef DEBUG_INFO
    std::cout << "img.size(): " << img.size() << std::endl;
    #endif
    img_post = img.clone();
    img_init = img;
    setInputImageForLane();
}

cv::Mat LaneProcess::GetResult(){
    #ifdef TIME_COUNT
    time1 = get_current_time();
    #endif
    dpuRunTask(lane_task);
    #ifdef TIME_COUNT
    time2 = get_current_time();
    std::cout << "dpuRunTask:    " << fixed << setprecision(3) <<  (time2 - time1)/1000.0  << "ms" << std::endl;
    #endif
    
    output_width = dpuGetOutputTensorWidth(lane_task, lane_params.output_node[0]);
    output_height = dpuGetOutputTensorHeight(lane_task, lane_params.output_node[0]);
    output_channel = dpuGetOutputTensorChannel(lane_task, lane_params.output_node[0]);
    output_size = dpuGetOutputTensorSize(lane_task, lane_params.output_node[0]);
    output_scale = dpuGetOutputTensorScale(lane_task, lane_params.output_node[0]);
    #ifdef DEBUG_INFO
    std::cout << "output_width: " << output_width << std::endl;
    std::cout << "output_height: " << output_height << std::endl;
    std::cout << "output_channel: " << output_channel << std::endl;
    std::cout << "output_size: " << output_size << std::endl;
    std::cout << "output_scale: " << output_scale << std::endl;
    #endif

    #ifdef TIME_COUNT
    time1 = get_current_time();
    #endif
    lane_mat = cv::Mat::zeros(cv::Size(output_width, output_height), CV_8UC1); 

    #if HWC
    output_data = dpuGetOutputTensorAddress(lane_task, lane_params.output_node[0]);

    // for (int row = 0; row < output_height; row++) { //method1
    //     for (int col = 0; col < output_width; col++) {
    //         int i = row * output_width * output_channel + col * output_channel;
    //         auto max_p = max_element(output_data + i, output_data + i + output_channel);
    //         int max_c = distance(output_data + i, max_p);
    //         lane_mat.at<uchar>(row, col) = (uchar)(max_c);
    //     }
    // }

    for(int i = 0; i < output_size; i += output_channel){
        int location = i/output_channel;
        int w = location % output_width;
        int h = location / output_width;

        // int max = output_data[i++];  //method2
        // int max_c = 0;
        // int c = 1;
        // while(c < output_channel){
        //     if(max < output_data[i]){
        //         max = output_data[i];
        //         max_c = c;
        //     }
        //     ++i;
        //     ++c;
        // }

        int8_t *curr_p = output_data + i; //method3
        auto max_p = max_element(curr_p, curr_p + output_channel);
        int max_c = distance(curr_p, max_p);
        
        lane_mat.at<uchar>(h,w) = (uchar)(max_c);
    }
    #elif CHW
    output_data = new int8_t[output_size]; //method4
    dpuGetOutputTensorInCHWInt8(lane_task, lane_params.output_node[0], output_data, output_size);

    for (int h = 0; h < output_height; ++h){
        for (int w = 0; w < output_width; ++w){
            float max = output_data[0*output_height*output_width + h*output_width + w];
            int max_c = 0;
            for (int c = 1; c < output_channel; ++c) {
                if (max < output_data[c*output_height*output_width + h*output_width + w]){
                    max = output_data[c*output_height*output_width + h*output_width + w];
                    max_c = c;
                }
            }
            lane_mat.data[h*output_width + w] = max_c;
        }
    }
    delete output_data;
    #endif

    #ifdef TIME_COUNT
    time2 = get_current_time();
    std::cout << "get map:    " << fixed << setprecision(3) <<  (time2 - time1)/1000.0  << "ms" << std::endl;
    #endif

    // resize(lane_mat, lane_mat, cv::Size(lane_params.crop_size_width, lane_params.crop_size_height));
    // lane_mat *= 255;
    
    // cv::Mat seg_res = cv::Mat::zeros(img_post.size(), CV_8UC1);
    // lane_mat.copyTo(seg_res(cv::Rect(0, lane_params.crop_size_left_top_y, lane_mat.cols, lane_mat.rows)));

    // cv::Mat seg3_res= convertTo3Channels(seg_res);

    // for (auto h = 0; h < img_post.size().height; ++h) {
    //     for (auto w = 0; w < img_post.size().width; ++w) {
    //         img_post.at<cv::Vec3b>(h, w) = img_post.at<cv::Vec3b>(h, w) * 0.5 + seg3_res.at<cv::Vec3b>(h, w) * 0.5;
    //     }
    // }

    return lane_mat;
}

void LaneProcess::PostProcess(std::string imagename){
    imwrite("images/results/" + imagename + "_" + medstr + "_result.jpg", img_post, {CV_IMWRITE_JPEG_QUALITY, 100});
}

cv::Mat LaneProcess::thread_func(cv::Mat& img){
    PreProcess(img);
    cv::Mat mat_result = GetResult();
    return mat_result;
}

void LaneProcess::setInputImageForLane(){
    input_width = dpuGetInputTensorWidth(lane_task, lane_params.input_node);
    input_height = dpuGetInputTensorHeight(lane_task, lane_params.input_node);
    input_channel = dpuGetInputTensorChannel(lane_task, lane_params.input_node);
    input_size = dpuGetInputTensorSize(lane_task, lane_params.input_node);
    input_scale = dpuGetInputTensorScale(lane_task, lane_params.input_node);
    input_data = dpuGetInputTensorAddress(lane_task, lane_params.input_node);
    #ifdef DEBUG_INFO
    std::cout << "input_width: " << input_width << std::endl;
    std::cout << "input_height: " << input_height << std::endl;
    std::cout << "input_channel: " << input_channel << std::endl;
    std::cout << "input_size: " << input_size << std::endl;
    std::cout << "input_scale: " << input_scale << std::endl;
    #endif
    
    #ifdef TIME_COUNT
    time1 = get_current_time();
    #endif
    img_input = img_init(cv::Rect(lane_params.crop_size_left_top_x, lane_params.crop_size_left_top_y, lane_params.crop_size_width, lane_params.crop_size_height));
    #ifdef TIME_COUNT
    time2 = get_current_time();
    std::cout << "crop:    " << fixed << setprecision(3) <<  (time2 - time1)/1000.0  << "ms" << std::endl;

    time1 = get_current_time();
    #endif
    // if(img_input.size() != cv::Size(input_width, input_height)){
        cv::resize(img_input, img_input, cv::Size(input_width, input_height));
    // }
    #ifdef TIME_COUNT
    time2 = get_current_time();
    std::cout << "resize:    " << fixed << setprecision(3) <<  (time2 - time1)/1000.0  << "ms" << std::endl;

    time1 = get_current_time();
    #endif
    img_input -= 128;
    #ifdef TIME_COUNT
    time2 = get_current_time();
    std::cout << "-128:    " << fixed << setprecision(3) <<  (time2 - time1)/1000.0  << "ms" << std::endl;

    time1 = get_current_time();
    #endif
    #if 1
    memcpy(input_data, img_input.data, input_size);
    #ifdef TIME_COUNT
    time2 = get_current_time();
    std::cout << "memcpy:    " << fixed << setprecision(3) <<  (time2 - time1)/1000.0  << "ms" << std::endl;
    #endif
    #else
    dpuSetInputImageWithScale(lane_task, lane_params.input_node, img_input, mean, 1.f);
    #ifdef TIME_COUNT
    time2 = get_current_time();
    std::cout << "dpuSetInputImageWithScale:    " << fixed << setprecision(3) <<  (time2 - time1)/1000.0  << "ms" << std::endl;
    #endif
    #endif
}

void LaneProcess::getParams(const char* file_name){
	std::filebuf fin; 
	if (!fin.open(file_name, std::ios::in)) {  
		std::cout << "fail to open file" << std::endl;  
		return; 
	} 
	
	std::istream iss(&fin); 
	std::istreambuf_iterator<char> eos; 
	std::string buf(std::istreambuf_iterator<char>(iss), eos); 

	std::string err; 
	auto json = json11::Json::parse(buf, err); 
	if (!err.empty()) {
		std::cout << "fail to parse file" << std::endl;    
		fin.close();  
		return; 
	}

    const char* elf_model_name = json[paramSet[0]].string_value().c_str();
	int elf_model_name_lenth = strlen(elf_model_name);
	lane_params.elf_model_name = new char[elf_model_name_lenth + 1];
	strcpy(lane_params.elf_model_name, elf_model_name);
	
	const char* input_node = json[paramSet[1]].string_value().c_str();
	int input_node_lenth = strlen(input_node);
	lane_params.input_node = new char[input_node_lenth + 1];
	strcpy(lane_params.input_node, input_node);

	json11::Json::array output_node_array = json[paramSet[2]].array_items(); 
    output_num = output_node_array.size();
    lane_params.output_node = new char*[output_num];
    for(int i = 0; i < output_num; ++i){
        const char* node =  output_node_array[i].string_value().c_str();
        int lenth = strlen(node);
        lane_params.output_node[i] = new char[lenth + 1];
        strcpy(lane_params.output_node[i], node);
    }

	lane_params.crop_size_left_top_x = json[paramSet[3]].int_value();
	lane_params.crop_size_left_top_y = json[paramSet[4]].int_value();
	lane_params.crop_size_width = json[paramSet[5]].int_value();
	lane_params.crop_size_height = json[paramSet[6]].int_value(); 
}

void LaneProcess::showParams(){
    printf("elf_model_name: %s\n", lane_params.elf_model_name);
    printf("input_node :%s\n", lane_params.input_node);
	for(int i = 0 ; i < output_num; ++i)
		printf("output_node %d : %s\n", i, lane_params.output_node[i]);
    printf("crop_size_left_top_x :%d\n", lane_params.crop_size_left_top_x);
	printf("crop_size_left_top_y :%d\n", lane_params.crop_size_left_top_y);
	printf("crop_size_width :%d\n", lane_params.crop_size_width);
	printf("crop_size_height :%d\n", lane_params.crop_size_height);
}

void LaneProcess::make_empty(){
    delete[] lane_params.elf_model_name;
    
    delete[] lane_params.input_node;
    
    for(int i = 0; i < output_num; ++i)
        delete[] lane_params.output_node[i];
    delete[] lane_params.output_node;
}