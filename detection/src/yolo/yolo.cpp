#include "yolo.h"

YoLoProcess::YoLoProcess(const char* config_file){
    getParams(config_file);
    #ifdef DEBUG_INFO
    showParams();
    #endif

    dpuOpen();

    yolo_kernel = dpuLoadKernel(yolo_params.elf_model_name);
	yolo_task = dpuCreateTask(yolo_kernel, 0);

    #if MY_SCALER
    scaler_kernel = dpuLoadKernel(elf_file.c_str());
    scaler_task = dpuCreateTask(scaler_kernel, 0);
    #endif
}

YoLoProcess::~YoLoProcess(){
    dpuDestroyTask(yolo_task);
    dpuDestroyKernel(yolo_kernel);

    #if MY_SCALER
    dpuDestroyTask(scaler_task);
    dpuDestroyKernel(scaler_kernel);
    #endif

    dpuClose();
    
    make_empty();
}

void YoLoProcess::PreProcess(cv::Mat& img){
    #ifdef DEBUG_INFO
    std::cout << "img.size(): " << img.size() << std::endl;
    #endif
    img_post = img.clone();
    img_init = img;
    setInputImageForYOLO();
}

std::vector<std::vector<float>> YoLoProcess::GetResult(){
    det_results.clear();
    nms_results.clear();
    final_results.clear();
    
    #ifdef TIME_COUNT
    time1 = get_current_time();
    #endif
    dpuRunTask(yolo_task);
    #ifdef TIME_COUNT
    time2 = get_current_time();
    std::cout << "dpuRunTask:    " << fixed << setprecision(3) <<  (time2 - time1)/1000.0  << "ms" << std::endl;

    time1 = get_current_time();
    #endif
    for(int i = 0; i < output_num; ++i){
	    output_width = dpuGetOutputTensorWidth(yolo_task, yolo_params.output_node[i]);
        output_height = dpuGetOutputTensorHeight(yolo_task, yolo_params.output_node[i]);
        output_channel = dpuGetOutputTensorChannel(yolo_task, yolo_params.output_node[i]);
        output_size = dpuGetOutputTensorSize(yolo_task, yolo_params.output_node[i]);
        output_scale = dpuGetOutputTensorScale(yolo_task, yolo_params.output_node[i]);
        output_data = dpuGetOutputTensorAddress(yolo_task, yolo_params.output_node[i]);

        #ifdef DEBUG_INFO
        std::cout << "output_width: " << output_width << std::endl;
        std::cout << "output_height: " << output_height << std::endl;
        std::cout << "output_channel: " << output_channel << std::endl;
        std::cout << "output_size: " << output_size << std::endl;
        std::cout << "output_scale: " << output_scale << std::endl;
        #endif

        /* Store the object detection frames as coordinate information  */
        detect(det_results, output_data, output_channel, output_height, output_width, i, input_height, input_width, output_size);
	}
    #ifdef TIME_COUNT
    time2 = get_current_time();
    std::cout << "detect: " << fixed << setprecision(3) <<  (time2 - time1)/1000.0  << "ms" << std::endl;

    time1 = get_current_time();
    #endif
    nms_results = applyNMS(det_results, yolo_params.nms_threshold);
    #ifdef TIME_COUNT
    time2 = get_current_time();
    std::cout << "nms: " << fixed << setprecision(3) <<  (time2 - time1)/1000.0  << "ms" << std::endl;

    time1 = get_current_time();
    #endif
    refine(nms_results, final_results);   
    #ifdef TIME_COUNT 
    time2 = get_current_time();
    std::cout << "refine: " << fixed << setprecision(3) <<  (time2 - time1)/1000.0  << "ms" << std::endl;
    #endif

    return final_results;
}

void YoLoProcess::PostProcess(std::string imagename){
    std::ofstream fout("images/results/" + imagename.substr(0, imagename.length() - 4) + "_"  + medstr + ".txt");
    std::ofstream box_file("images/results/boxcount_" + medstr + ".txt", ios::app);

    for(size_t i = 0; i < final_results.size(); ++i) {
        float xmin = final_results[i][0];
        float ymin = final_results[i][1];
        float obj_w = final_results[i][2];
        float obj_h = final_results[i][3];
        int cls = final_results[i][4];
        float conf = final_results[i][5];
        fout << cls << "," << conf << "," << xmin << "," << ymin << "," << obj_w << "," << obj_h << endl;

        if (cls==0) {
            rectangle(img_post, cv::Rect(xmin, ymin, obj_w, obj_h), Scalar(0, 255, 0), 2, 1, 0);
        }
        else if (cls==1 || cls == 2 ||cls == 3) {
            rectangle(img_post, cv::Rect(xmin, ymin, obj_w, obj_h), Scalar(255, 0, 0), 2, 1, 0);
        }
        else {
            rectangle(img_post, cv::Rect(xmin, ymin, obj_w, obj_h), Scalar(0 ,255, 255), 2, 1, 0);
        }
        cv::putText(img_post, std::to_string(cls), cv::Point(xmin, ymin-5), cv::FONT_HERSHEY_SIMPLEX, 0.65, cv::Scalar(0, 0, 255), 2, 8);
    }
    box_cnt += final_results.size();
    box_file << imagename << "--->" << final_results.size() << endl;
    box_file << "boxcount: " << box_cnt << endl;

    fout.close();
    box_file.close();

    imwrite("images/results/" + imagename + "_" + medstr + "_result.jpg", img_post, {CV_IMWRITE_JPEG_QUALITY, 100});
}

std::vector<std::vector<float>> YoLoProcess::thread_func(cv::Mat& img){
    PreProcess(img);
    std::vector<std::vector<float>> vec_result = GetResult();
    return vec_result;
}

void YoLoProcess::getParams(const char* file_name){
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
	yolo_params.elf_model_name = new char[elf_model_name_lenth + 1];
	strcpy(yolo_params.elf_model_name, elf_model_name);

    const char* input_node = json[paramSet[1]].string_value().c_str();
	int input_node_lenth = strlen(input_node);
	yolo_params.input_node = new char[input_node_lenth + 1];
	strcpy(yolo_params.input_node, input_node);

    json11::Json::array output_node_array = json[paramSet[2]].array_items(); 
    output_num = output_node_array.size();
    yolo_params.output_node = new char*[output_num];
    for(int i = 0; i < output_num; ++i){
        const char* node =  output_node_array[i].string_value().c_str();
        int lenth = strlen(node);
        yolo_params.output_node[i] = new char[lenth + 1];
        strcpy(yolo_params.output_node[i], node);
    }

    yolo_params.num_classes = json[paramSet[3]].int_value();
    yolo_params.anchorCnt = json[paramSet[4]].int_value();

    json11::Json::array conf_threshold_array = json[paramSet[5]].array_items(); 
    conf_num = conf_threshold_array.size();
    yolo_params.conf_threshold = new float[conf_num];
    for(int i=0; i< conf_num; ++i){
        yolo_params.conf_threshold[i] = atof(conf_threshold_array[i].string_value().c_str());
	}

    yolo_params.nms_threshold = atof(json[paramSet[6]].string_value().c_str());
    
    json11::Json::array biases_array = json[paramSet[7]].array_items(); 
    biases_num = biases_array.size();
    yolo_params.biases = new float[biases_num];
    for(int i=0; i<biases_num;i++){
        yolo_params.biases[i] = biases_array[i].int_value();
	}

    yolo_params.crop_size_left_top_x = json[paramSet[8]].int_value();
	yolo_params.crop_size_left_top_y = json[paramSet[9]].int_value();
	yolo_params.crop_size_width = json[paramSet[10]].int_value();
	yolo_params.crop_size_height = json[paramSet[11]].int_value();

    GetSigMatrix(yolo_params.Sig_Matrix, yolo_params.Exp_Matrix, ImgScaleMatrix, 0.0625, 64/255.f);
}

void YoLoProcess::showParams(){
    printf("elf_model_name: %s\n", yolo_params.elf_model_name);
    printf("input_node: %s\n", yolo_params.input_node);
    std::cout << "output_num: " << output_num << std::endl;
	for(int i = 0 ; i < output_num; ++i)
		printf("output_node %d : %s\n", i, yolo_params.output_node[i]);
	printf("num_classes : %d\n", yolo_params.num_classes);
	printf("anchorCnt : %d\n" , yolo_params.anchorCnt);
    std::cout << "conf_num:   " << conf_num << std::endl;
    for(int i = 0 ; i < conf_num ; ++i)
		printf("conf_threshold : class%d-%f\n", i, yolo_params.conf_threshold[i]);
	printf("nms_threshold : %f\n", yolo_params.nms_threshold);
    std::cout << "biases_num: " << biases_num << std::endl;
	for(int i = 0 ; i < biases_num ; ++i)
		printf("biases : %f\n", yolo_params.biases[i]);
    printf("crop_size_left_top_x :%d\n", yolo_params.crop_size_left_top_x);
	printf("crop_size_left_top_y :%d\n", yolo_params.crop_size_left_top_y);
	printf("crop_size_width :%d\n", yolo_params.crop_size_width);
	printf("crop_size_height :%d\n", yolo_params.crop_size_height);
}

void YoLoProcess::setInputImageForYOLO() {
    input_width = dpuGetInputTensorWidth(yolo_task, yolo_params.input_node);
    input_height = dpuGetInputTensorHeight(yolo_task, yolo_params.input_node);
    input_channel = dpuGetInputTensorChannel(yolo_task, yolo_params.input_node);
    input_size = dpuGetInputTensorSize(yolo_task, yolo_params.input_node);
    input_scale = dpuGetInputTensorScale(yolo_task, yolo_params.input_node);//pow(2, 6);
    input_data = dpuGetInputTensorAddress(yolo_task, yolo_params.input_node);

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
    img_input = img_init(cv::Rect(yolo_params.crop_size_left_top_x, yolo_params.crop_size_left_top_y, yolo_params.crop_size_width, yolo_params.crop_size_height));
    #ifdef TIME_COUNT
    time2 = get_current_time();
    std::cout << "crop:    " << fixed << setprecision(3) <<  (time2 - time1)/1000.0  << "ms" << std::endl;
    #endif

    #ifdef USE_RESIZE
    #ifdef TIME_COUNT
    time1 = get_current_time();
    #endif
    if(img_input.size() != cv::Size(input_width, input_height)){ // now we donot need resize
        cv::resize(img_input, img_input, cv::Size(input_width, input_height));
        ws = (float)yolo_params.crop_size_width / img_input.cols;
        hs = (float)yolo_params.crop_size_height / img_input.rows;
    }
    #ifdef TIME_COUNT
    time2 = get_current_time();
    std::cout << "resize:    " << fixed << setprecision(3) <<  (time2 - time1)/1000.0  << "ms" << std::endl;
    #endif
    #endif

    #if USER_DEFINE
    input_scale /= 255.f;

    #ifdef TIME_COUNT
    time1 = get_current_time();
    #endif
    
    // img_input *= input_scale; // opencv method1

    // cv::Mat image; // opencv method2
    // img_input.convertTo(image, CV_8UC3, input_scale);

    // for(int i = input_size-1; i >= 0; --i) // my method1
    //     img_input.data[i] *= input_scale;   

    // for(int i = input_size-1; i >= 0; --i) // my method2
    //     data[i] = ImgScaleMatrix[img_input.data[i]];

    for(int i = input_size-1; i >= 0; --i) // my method3
        img_input.data[i] = (img_input.data[i] >> 2);

    #ifdef TIME_COUNT
    time2 = get_current_time();
    std::cout << "scale:    " << fixed << setprecision(3) <<  (time2 - time1)/1000.0  << "ms" << std::endl;
    #endif

    memcpy(input_data, img_input.data, input_size);

    #elif DPU_SCALE

    #ifdef TIME_COUNT
    auto pre_in_time = chrono::system_clock::now();
    #endif
    
    dpuSetInputImageWithScale(yolo_task, yolo_params.input_node, img_input, mean, 1/255.f);
    
    #ifdef TIME_COUNT
    auto pre_out_time = chrono::system_clock::now();
    auto pre_duration = (chrono::duration_cast<chrono::microseconds>(pre_out_time - pre_in_time)).count();
    std::cout << "dpuSetInputImageWithScale: " << pre_duration /1000. << "ms" << std::endl;
    #endif

    #elif MY_SCALER
    sinput_width = dpuGetInputTensorWidth(scaler_task, sinput_node.c_str());
    sinput_height = dpuGetInputTensorHeight(scaler_task, sinput_node.c_str());
    sinput_channel = dpuGetInputTensorChannel(scaler_task, sinput_node.c_str());
    sinput_size = dpuGetInputTensorSize(scaler_task, sinput_node.c_str());
    sinput_scale = dpuGetInputTensorScale(scaler_task, sinput_node.c_str());
    sinput_data = dpuGetInputTensorAddress(scaler_task, sinput_node.c_str());

    #ifdef DEBUG_INFO
    std::cout << "sinput_width: " << sinput_width << std::endl;
    std::cout << "sinput_height: " << sinput_height << std::endl;
    std::cout << "sinput_channel: " << sinput_channel << std::endl;
    std::cout << "sinput_size: " << sinput_size << std::endl;
    std::cout << "sinput_scale: " << sinput_scale << std::endl;
    #endif

    memcpy(sinput_data, img_input.data, sinput_size);

    #ifdef TIME_COUNT
    auto pre_in_time = chrono::system_clock::now();
    #endif
    dpuRunTask(scaler_task);
    #ifdef TIME_COUNT
    auto pre_out_time = chrono::system_clock::now();
    auto pre_duration = (chrono::duration_cast<chrono::microseconds>(pre_out_time - pre_in_time)).count();
    std::cout << "scaler.GetReults(): " << pre_duration /1000. << "ms" << std::endl;
    #endif

    soutput_width = dpuGetOutputTensorWidth(scaler_task, soutput_node.c_str());
    soutput_height = dpuGetOutputTensorHeight(scaler_task, soutput_node.c_str());
    soutput_channel = dpuGetOutputTensorChannel(scaler_task, soutput_node.c_str());
    soutput_size = dpuGetOutputTensorSize(scaler_task, soutput_node.c_str());
    soutput_scale = dpuGetOutputTensorScale(scaler_task, soutput_node.c_str());
    soutput_data = dpuGetOutputTensorAddress(scaler_task, soutput_node.c_str());

    #ifdef DEBUG_INFO
    std::cout << "soutput_width: " << soutput_width << std::endl;
    std::cout << "soutput_height: " << soutput_height << std::endl;
    std::cout << "soutput_channel: " << soutput_channel << std::endl;
    std::cout << "soutput_size: " << soutput_size << std::endl;
    std::cout << "soutput_scale: " << soutput_scale << std::endl;
    #endif

    memcpy(input_data, soutput_data, input_size);

    #elif MULTI_THREAD_SCALE
    
    #ifdef TIME_COUNT
    auto pre_in_time = chrono::system_clock::now();
    #endif

    int type = 0;
	vector<cv::Mat> v = splitImage(img_input, THREAD_NUMS, type);
	dataThread data_threads[THREAD_NUMS];
	pthread_t pt[THREAD_NUMS];
	for (size_t i = 0; i < THREAD_NUMS; i++)
	{
		data_threads[i].h = v.at(i).rows;
		data_threads[i].w = v.at(i).cols;
		data_threads[i].data = v.at(i).data;
		pthread_create(&pt[i], NULL, &threadProcess, (void *)(&data_threads[i]));
	}

	for (size_t i = 0; i < THREAD_NUMS; i++)
	{
		pthread_join(pt[i], NULL);
	}
	cv::Mat dest = catImage(v, type);

    #ifdef TIME_COUNT
    auto pre_out_time = chrono::system_clock::now();
    auto pre_duration = (chrono::duration_cast<chrono::microseconds>(pre_out_time - pre_in_time)).count();
    std::cout << "multi-threads scale: " << pre_duration /1000. << "ms" << std::endl;
    #endif

    memcpy(input_data, dest.data, input_size);
    #endif

    #if ARM_NEON_SCALE

    #ifdef TIME_COUNT
    auto pre_in_time = chrono::system_clock::now();
    #endif
   

    // // method (failed)
    // float* input_data_ = reinterpret_cast<float*>(img_input.data);
    // float* output_data_ = reinterpret_cast<float*>(malloc(sizeof(float) * input_size));
    // float32x4_t regin, regout;
    // float32_t s = input_scale/255.;
    // for(int i = 0; i < input_size; i+=4){
    //     regin = vld1q_f32(input_data_+ i);
    //     regout = vmulq_n_f32(regin, s);
    //     vst1q_f32(output_data_ + i, regout);
    // }

    // method2
    uint8_t* output_data_ = reinterpret_cast<uint8_t*>(malloc(sizeof(uint8_t) * input_size));
    uint8x16_t regin, regout;
    for(int i = 0; i < input_size; i += 16){
        regin = vld1q_u8(img_input.data + i);
        regout = vshrq_n_u8(regin, 2);
        vst1q_u8(output_data_ + i, regout);
    }

    #ifdef TIME_COUNT
    auto pre_out_time = chrono::system_clock::now();
    auto pre_duration = (chrono::duration_cast<chrono::microseconds>(pre_out_time - pre_in_time)).count();
    std::cout << "ARM_NEON: " << pre_duration /1000. << "ms" << std::endl;
    #endif

    // cv::Mat out_tmp = cv::Mat::zeros(cv::Size(input_width, input_height), CV_8UC3);

    // for(int i = 0; i < input_size; ++i){
    //     out_tmp.data[i] = static_cast<uchar>(output_data_[i]);
    // }
    // // out_tmp.data = reinterpret_cast<uchar*>(output_data_);
    // // for(int h = 0; h < input_height; ++h){
    // //     for(int w = 0; w < input_width; ++w){
    // //         for(int c = 0; c < input_channel; ++c){
    // //             out_tmp.data[h*input_width*input_channel + w*input_channel + c] = static_cast<uchar>(output_data_[h*input_width*input_channel + w*input_channel + c]);
    // //         }
    // //     }
    // // }

    // imwrite("out_tmp.jpg", out_tmp);

    memcpy(input_data, reinterpret_cast<int8_t*>(output_data_), input_size); 

    #endif
}

void YoLoProcess::detect(vector<vector<float>> &boxes, int8_t* input, int channel, int height, int width, int num, int sHeight, int sWidth, int sizeOut) {
    size_t length = height*width*channel;
    int8_t *df = input;
    float *sig_m = yolo_params.Sig_Matrix + 128;
    float *exp_m = yolo_params.Exp_Matrix + 128;
    unsigned int conf_box = yolo_params.num_classes + 5;

    auto min_ptr = min_element(yolo_params.conf_threshold, yolo_params.conf_threshold + yolo_params.num_classes);
    for(unsigned int i = 0; i < length;i+=conf_box){
        float obj_score = sig_m[df[i+4]];
        if(obj_score < *min_ptr){
            continue;
        }
        int channelP = i%channel;
        int location = i/channel;
        int c = channelP/conf_box;
        int w = location%width;
        int h = location/width;
        std::vector<float> box;
        box.push_back((w + sig_m[df[i+0]])/width);	
        box.push_back((h + sig_m[df[i+1]])/height);
        box.push_back((exp_m[df[i+2]]) * yolo_params.biases[2 * c + 2 * yolo_params.anchorCnt * num] / float(sWidth));
        box.push_back((exp_m[df[i+3]]) * yolo_params.biases[2 * c + 2 * yolo_params.anchorCnt * num + 1] / float(sHeight));
        box.push_back(-1);
        box.push_back(obj_score);
        int max_index = -1;
        float max = -10;
        for (unsigned int p = 0; p < yolo_params.num_classes; p++) {
            float pro = obj_score * sig_m[df[i+p+5]];
            if(pro >= max){
                max = pro;
                max_index = p;
            }
            box.push_back(pro);
        }
		box[4] = max_index;

        if(obj_score < yolo_params.conf_threshold[max_index]) continue;

        boxes.push_back(box);
    }
}

vector<vector<float>> YoLoProcess::applyNMS(vector<vector<float>>& boxes, const float thres) {
    vector<vector<float>> result;
    for(unsigned int k = 0; k < yolo_params.num_classes; k++) {
        vector<pair<int, float>> order;
        for (size_t i = 0; i < boxes.size(); ++i) {
            if(boxes[i][4] != k) continue;
            pair<int, float> od;
            od.first = i;
            od.second = boxes[i][6 + k];
            order.push_back(od);
        }
	    if(order.size() == 0) continue;

        sort(order.begin(), order.end(), [](const pair<int, float> &ls, const pair<int, float> &rs) { return ls.second > rs.second; });

        vector<bool> exist_box(order.size(), true);

        for (size_t _i = 0; _i < order.size(); ++_i) {
            size_t i = order[_i].first;
            if (!exist_box[i]) continue;
            /* add a box as result */
            result.push_back(boxes[i]);
            for (size_t _j = _i + 1; _j < order.size(); ++_j) {
                size_t j = order[_j].first;
                if (!exist_box[j]) continue;
                float ovr = cal_iou(boxes[j], boxes[i]);
                if (ovr >= thres) 
                    exist_box[j] = false;
            }
        }
    }
    return result;
}

void YoLoProcess::refine(std::vector<std::vector<float>>& nms_results_, std::vector<std::vector<float>>& refine_results_){
    refine_results_.reserve(nms_results_.size());
    std::vector<float> per_info;
    per_info.reserve(6);

    float w = img_input.cols * ws;
    float h = img_input.rows * hs;

    for(size_t i = 0; i < nms_results_.size(); ++i){ 
        per_info.clear();       
        int cls = nms_results[i][4];
        float conf = nms_results[i][5];

        float xmin = (nms_results[i][0] - nms_results[i][2]/2.0) * w + 1.0 + yolo_params.crop_size_left_top_x;
        float ymin = (nms_results[i][1] - nms_results[i][3]/2.0) * h + 1.0 + yolo_params.crop_size_left_top_y;
        float xmax = (nms_results[i][0] + nms_results[i][2]/2.0) * w + 1.0 + yolo_params.crop_size_left_top_x;
        float ymax = (nms_results[i][1] + nms_results[i][3]/2.0) * h + 1.0 + yolo_params.crop_size_left_top_y;
        if(xmin < 0) xmin = 0.;
        if(ymin < 0) ymin = 0.;
        if(xmax > img_post.cols) xmax = img_post.cols;
        if(ymax > img_post.rows) ymax = img_post.rows;
        float obj_w = xmax - xmin;
        float obj_h = ymax - ymin;

        per_info.push_back(xmin);
        per_info.push_back(ymin);
        per_info.push_back(obj_w);
        per_info.push_back(obj_h);
        per_info.push_back(cls);
        per_info.push_back(conf);

        per_info.push_back(nms_results[i][6]);
        per_info.push_back(nms_results[i][7]);
        per_info.push_back(nms_results[i][8]);
        per_info.push_back(nms_results[i][9]);
        per_info.push_back(nms_results[i][10]);
        per_info.push_back(nms_results[i][11]);
        per_info.push_back(nms_results[i][12]);
        per_info.push_back(nms_results[i][13]);
        per_info.push_back(nms_results[i][14]);

        refine_results_.push_back(per_info);
    } 
}

void YoLoProcess::make_empty(){
    delete[] yolo_params.elf_model_name;
    
    delete[] yolo_params.input_node;
    
    for(int i = 0; i < output_num; ++i)
        delete[] yolo_params.output_node[i];
    delete[] yolo_params.output_node;

    delete[] yolo_params.conf_threshold;
    delete[] yolo_params.biases;
}

#if MULTI_THREAD
void* YoLoProcess::threadProcess(void* args) {
	dataThread *pt_param = (dataThread *)args;
	int w = pt_param->w;
	int h = pt_param->h;

    for(int i = w*h*3-1; i >= 0; --i)
        pt_param->data[i] = (pt_param->data[i] >> 2);

	pthread_exit(NULL);
	return NULL;
}
#endif