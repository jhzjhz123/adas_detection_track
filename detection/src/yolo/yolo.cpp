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
    height0 = img.rows;
    width0 = img.cols;
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

void YoLoProcess::PostProcess(const std::string& imagename, cv::Mat& img_post){
    std::ofstream fout("images/results/" + imagename.substr(0, imagename.length() - 4) + "_"  + medstr + ".txt");
    std::ofstream box_file("images/results/boxcount_" + medstr + ".txt", ios::app);

    for(size_t i = 0; i < final_results.size(); ++i) {
        float xmin = final_results[i][0];
        float ymin = final_results[i][1] + 208; // where from crop y
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
    img_input = img_init;//(cv::Rect(yolo_params.crop_size_left_top_x, yolo_params.crop_size_left_top_y, yolo_params.crop_size_width, yolo_params.crop_size_height));
    #ifdef TIME_COUNT
    time2 = get_current_time();
    std::cout << "crop:    " << fixed << setprecision(3) <<  (time2 - time1)/1000.0  << "ms" << std::endl;
    #endif

    if(yolo_params.crop_size_height == 720){
        #define USE_RESIZE
    }
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

    #if USER_SCALE
    input_scale /= 255.f;

    #ifdef TIME_COUNT
    time1 = get_current_time();
    #endif
    
    // img_input *= input_scale; // opencv method1

    // cv::Mat image; // opencv method2
    // img_input.convertTo(image, CV_8UC3, input_scale);

    // for(int i = input_size-1; i >= 0; --i){ // 4.2ms
    //     // img_input.data[i] *= input_scale;  // not process x = x... format, will cause slow
    //     data[i] = (int8_t)img_input.data[i] * input_scale;   
    // }

    for(int i = input_size-1; i >= 0; --i) // 11.6ms
        data[i] = ImgScaleMatrix[img_input.data[i]];

    // for(int i = input_size-1; i >= 0; --i){ // 1.4ms
    //     // img_input.data[i] = (img_input.data[i] >> 2); // not process x = x... format, will cause slow
    //     data[i] = (int8_t)(img_input.data[i] >> 2);
    // }

    #ifdef TIME_COUNT
    time2 = get_current_time();
    std::cout << "USER_SCALE:    " << fixed << setprecision(3) <<  (time2 - time1)/1000.0  << "ms" << std::endl;
    #endif

    memcpy(input_data, data, input_size);

    #elif DPU_SCALE

    #ifdef TIME_COUNT
    auto pre_in_time = chrono::system_clock::now();
    #endif
    
    dpuSetInputImageWithScale(yolo_task, yolo_params.input_node, img_input, mean, 1/255.f);
    
    #ifdef TIME_COUNT
    auto pre_out_time = chrono::system_clock::now();
    auto pre_duration = (chrono::duration_cast<chrono::microseconds>(pre_out_time - pre_in_time)).count();
    std::cout << "DPU_SCALE: " << pre_duration /1000. << "ms" << std::endl;
    #endif

    #elif NET_SCALER
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
    std::cout << "NET_SCALER: " << pre_duration /1000. << "ms" << std::endl;
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

    #elif MULTI_THREADS_SCALE
    
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
	cv::Mat dst = catImage(v, type);

    #ifdef TIME_COUNT
    auto pre_out_time = chrono::system_clock::now();
    auto pre_duration = (chrono::duration_cast<chrono::microseconds>(pre_out_time - pre_in_time)).count();
    std::cout << "MULTI_THREAD_SCALE: " << pre_duration /1000. << "ms" << std::endl;
    #endif

    memcpy(input_data, dst.data, input_size);
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

    // // method2-neon intrinsic 1.0ms with neon_memcpy 1.0ms preprocess 5.0ms
    // uint8_t* output_data_ = reinterpret_cast<uint8_t*>(malloc(sizeof(uint8_t) * input_size));
    // uint8x16_t regin, regout;
    // int i;
    // int cnt = input_size - (input_size % 16);
    // for(i = 0; i < cnt; i += 16){
    //     regin = vld1q_u8(img_input.data + i);
    //     regout = vshrq_n_u8(regin, 2);
    //     vst1q_u8(output_data_ + i, regout);
    // }
    // for( ; i < input_size; ++i){
    //     output_data_[i] = (img_input.data[i] >> 2);
    // }

    // // method2-neon assembly 0.9ms
    // uint8_t* output_data_ = reinterpret_cast<uint8_t*>(malloc(sizeof(uint8_t) * input_size));
    // int i = 0;
    // int remained = input_size & 15;
    // int cnt = input_size - remained;

    // __asm__ __volatile__(
    //     "1:                             \n\t"
    //     "ldr q0, [%1, %2]               \n\t"
    //     "ushr v0.16b, v0.16b, #2        \n\t"
    //     "str q0, [%0, %2]               \n\t"
    //     "add %2, %2, #16                \n\t"
    //     "cmp %2, %3                     \n\t"
    //     "bne 1b                         \n\t"
    //     : "=r"(output_data_), "=r"(img_input.data), "=r"(i), "=r"(cnt)
    //     : "0"(output_data_), "1"(img_input.data), "2"(i), "3"(cnt)
    //     : "memory", "q0", "cc"
    // );

    // while(remained--){
    //     *output_data_++ = (*img_input.data++ >> 2);
    // }

    // // method3-neon intrinsic 1.1ms without neon_memcpy preprocess 2.8ms
    // uint8x16_t regin;
    // int8x16_t regout;
    // int i;
    // int cnt = input_size - (input_size % 16);
    // for(i = 0; i < cnt; i += 16){
    //     regin = vld1q_u8(img_input.data + i);
    //     regout = vreinterpretq_s8_u8(vshrq_n_u8(regin, 2));
    //     vst1q_s8(input_data + i, regout);
    // }
    // for( ; i < input_size; ++i){
    //     input_data[i] = ((int8_t)(*img_input.data++) >> 2);
    // }


    // // method3-neon assembly 
    // int remained = input_size % 16;
    // int cnt = input_size - remained;
    // __asm__ __volatile__(
    //     "1:                        \n\t"
    //     "ld1 {v0.16b}, [%0], #16   \n\t"
    //     "subs %w2, %w2, #16        \n\t" // 16 processed per loop
    //     "sshr v0.16b, V0.16b, #2   \n\t"
    //     "st1 {v0.16b}, [%1], #16   \n\t"
    //     "b.gt 1b"
    //     : "+r"((int8_t*)img_input.data),  // %0
    //       "+r"(input_data),               // %1
    //       "+r"(cnt)                       // %2 
    //     :
    //     : "v0", "memory", "cc"            // Clobber List
    // );
    // while(remained--){
    //     *input_data++ = ((int8_t)(*img_input.data++) >> 2);
    // }

    // method4-neon intrinsic 8.6ms uint8->uint16->uint32->float32->做乘法处理->float32->uint32->uint16->uint8->int8
    // float32_t s = input_scale/255.;
    // int i;
    // int cnt = input_size - (input_size % 8);
    // for(i = 0; i < cnt; i += 8){
    //     uint8x8_t  regin8         = vld1_u8(img_input.data + i);         // LD1 {Vt.8B},[Xn] 
    //     uint16x8_t regin16        = vmovl_u8(regin8);                    // USHLL Vd.8H,Vn.8B,#0

    //     uint16x4_t  regin16_low   = vget_low_u16(regin16);               // DUP Vd.1D,Vn.D[0]
    //     uint16x4_t  regin16_high  = vget_high_u16(regin16);              // DUP Vd.1D,Vn.D[1]    

    //     uint32x4_t  regin32_low   = vmovl_u16(regin16_low);              // 宽指令，将16位扩展为32位 USHLL Vd.4S,Vn.4H,#0
    //     uint32x4_t  regin32_high  = vmovl_u16(regin16_high);             // USHLL Vd.4S,Vn.4H,#0

    //     float32x4_t regin32f_low  = vcvtq_f32_u32(regin32_low);          // 将int转换为float UCVTF Vd.4S,Vn.4S
    //     float32x4_t regin32f_high = vcvtq_f32_u32(regin32_high);         // UCVTF Vd.4S,Vn.4S

    //     float32x4_t res32f_low    = vmulq_n_f32(regin32f_low, s);        //  FMUL Vd.4S,Vn.4S,Vm.S[0]
    //     float32x4_t res32f_high   = vmulq_n_f32(regin32f_high, s);       // FMUL Vd.4S,Vn.4S,Vm.S[0]

    //     uint32x4_t  res32_low     = vcvtq_u32_f32(res32f_low);           // 将float转换为int FCVTZU Vd.4S,Vn.4S
    //     uint32x4_t  res32_high    = vcvtq_u32_f32(res32f_high);          // FCVTZU Vd.4S,Vn.4S

    //     uint16x4_t  res16_low     = vqmovn_u32(res32_low);               // 窄指令，32位变为16位 UQXTN Vd.4H,Vn.4S
    //     uint16x4_t  res16_high    = vqmovn_u32(res32_high);              // UQXTN Vd.4H,Vn.4S

    //     uint16x8_t  res16         = vcombine_u16(res16_low, res16_high); // DUP Vd.1D,Vn.D[0]    INS Vd.D[1],Vm.D[0]
    //     uint8x8_t   res8          = vqmovn_u16(res16);                   // UQXTN Vd.8B,Vn.8H
    //     int8x8_t    result        = vreinterpret_s8_u8(res8);

    //     vst1_s8(input_data + i, result);                                 // ST1 {Vt.8B},[Xn]
    // }

    // for( ; i < input_size; ++i){
    //     input_data[i] = img_input.data[i] * s;
    // }

    // // method4-neon assembly 7.8ms
    // float s = input_scale/255.f;
    // int i = 0;
    // int cnt = input_size - (input_size % 8);
    // __asm__ __volatile__(
    //     "dup v3.4s, %w2               \n\t"
    //     "1:                           \n\t"

    //     "ld1 {v0.8b}, [%1], #8        \n\t"
    //     "ushll v0.8h,v0.8b,#0         \n\t"

    //     "dup d1, v0.d[0]              \n\t"
    //     "dup d0, v0.d[1]              \n\t"

    //     "ushll v1.4s,v1.4h,#0         \n\t"
    //     "ushll v0.4s, v0.4h,#0        \n\t"

    //     "ucvtf v1.4s, v1.4s           \n\t"    
    //     "ucvtf v0.4s, v0.4s           \n\t"

    //     "fmul v1.4s, v1.4s, v3.4s     \n\t"
    //     "fmul v0.4s, v0.4s, v3.4s     \n\t"

    //     "fcvtzu v1.4s, v1.4s          \n\t"
    //     "fcvtzu v0.4s, v0.4s          \n\t"

    //     "uqxtn v1.4h, v1.4s           \n\t"
    //     "uqxtn v0.4h, v0.4s           \n\t"

    //     "dup d2, v1.d[0]              \n\t"
    //     "ins v2.d[1], v0.d[0]         \n\t"
    //     "uqxtn v0.8b, v2.8h           \n\t"

    //     "st1 {v0.8b}, [%0], #8        \n\t"

    //     // "add %3, %3, #8               \n\t"
    //     // "cmp %3, %4                   \n\t"
    //     // "bne 1b                       \n\t"
    //     "subs %4, %4, #8              \n\t"
    //     "bgt 1b                       \n\t"
        
    //     : "=r"(input_data), "=r"(img_input.data), "=r"(s), "=r"(i), "=r"(cnt)
    //     : "0"(input_data), "1"(img_input.data), "2"(s), "3"(i), "4"(cnt)
    //     : "memory", "cc", "v0", "v1", "v2", "v3", "v4", "d0", "d1", "d2", "s4"
    // );

    // method5-neon intrinsic 8.7ms uint8->uint16->uint32->float32->做乘法处理->float32->int32->int16->int8
    float32_t s = input_scale/255.;
    int i;
    int cnt = input_size - (input_size % 8);
    for(i = 0; i < cnt; i += 8){
        uint8x8_t   regin8        = vld1_u8(img_input.data + i);
        uint16x8_t  regin16       = vmovl_u8(regin8);

        uint16x4_t  regin16_low   = vget_low_u16(regin16);
        uint16x4_t  regin16_high  = vget_high_u16(regin16);

        uint32x4_t  regin32_low   = vmovl_u16(regin16_low);     // 宽指令，将16位扩展为32位
        uint32x4_t  regin32_high  = vmovl_u16(regin16_high);

        float32x4_t regin32f_low  = vcvtq_f32_u32(regin32_low); // 将int转换为float
        float32x4_t regin32f_high = vcvtq_f32_u32(regin32_high);

        float32x4_t res32f_low    = vmulq_n_f32(regin32f_low, s);
        float32x4_t res32f_high   = vmulq_n_f32(regin32f_high, s);

        int32x4_t   res32_low     = vcvtq_s32_f32(res32f_low);  // 将float转换为int
        int32x4_t   res32_high    = vcvtq_s32_f32(res32f_high);

        int16x4_t   res16_low     = vqmovn_s32(res32_low);      // 窄指令，32位变为16位
        int16x4_t   res16_high    = vqmovn_s32(res32_high);

        int16x8_t   res16         = vcombine_s16(res16_low, res16_high);
        int8x8_t    res8          = vqmovn_s16(res16);

        vst1_s8(input_data + i, res8);
    }
    for( ; i < input_size; ++i){
        input_data[i] = img_input.data[i] * s;
    }

    // // method5-neon assembly7.2ms
    // float s = input_scale/255.f;
    // int i = 0;
    // int cnt = input_size - (input_size & 7);
    // __asm__ __volatile__(
    //     "dup v3.4s, %w2           \n\t"
    //     "1:                       \n\t"

    //     "ldr d0, [%1], #8         \n\t"
    //     "ushll v0.8h, v0.8b, #0   \n\t"

    //     "dup d1, v0.d[0]          \n\t"
    //     "dup d0, v0.d[1]          \n\t"
        
    //     "ushll v1.4s, v1.4h, #0   \n\t"
    //     "ushll v0.4s, v0.4h, #0   \n\t"

    //     "ucvtf v1.4s, v1.4s       \n\t"
    //     "ucvtf v0.4s, v0.4s       \n\t"

    //     "fmul v1.4s, v1.4s, v3.4s \n\t"
    //     "fmul v0.4s, v0.4s, v3.4s \n\t"

    //     "fcvtzs v1.4s, v1.4s      \n\t"
    //     "fcvtzs v0.4s, v0.4s      \n\t"

    //     "sqxtn v1.4h, v1.4s       \n\t"
    //     "sqxtn v0.4h, v0.4s       \n\t"

    //     "dup d2, v1.d[0]          \n\t"
    //     "ins v2.d[1], v0.d[0]     \n\t"
    //     "sqxtn v0.8b, v2.8h       \n\t"

    //     "str d0, [%0], #8         \n\t"
        
    //     // "add %3, %3, #8  \n\t"
    //     // "cmp %3, %4      \n\t"
    //     // "bne 1b          \n\t"
    //     "subs %4, %4, #8 \n\t"
    //     "bgt 1b          \n\t"
    //     : "=r"(input_data), "=r"(img_input.data), "=r"(s), "=r"(i), "=r"(cnt)
    //     : "0"(input_data), "1"(img_input.data), "2"(s), "3"(i), "4"(cnt)
    //     : "memory", "cc", "v0", "v1", "v2", "d0", "d1", "d2"
    // );

    // // method6-neon intrinsic uint8->uint16->float16->做乘法处理->float16->uint16->uint8->int8
    // float16_t s = input_scale/255.f;
    // int i = 0;
    // int cnt = input_size - (input_size & 7);
    // for( ; i < cnt; i += 16){
    //     uint8x16_t in_u8 = vld1q_u8(img_input.data + i);

    //     uint8x8_t in_u8_low = vget_low_u8(in_u8);
    //     uint8x8_t in8_u8_high = vget_high_u8(in_u8);

    //     uint16x8_t in_u16_low = vmovl_u8(in_u8_low);
    //     uint16x8_t in_u16_high = vmovl_u8(in8_u8_high);

    //     float16x8_t in_f16_low = vcvtq_f16_u16(in_u16_low);
    //     float16x8_t in_f16_high = vcvtq_f16_u16(in_u16_high);

    //     float16x8_t out_f16_low = vmulq_n_f16(in_f16_low, s);
    //     float16x8_t out_f16_high = vmulq_n_f16(in_f16_high, s);

    //     int16x8_t out_s16_low = vcvtq_s16_f16(out_f16_low);
    //     int16x8_t out_s16_high = vcvtq_s16_f16(out_f16_high);

    //     int8x8_t out_s8_low = vqmovn_s16(out_s16_low);
    //     int8x8_t out_s8_high = vqmovn_s16(out_s16_high);

    //     int8x16_t out_s8 = vcombine_s8(out_s8_low, out_s8_high);

    //     vst1q_s8(input_data + i, out_s8);
    // }

    // // method6-neon assembly
    // float16_t s = input_scale/255.f;
    // int i = 0;
    // int cnt = input_size - (input_size & 15);
    // __asm__ __volatile__(
    //     "dup v3.8h, %w2 \n"
    //     "1: \n"
        
    //     "ldr q0, [%1], #16 \n"
    //     "dup v1.1d, v0.d[0] \n"
    //     "dup v0.1d, v0.d[1] \n"

    //     "ushll v1.8h, v1.8b, #0 \n"
    //     "ushll v0.8h, v0.8b, #0 \n"

    //     "ucvtf v1.8h, v1.8h \n"
    //     "ucvtf v0.8h, v0.8h \n"

    //     "fmul v1.8h, v1.8h, v3.8h \n"
    //     "fmul v0.8h, v0.8h, v3.8h \n"

    //     "fcvtzs v1.8h, v1.8h \n"
    //     "fcvtzs v0.8h, v0.8h \n"

    //     "sqxtn v1.8b, v1.8h \n"
    //     "sqxtn v0.8b, v0.8h \n"

    //     "dup v2.1d, v1.d[0] \n"
    //     "ins v2.d[1], v0.d[0] \n"

    //     "str q2, [%0], #16 \n"

    //     : "=r"(input_data), "=r"(img_input.data), "=r"(s), "=r"(i), "=r"(cnt)
    //     : "0"(input_data), "1"(img_input.data), "2"(s), "3"(i), "4"(cnt)
    //     : "memory", "cc"
    // );

    // // method7 my-neon_norm
    // float s = input_scale/255.f;
    // neon_norm(img_input.data, input_data, input_size, mean, s);

    #ifdef TIME_COUNT
    auto pre_out_time = chrono::system_clock::now();
    auto pre_duration = (chrono::duration_cast<chrono::microseconds>(pre_out_time - pre_in_time)).count();
    std::cout << "ARM_NEON_SCALE: " << pre_duration /1000. << "ms" << std::endl;
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

    // pre_in_time = chrono::system_clock::now();
    // // memcpy(input_data, reinterpret_cast<int8_t*>(output_data_), input_size); 
    // // fast_memcpy(input_data, reinterpret_cast<int8_t*>(output_data_), input_size);
    // neon_memcpy(input_data, reinterpret_cast<int8_t*>(output_data_), input_size);
    // pre_out_time = chrono::system_clock::now();
    // pre_duration = (chrono::duration_cast<chrono::microseconds>(pre_out_time - pre_in_time)).count();
    // std::cout << "neon_memcpy: " << pre_duration /1000. << "ms" << std::endl;

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
        if(xmax > width0) xmax = width0;
        if(ymax > height0) ymax = height0;
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