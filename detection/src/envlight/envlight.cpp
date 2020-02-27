#include"envlight.h"

using namespace std;

EnvlightProcess::EnvlightProcess(const char* config_file){
	getParams(config_file);
	showParams();

	dpuOpen();
    envlight_kernel = dpuLoadKernel(envlight_params.elf_model_name);
	envlight_task = dpuCreateTask(envlight_kernel, 0);
}

EnvlightProcess::~EnvlightProcess(){
	dpuDestroyTask(envlight_task);
    dpuDestroyKernel(envlight_kernel);
    dpuClose();
	    
    make_empty();
}

void EnvlightProcess::PreProcess(cv::Mat& img){
	img_init = img;
	setInputImageForEnvLight();
}

std::vector<float> EnvlightProcess::GetResult(){
	dpuRunTask(envlight_task);

	output_channel = dpuGetOutputTensorChannel(envlight_task, envlight_params.output_node);
	fcn_out.reserve(output_channel);
	softmax_out.reserve(output_channel);

	dpuGetOutputTensorInHWCFP32(envlight_task, envlight_params.output_node, &fcn_out[0], output_channel);
    CPUCalcSoftmax(&fcn_out[0], output_channel, &softmax_out[0]);

	// cout << fixed << setprecision(3) << "fcn_out class1: " << fcn_out[0] << "; fcn_out class2: " << fcn_out[1] << endl;
	// cout << fixed << setprecision(3) << "softmax_out class1: " << softmax_out[0] << "; softmax_out class2: " << softmax_out[1] << endl;

	return softmax_out;
}

void EnvlightProcess::PostProcess(std::string imagename){
    std::ofstream fout("images/results/" + imagename.substr(0, imagename.length() - 4) + "_"  + medstr + ".txt");
	fout << fixed << setprecision(3) << "fcn_out class1:: " << fcn_out[0] << "; fcn_out class2: " << fcn_out[1] << endl;
	fout << fixed << setprecision(3) << "softmax_out class1: " << softmax_out[0] << "; softmax_out class2: " << softmax_out[1] << endl;
	fout.close();
}

void EnvlightProcess::setInputImageForEnvLight(){
	input_width = dpuGetInputTensorWidth(envlight_task, envlight_params.input_node);
    input_height = dpuGetInputTensorHeight(envlight_task, envlight_params.input_node);

	img_input = img_init(cv::Rect(envlight_params.crop_size_left_top_x, envlight_params.crop_size_left_top_y, envlight_params.crop_size_width, envlight_params.crop_size_height));
	if(img_input.size() != cv::Size(input_width, input_height)){
        cv::resize(img_input, img_input, cv::Size(input_width, input_height));
    }

	cv::subtract(img_input, cv::Scalar(15, 14, 14), img_input);

    dpuSetInputImage2(envlight_task, envlight_params.input_node, img_input);
}

void EnvlightProcess::getParams(const char* file_name){
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
	envlight_params.elf_model_name = new char[elf_model_name_lenth + 1];
	strcpy(envlight_params.elf_model_name, elf_model_name);
	
	const char* input_node = json[paramSet[1]].string_value().c_str();
	int input_node_lenth = strlen(input_node);
	envlight_params.input_node = new char[input_node_lenth + 1];
	strcpy(envlight_params.input_node, input_node);

	const char* output_node = json[paramSet[2]].string_value().c_str();
	int output_node_lenth = strlen(output_node);
	envlight_params.output_node = new char[output_node_lenth + 1];
	strcpy(envlight_params.output_node, output_node);

	envlight_params.crop_size_left_top_x = json[paramSet[3]].int_value();
	envlight_params.crop_size_left_top_y = json[paramSet[4]].int_value();
	envlight_params.crop_size_width = json[paramSet[5]].int_value();
	envlight_params.crop_size_height = json[paramSet[6]].int_value(); 
}

void deleteAllMark(std::string &s, const std::string &mark){
	int nSize = mark.size();
	while(1){
		long unsigned int po = s.find(mark);
		if(po == std::string::npos) 
			return;
		s.erase(po, nSize);
	}
}

void EnvlightProcess::showParams(){
	printf("elf_model_name: %s\n", envlight_params.elf_model_name);
	printf("input_node :%s\n", envlight_params.input_node);
	printf("output_node :%s\n", envlight_params.output_node);
	printf("crop_size_left_top_x :%d\n", envlight_params.crop_size_left_top_x);
	printf("crop_size_left_top_y :%d\n", envlight_params.crop_size_left_top_y);
	printf("crop_size_width :%d\n", envlight_params.crop_size_width);
	printf("crop_size_height :%d\n", envlight_params.crop_size_height);
}

void EnvlightProcess::make_empty(){
    delete[] envlight_params.elf_model_name;
    
    delete[] envlight_params.input_node;

    delete[] envlight_params.output_node;
}