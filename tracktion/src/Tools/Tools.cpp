#include "Tools.h"

errMsg::errMsg(){}

errMsg::~errMsg(){
    if(instance)
        delete instance;
}

errMsg* errMsg::instance = NULL;

errMsg* errMsg::getInstance()
{
    if(instance == NULL)
        instance = new errMsg();
    return instance;
}

void errMsg::out(std::string file, std::string func, std::string msg, bool pause)
{
    std::cout << "In file<" << file << ">" << func << " : " << msg << std::endl;
    if(pause) 
        exit(0);
}

extern float maxStirAngle;

float getdata(std::string &s){

	int pos = s.find_first_of("\t");

	std::string re = s.substr(0, pos);
	s = s.substr(pos+1, s.length());
	while (s.find_first_of("\t") == 0)
		s = s.substr(1,s.length());
	
	while (re.find("\t") != -1)
		re = re.replace(re.find("\t"),1,""); 
	
	float result = atof(re.c_str());

	return result;
}

void GetData(const char* s, float& Velocity, float& StirAngle, float AngleOffset){
	std::string ss =s;
	int index = getdata(ss);
	getdata(ss);
	getdata(ss);
	Velocity = getdata(ss)/ 3.6;
	StirAngle = getdata(ss);
	if(StirAngle > maxStirAngle) StirAngle -= 4096.;
	StirAngle -= AngleOffset;
	// std::cout << index << " " << Velocity << " " << StirAngle << std::endl;

}

void GetInputData(std::ifstream &inputfile, float &Velocity, float &StirAngle, const float AngleOffset){
	char s[1000];
	inputfile.getline(s, sizeof(s));
	GetData(s, Velocity, StirAngle, AngleOffset);
}

cv::Mat image_add(cv::Mat image, cv::Mat image_add1, cv::Mat image_add2, float factor){
    cv::Size scale(image.cols*factor, image.rows*factor);

    cv::resize(image,image,scale,0,0,1);
    cv::Mat left = cv::Mat::zeros(int(image_add1.rows+image_add2.rows),image_add2.cols, CV_8UC3);
    image_add1.copyTo(left(cv::Rect(0,0,image_add1.cols,image_add1.rows)));
    image_add2.copyTo(left(cv::Rect(0,image_add1.rows,image_add2.cols, image_add2.rows)));

    cv::resize(left, left, cv::Size( image.rows / float(left.rows) *left.cols,image.rows),0,0,1);
    
    cv::Mat img = cv::Mat::zeros(image.rows, int(image.cols+left.cols),CV_8UC3);
    left.copyTo(img(cv::Rect(0,0,left.cols, left.rows)));
    image.copyTo(img(cv::Rect(left.cols,0,image.cols, image.rows)));

    return img;
}

void ListImagesPaths(std::string const &path, std::vector<std::string> &images) {
    images.clear();
    struct dirent *entry;

    /*Check if path is a valid directory path. */
    struct stat s;
    lstat(path.c_str(), &s);
    if (!S_ISDIR(s.st_mode)) {
        fprintf(stderr, "Error: %s is not a valid directory!\n", path.c_str());
        exit(1);
    }

    DIR *dir = opendir(path.c_str());
    if (dir == nullptr) {
        fprintf(stderr, "Error: Open %s path failed.\n", path.c_str());
        exit(1);
    }

    while ((entry = readdir(dir)) != nullptr) {
        if (entry->d_type == DT_REG || entry->d_type == DT_UNKNOWN) {
            std::string name = entry->d_name;
            std::string ext = name.substr(name.find_last_of(".") + 1);
            if ((ext == "JPEG") || (ext == "jpeg") || (ext == "JPG") ||
                (ext == "jpg") || (ext == "PNG") || (ext == "png") || (ext == "420")) {
                images.push_back(name);
            }
        }
    }

    closedir(dir);
}

#ifdef USE_FPGA_PLANE
#else
float* get_hog_feature(cv::Mat img, cv::Point2f center_point)
{
    cv::HOGDescriptor hog = cv::HOGDescriptor(cvSize(20, 20), cvSize(10, 10), cvSize(5, 5), cvSize(5, 5), 9);
    // https://pic002.cnblogs.com/images/2012/381513/2012081520322571.png
    // 对每一个cell-cvSize(5, 5), 有9个向量
    // 对每一个block-cvSize(10, 10), 有2*2个cell, 所以有36个向量
    // 对于window-cvSize(20, 20)而言, 计算block个数的方法是, 对两个方向计算 (window_size(20) - block_size(10))/block_stride(10) + 1, 算得共有2*2 个block, 共有36*4=144个向量
    cv::resize(img,img,cv::Size(30,30),(0, 0), (0, 0), cv::INTER_LINEAR);
    cv::Mat obj_roi;
    cv::getRectSubPix(img, cv::Size(30, 30), center_point, obj_roi);
    std::vector<float> descriptors;
    hog.compute(obj_roi, descriptors, cv::Size(20, 20), cv::Size(0, 0));
    float *feature_float = (float *)malloc(descriptors.size()*sizeof(float));
    assert(feature_float);
    for(int i=0;i<128;i++)
    {
        feature_float[i]=descriptors[i*2];
    }

    return feature_float;
}
#endif