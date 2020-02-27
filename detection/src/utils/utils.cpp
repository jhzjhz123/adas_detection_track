#include "utils.h"

unsigned long get_current_time(void)
{
    struct timeval tv;

    gettimeofday(&tv, NULL);

    return (tv.tv_sec*1000000 + tv.tv_usec);
}

void swapPointerUsePointer(int8_t **p, int8_t **q)
{
    int8_t *t = *p;
    *p = *q;
    *q = t;
}

void swapPointerUseReference(int8_t *&p, int8_t *&q)
{
    int8_t *t = p;
    p = q;
    q = t;
}

void ListPath(string const &path, vector<string> &paths) {
    paths.clear();
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
 /*   
        if (entry->d_type == DT_REG || entry->d_type == DT_UNKNOWN) {
            string name = entry->d_name;
            cout << "name: "<<name<<endl;
            paths.push_back(name);
        }*/
        string name = entry->d_name;
        int type = (int)(entry->d_type);
       
       if(type != 8)
       {
         if((strcmp(name.c_str(), ".")!=0) && (strcmp(name.c_str(), "..")!=0) && (strcmp(name.c_str(), "results")!=0)) 
          {            
            #if(YUV420OPEN)
            if((strcmp(name.c_str(), "yuv")==0))
            {
                cout << "Dir name: "<<name<<endl;
                paths.push_back(name);  
            }
            #else
            if(strcmp(name.c_str(), "yuv")!=0)
            {
                cout << "Dir name: "<<name<<endl;
                paths.push_back(name);  
            }
            #endif
          }
        }
        
    }

    closedir(dir);
}

void ListImages(string const &path, vector<string> &images) {
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
            string name = entry->d_name;
            string ext = name.substr(name.find_last_of(".") + 1);
            if ((ext == "JPEG") || (ext == "jpeg") || (ext == "JPG") ||
                (ext == "jpg") || (ext == "PNG") || (ext == "png") || (ext == "420")) {
                images.push_back(name);
            }
        }
    }

    closedir(dir);
}

float overlap(float x1, float w1, float x2, float w2) {
    float left = max(x1 - w1 / 2.0, x2 - w2 / 2.0);
    float right = min(x1 + w1 / 2.0, x2 + w2 / 2.0);
    return right - left;
}

float cal_iou(vector<float> box, vector<float>truth) {
    float w = overlap(box[0], box[2], truth[0], truth[2]);
    float h = overlap(box[1], box[3], truth[1], truth[3]);
    if(w < 0 || h < 0) return 0;

    float inter_area = w * h;
    float union_area = box[2] * box[3] + truth[2] * truth[3] - inter_area;
    return inter_area * 1.0 / union_area;
}

void GetSigMatrix(float *SigMatrix, float *ExpMatrix, float scale){
    for(int i=-128;i<128;i++){
       SigMatrix[i+128] = sigmoid(i * scale);
       ExpMatrix[i+128] = exp(i * scale);
    }
}

void GetSigMatrix(float *SigMatrix, float *ExpMatrix, int8_t *ImgScaleMatrix, float scale, float img_scale){
    for(int i=-128;i<128;i++){
       SigMatrix[i+128] = sigmoid(i * scale);
       ExpMatrix[i+128] = exp(i * scale);
       ImgScaleMatrix[i+128] = (i+128)*img_scale;
    }
}

float sigmoid(float p) {
    return 1.0 / (1. + exp(-p * 1.0));
}

void CPUCalcSoftmax(const float *data, size_t size, float *result) {
    assert(data && result);
    double sum = 0.0f;

    for (size_t i = 0; i < size; i++) {
        result[i] = exp(data[i]);
        sum += result[i];
    }

    for (size_t i = 0; i < size; i++) {
        result[i] /= sum;
    }
}

Mat convertTo3Channels(const Mat& binImg)
{
    Mat three_channel = Mat::zeros(binImg.rows, binImg.cols, CV_8UC3);
    vector<Mat> channels;
    for (int i=0;i<3;i++)
    {
        channels.push_back(binImg);
    }
    merge(channels,three_channel);
    return three_channel;
}

vector<cv::Mat> splitImage(cv::Mat image, int num,int type) {
	int rows = image.rows;
	int cols = image.cols;
	vector<cv::Mat> v;
	if (type == 0) {//垂直分割
		for (int i = 0; i < num; i++) {
			int star = rows / num*i;
			int end = rows / num*(i + 1);
			if (i == num - 1) {
				end = rows;
			}
			//cv::Mat b = image.rowRange(star, end);
			v.push_back(image.rowRange(star, end));
		}
	}
	else if (type == 1) {//水平分割
		for (int i = 0; i < num; i++){
			int star = cols / num*i;
			int end = cols / num*(i + 1);
			if (i == num - 1) {
				end = cols;
			}
			//cv::Mat b = image.colRange(star, end);
			/*解决水平分割的Bug:必须clone()*/
			v.push_back(image.colRange(star, end).clone());
		}
	}
	return  v;
}

cv::Mat catImage(vector<cv::Mat> v, int type) {
	cv::Mat dest= v.at(0);
	for (size_t i = 1; i < v.size(); i++)
	{
		if (type == 0)//垂直拼接
		{
			cv::vconcat(dest, v.at(i), dest);
		}
		else if (type == 1)//水平拼接
		{
			cv::hconcat(dest, v.at(i), dest);
		}
	}
	return dest;
}