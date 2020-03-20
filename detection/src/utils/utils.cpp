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

void* fast_memcpy(void *dst, const void *src, size_t sz)
{
    void *r = dst;

    //先进行uint64_t长度的拷贝，一般而言，内存地址都是对齐的，
    size_t n = sz & ~(sizeof(uint64_t) - 1);
    uint64_t *src_u64 = (uint64_t *) src;
    uint64_t *dst_u64 = (uint64_t *) dst;

    while (n)
    {// 每次循环拷贝了2次uint64_t字长的数据
        *dst_u64++ = *src_u64++;
        *dst_u64++ = *src_u64++;
        n -= sizeof(uint64_t)*2;
    }

    //将没有非8字节字长取整的部分copy
    n = sz & (sizeof(uint64_t) - 1);
    uint8_t *src_u8 = (uint8_t *) src;
    uint8_t *dst_u8 = (uint8_t *) dst;
    while (n-- )
    {
        *dst_u8++ = *src_u8++;
    }

    return r;
}

void neon_memcpy(int8_t* dst, int8_t* src, int len){
    int8x16_t reg;
    int i;
    int len2 = len % 16;
    int len1 = len - len2;

    // use neon
    for(i = 0; i < len1; i += 16){
        reg = vld1q_s8(src);
        vst1q_s8(dst, reg);
        src += 16;
        dst += 16;
    }

    // duplicate the rest data
    while(len2--){
        *dst++ = *src++;
    }
}

void neon_norm(uint8_t* src, int8_t* dst, int len, float* mean, float scale){
    if(mean[0] != 0.f){
        #define USE_NEON_SUB
    }
    
    if(scale != 1.0){
        #define USE_NEON_MUL
    }

    if(mean[0] == mean[1] && mean[1] == mean[2]){
        // 保持精度的浮点运算方法
        float32x4_t bgr_mean = vdupq_n_f32(mean[0]);
        float32x4_t snum   = vdupq_n_f32(scale);

        int cnt = len - (len % 8);
        int i;

        for(i = 0; i < cnt; i += 8){
            uint8x8_t   bgr_u8            = vld1_u8(src + i);
            uint16x8_t  bgr_u16           = vmovl_u8(bgr_u8);
            
            uint16x4_t  bgr_u16_low       = vget_low_u16(bgr_u16);
            uint16x4_t  bgr_u16_high      = vget_high_u16(bgr_u16);

            uint32x4_t  bgr_u32_low       = vmovl_u16(bgr_u16_low);
            uint32x4_t  bgr_u32_high      = vmovl_u16(bgr_u16_high);

            float32x4_t bgr_f32_low       = vcvtq_f32_u32(bgr_u32_low);
            float32x4_t bgr_f32_high      = vcvtq_f32_u32(bgr_u32_high);
    #ifdef USE_NEON_SUB
                        bgr_f32_low       = vsubq_f32(bgr_f32_low, bgr_mean);
                        bgr_f32_high      = vsubq_f32(bgr_f32_high, bgr_mean);
    #endif
    #ifdef USE_NEON_MUL
                        bgr_f32_low       = vmulq_f32(bgr_f32_low, snum);
    #endif
            int32x4_t   res_s32_low       = vcvtq_s32_f32(bgr_f32_low);
            int32x4_t   res_s32_high      = vcvtq_s32_f32(bgr_f32_high);

            int16x4_t   res_s16_low       = vqmovn_s32(res_s32_low);
            int16x4_t   res_s16_high      = vqmovn_s32(res_s32_high);

            int16x8_t   res_s16           = vcombine_s16(res_s16_low, res_s16_high);
            int8x8_t    res               = vqmovn_s16(res_s16);

            vst1_s8(dst + i, res);
        }

        while(i < len){
    #ifdef USE_NEON_SUB
            dst[i] = (int8_t)((float)(src[i]) - mean[0]);
    #endif
    #ifdef USE_NEON_MUL
            dst[i] *= scale;
    #endif
            i++;
        }

        // // 低精度无符号整型计算
        // uint8x16_t bgr_mean = vdupq_n_u8((uint8_t)mean[0]);

        // int cnt = len - (len % 16);
        // int i;

        // for(i =0; i < cnt; i += 16){
        //     uint8x16_t bgr = vld1q_u8(src + i);
        //     int8x16_t  res = vreinterpretq_s8_u8(vsubq_u8(bgr, bgr_mean));
        //     vst1q_s8(dst + i, res);
        // }

        // while(i < len){
        //     dst[i] = (int8_t)(src[i]) - (int8_t)mean[0];
        //     i++;
        // }

    }
    else{

        float32x4_t b_mean = vdupq_n_f32(mean[2]);
        float32x4_t g_mean = vdupq_n_f32(mean[1]);
        float32x4_t r_mean = vdupq_n_f32(mean[0]);
        float32x4_t snum   = vdupq_n_f32(scale);
        int8x8x3_t bgr_res = {vdup_n_s8(0), vdup_n_s8(0), vdup_n_s8(0)};

        int cnt = len - (len % 8);
        int i;

        for(i = 0; i < cnt; i += 24){
            uint8x8x3_t bgr   = vld3_u8(src + i);

            // method one
            uint8x8_t   b_u8  = bgr.val[0];
            uint8x8_t   g_u8  = bgr.val[1];
            uint8x8_t   r_u8  = bgr.val[2];

            uint16x8_t  b_u16 = vmovl_u8(b_u8);
            uint16x8_t  g_u16 = vmovl_u8(g_u8);
            uint16x8_t  r_u16 = vmovl_u8(r_u8);

            // bgr low process
            uint16x4_t  b_u16_low       = vget_low_u16(b_u16);
            uint16x4_t  g_u16_low       = vget_low_u16(g_u16);
            uint16x4_t  r_u16_low       = vget_low_u16(r_u16);

            uint32x4_t  b_u32_low       = vmovl_u16(b_u16_low);
            uint32x4_t  g_u32_low       = vmovl_u16(g_u16_low);
            uint32x4_t  r_u32_low       = vmovl_u16(r_u16_low);

            float32x4_t b_f32_low       = vcvtq_f32_u32(b_u32_low);
            float32x4_t g_f32_low       = vcvtq_f32_u32(g_u32_low);
            float32x4_t r_f32_low       = vcvtq_f32_u32(r_u32_low);
    #ifdef USE_NEON_SUB
                        b_f32_low       = vsubq_f32(b_f32_low, b_mean);
                        g_f32_low       = vsubq_f32(g_f32_low, g_mean);
                        r_f32_low       = vsubq_f32(r_f32_low, r_mean);
    #endif
    #ifdef USE_NEON_MUL
                        b_f32_low       = vmulq_f32(b_f32_low, snum);
                        g_f32_low       = vmulq_f32(g_f32_low, snum);
                        r_f32_low       = vmulq_f32(r_f32_low, snum);
    #endif

            int32x4_t   bres_s32_low    = vcvtq_s32_f32(b_f32_low);
            int32x4_t   gres_s32_low    = vcvtq_s32_f32(g_f32_low);
            int32x4_t   rres_s32_low    = vcvtq_s32_f32(r_f32_low);

            int16x4_t   bres_s16_low    = vqmovn_s32(bres_s32_low);
            int16x4_t   gres_s16_low    = vqmovn_s32(gres_s32_low);
            int16x4_t   rres_s16_low    = vqmovn_s32(rres_s32_low);

            // bgr high process
            uint16x4_t  b_u16_high      = vget_high_u16(b_u16);
            uint16x4_t  g_u16_high      = vget_high_u16(g_u16);
            uint16x4_t  r_u16_high      = vget_high_u16(r_u16);

            uint32x4_t  b_u32_high      = vmovl_u16(b_u16_high);
            uint32x4_t  g_u32_high      = vmovl_u16(g_u16_high);
            uint32x4_t  r_u32_high      = vmovl_u16(r_u16_high);

            float32x4_t b_f32_high      = vcvtq_f32_u32(b_u32_high);
            float32x4_t g_f32_high      = vcvtq_f32_u32(g_u32_high);
            float32x4_t r_f32_high      = vcvtq_f32_u32(r_u32_high);
    #ifdef USE_NEON_SUB
                        b_f32_high      = vsubq_f32(b_f32_high, b_mean);
                        g_f32_high      = vsubq_f32(g_f32_high, g_mean);
                        r_f32_high      = vsubq_f32(r_f32_high, r_mean);
    #endif
    #ifdef USE_NEON_MUL
                        b_f32_high      = vmulq_f32(b_f32_high, snum);
                        g_f32_high      = vmulq_f32(g_f32_high, snum);
                        r_f32_high      = vmulq_f32(r_f32_high, snum);
    #endif

            int32x4_t   bres_s32_high   = vcvtq_s32_f32(b_f32_high);
            int32x4_t   gres_s32_high   = vcvtq_s32_f32(g_f32_high);
            int32x4_t   rres_s32_high   = vcvtq_s32_f32(r_f32_high);

            int16x4_t   bres_s16_high   = vqmovn_s32(bres_s32_high);
            int16x4_t   gres_s16_high   = vqmovn_s32(gres_s32_high);
            int16x4_t   rres_s16_high   = vqmovn_s32(rres_s32_high);

            int16x8_t   bres_s16        = vcombine_s16(bres_s16_low, bres_s16_high);
            int16x8_t   gres_s16        = vcombine_s16(gres_s16_low, gres_s16_high);
            int16x8_t   rres_s16        = vcombine_s16(rres_s16_low, rres_s16_high);

            int8x8_t    bres_s8         = vqmovn_s16(bres_s16);
            int8x8_t    gres_s8         = vqmovn_s16(gres_s16);
            int8x8_t    rres_s8         = vqmovn_s16(rres_s16);           

            bgr_res.val[0] = bres_s8;
            bgr_res.val[1] = gres_s8;
            bgr_res.val[2] = rres_s8;
            vst3_s8(dst + i, bgr_res);

            // // method two
            // vst3_s8(dst, {vqmovn_s16(vcombine_s16(vqmovn_s32(vcvtq_s32_f32(vmulq_f32(vsubq_f32(vcvtq_f32_u32(vmovl_u16( vget_low_u16(vmovl_u8(bgr.val[0])))), b_mean), snum))),
            //                                       vqmovn_s32(vcvtq_s32_f32(vmulq_f32(vsubq_f32(vcvtq_f32_u32(vmovl_u16(vget_high_u16(vmovl_u8(bgr.val[0])))), b_mean), snum))))),

            //               vqmovn_s16(vcombine_s16(vqmovn_s32(vcvtq_s32_f32(vmulq_f32(vsubq_f32(vcvtq_f32_u32(vmovl_u16( vget_low_u16(vmovl_u8(bgr.val[1])))), g_mean), snum))),
            //                                       vqmovn_s32(vcvtq_s32_f32(vmulq_f32(vsubq_f32(vcvtq_f32_u32(vmovl_u16(vget_high_u16(vmovl_u8(bgr.val[1])))), g_mean), snum))))),
                                                                
            //               vqmovn_s16(vcombine_s16(vqmovn_s32(vcvtq_s32_f32(vmulq_f32(vsubq_f32(vcvtq_f32_u32(vmovl_u16( vget_low_u16(vmovl_u8(bgr.val[2])))), r_mean), snum))),
            //                                       vqmovn_s32(vcvtq_s32_f32(vmulq_f32(vsubq_f32(vcvtq_f32_u32(vmovl_u16(vget_high_u16(vmovl_u8(bgr.val[2])))), r_mean), snum)))))});

            // // method three // 不显示结果
            // vst3_s8(dst, {vqmovn_s16(vcombine_s16(vqmovn_s32(vreinterpretq_s32_f32(vmulq_f32(vsubq_f32(vreinterpretq_f32_u32(vmovl_u16( vget_low_u16(vmovl_u8(bgr.val[0])))), b_mean), snum))),
            //                                       vqmovn_s32(vreinterpretq_s32_f32(vmulq_f32(vsubq_f32(vreinterpretq_f32_u32(vmovl_u16(vget_high_u16(vmovl_u8(bgr.val[0])))), b_mean), snum))))),

            //               vqmovn_s16(vcombine_s16(vqmovn_s32(vreinterpretq_s32_f32(vmulq_f32(vsubq_f32(vreinterpretq_f32_u32(vmovl_u16( vget_low_u16(vmovl_u8(bgr.val[1])))), g_mean), snum))),
            //                                       vqmovn_s32(vreinterpretq_s32_f32(vmulq_f32(vsubq_f32(vreinterpretq_f32_u32(vmovl_u16(vget_high_u16(vmovl_u8(bgr.val[1])))), g_mean), snum))))),

            //               vqmovn_s16(vcombine_s16(vqmovn_s32(vreinterpretq_s32_f32(vmulq_f32(vsubq_f32(vreinterpretq_f32_u32(vmovl_u16( vget_low_u16(vmovl_u8(bgr.val[2])))), r_mean), snum))),
            //                                       vqmovn_s32(vreinterpretq_s32_f32(vmulq_f32(vsubq_f32(vreinterpretq_f32_u32(vmovl_u16(vget_high_u16(vmovl_u8(bgr.val[2])))), r_mean), snum)))))});
        }

        while(i < len){
    #ifdef USE_NEON_SUB
            dst[i] = (int8_t)((float)(src[i]) - mean[2]);
    #endif
    #ifdef USE_NEON_MUL
            dst[i] *= scale;
    #endif
    #ifdef USE_NEON_SUB
            dst[i] = (int8_t)((float)(src[i]) - mean[1]);
    #endif
    #ifdef USE_NEON_MUL
            dst[i] *= scale;
    #endif
    #ifdef USE_NEON_SUB
            dst[i] = (int8_t)((float)(src[i]) - mean[0]);
    #endif
    #ifdef USE_NEON_MUL
            dst[i] *= scale;
    #endif
            i += 3;
        }
    }


}