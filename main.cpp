#include "yolo/yolo.h"
#include "envlight/envlight.h"
#include "lane/lane.h"
#include "ObjectEstimation/ObjectEstimation.h"
#include <thread>
#include "ldw/src/common.h"
#include "ldw/hvapi/hvLanedet.h"


HV_S32 test_ldw_image(HV_S32 argc, char *argv[])
{	
	//lane_detect_c lane_detect_c(0, 0, 1);
	HV_LANE_DETAPI lane_api;
	HV_U32 uFlag = 0;
	HV_INIT_USERDATA pUserData;

	//pUserData.camera_yaw = 0, pUserData.camera_pitch = -2.859, pUserData.camera_roll = 0;	//cadillac
	//pUserData.g_k1 = -0.2908, pUserData.g_k2 = -0.052948, pUserData.g_k3 = 0.834010;
	//pUserData.g_p1 = 0.002612, pUserData.g_p2 = 0.009260;
	//pUserData.g_fx = 1458, pUserData.g_fy = 1463;
	//pUserData.g_cx = 648, pUserData.g_cy = 229;
	//pUserData.g_camera_position = 0.18;
	//pUserData.g_camera_height = 1.45;
	//pUserData.g_camera_head = 1.68;

	pUserData.camera_yaw = -2.28, pUserData.camera_pitch = 0.31, pUserData.camera_roll = 1.64; //veran
	pUserData.g_k1 = -0.48571, pUserData.g_k2 = 0.292982, pUserData.g_k3 = -0.54362;
	pUserData.g_p1 = -0.00503, pUserData.g_p2 = -0.00544;
	pUserData.g_fx = 1479, pUserData.g_fy = 1472;
	pUserData.g_cx = 671, pUserData.g_cy = 377;
	pUserData.g_camera_position = -0.35935;
	pUserData.g_camera_height = 1.3476;
	pUserData.g_camera_head = 2;

	//pUserData.camera_yaw = 6.478, pUserData.camera_pitch = -0.459, pUserData.camera_roll = 1.06567;	//tiguan
	//pUserData.g_k1 = -0.2908, pUserData.g_k2 = -0.052948, pUserData.g_k3 = 0.834010;
	//pUserData.g_p1 = 0.002612, pUserData.g_p2 = 0.009260;
	//pUserData.g_fx = 1458, pUserData.g_fy = 1463;
	//pUserData.g_cx = 648, pUserData.g_cy = 229;
	//pUserData.g_camera_position = 0.18;
	//pUserData.g_camera_height = 1.45;
	//pUserData.g_camera_head = 1.68;

	//pUserData.camera_yaw = -2.6, pUserData.camera_pitch = 1.51, pUserData.camera_roll = 0; //inlight
	//pUserData.g_k1 = -0, pUserData.g_k2 = 0, pUserData.g_k3 = -0;
	//pUserData.g_p1 = -0, pUserData.g_p2 = -0;
	//pUserData.g_fx = 1458.67, pUserData.g_fy = 1458.67;
	//pUserData.g_cx = 671, pUserData.g_cy = 377;
	//pUserData.g_camera_position = -0.65;
	//pUserData.g_camera_height = 0.8;
	//pUserData.g_camera_head = 0.1;


	pUserData.hv_clock = clock;
	pUserData.hv_printf = printf;
	pUserData.hv_rand = rand;

	hvGetLaneDetAPI(&lane_api, uFlag);
	HV_HANDLE * phLaneDet = new HV_HANDLE;
	lane_api.Init(phLaneDet, &pUserData);
    
    std::string params_file = argc==2 ? argv[1] : "detection/model/lane/he/0115/lane_params.json";
    string basePath;
    #if(YUV420OPEN)
	    basePath = "images/yuv/";
    #else
        basePath = "images/rgb/";
    #endif
    vector<string> images;
    ListImages(basePath, images);
    if (images.size() == 0) {
        cerr << "\nError: Not images exists in "<< basePath << endl;
    }           

    LaneProcess lane_processor(params_file.c_str());
    cv::Mat img;
    cv::Mat lane_result;
    unsigned long time1, time2;
    double amount1 = 0.0, amount2 = 0.0, amount3 = 0.0, amount4 = 0.0;
    for(auto image_name:images) 
    {
        cout << "\nimage: " << basePath + image_name << endl;
        #if(YUV420OPEN)
        {
            std::ifstream yuvfile(basePath + image_name, std::ios::in | std::ios::binary);
            cv::Mat yuvmat(720*3/2, 1280, CV_8UC1);
            yuvfile.read((char*)yuvmat.data, 1280*720*3/2);
            time1 = get_current_time();
            cv::cvtColor(yuvmat, img, CV_YUV2RGB_YV12);
            time2 = get_current_time();
            std::cout << "yuv2rgb spend:       " << fixed << setprecision(3) <<  (time2 - time1)/1000.0  << "ms" << std::endl;
            amount1 += ((time2 - time1)/1000.0);

            yuvfile.close();
        }
        #else
        img = cv::imread(basePath + image_name);
        #endif

        time1 = get_current_time();
        lane_processor.PreProcess(img);
        time2 = get_current_time();
        std::cout << "PreProcess spend:    " << fixed << setprecision(3) <<  (time2 - time1)/1000.0  << "ms" << std::endl;
        amount2 += ((time2 - time1)/1000.0);
        
        time1 = get_current_time();
        lane_result = lane_processor.GetResult();
        time2 = get_current_time();
        std::cout << "GetResult spend:    " << fixed << setprecision(3) <<  (time2 - time1)/1000.0  << "ms" << std::endl;
        amount3 += ((time2 - time1)/1000.0);

        time1 = get_current_time();
        // lane_processor.PostProcess(image_name);

        HV_CODECBUFFER pInput;
        pInput.Buffer = lane_result.data;
        pInput.Height = 192;//720;
        pInput.Width = 512;//1280;
        lane_api.SetInputData(*phLaneDet, &pInput);
        HV_VIDEO_OUTPUTINFO out_info;
        lane_api.GetOutputData(*phLaneDet, NULL, &out_info);

        if (out_info.detected[2] == 1)
        {
            HV_F32 cur_a0 = out_info.wrda0[2];
            HV_F32 cur_a1 = out_info.wrda1[2];
            HV_F32 cur_a2 = out_info.wrda2[2];
            HV_F32 cur_bot = out_info.wr_bot_end[2];
            HV_F32 cur_dist = cur_a0 + cur_a1 * cur_bot + cur_a2 * cur_bot * cur_bot;
            //a0_txt << cur_a0 << setw(20) << cur_dist << endl;
        }

        time2 = get_current_time();
        std::cout << "PostProcess spend:    " << fixed << setprecision(3) <<  (time2 - time1)/1000.0  << "ms" << std::endl;
        amount4 += ((time2 - time1)/1000.0);
    }
    std::cout << std::endl;
    std::cout << "total images: " << images.size() << std::endl;
    std::cout << "average time of yuv2rgb:     " << fixed << setprecision(3) <<  amount1/images.size() << "ms" << std::endl;
    std::cout << "average time of PreProcess:  " << fixed << setprecision(3) <<  amount2/images.size() << "ms" << std::endl;
    std::cout << "average time of GetResult:   " << fixed << setprecision(3) <<  amount3/images.size() << "ms" << std::endl;
    std::cout << "average time of PostProcess: " << fixed << setprecision(3) <<  amount4/images.size() << "ms" << std::endl;
 
	return 0;
}

HV_S32 cur_frame_num;
HV_S32 test_ldw_video(HV_S32 argc, char *argv[])
{	
	HV_S32 width = ORI_IMG_WIDTH;
	HV_S32 height = ORI_IMG_HEIGHT;

	extern HV_S32 g_camera_height, g_camera_position;

	clock_t start, finish;

	//lane_detect_c lane_detect_c(0, 0, 1);
	HV_LANE_DETAPI lane_api;
	extern HV_COMMON_API g_common_api;
	HV_U32 uFlag = 0;
	HV_INIT_USERDATA pUserData;

	//pUserData.camera_yaw = 0, pUserData.camera_pitch = -2.859, pUserData.camera_roll = 0;	//cadillac
	//pUserData.g_k1 = -0.2908, pUserData.g_k2 = -0.052948, pUserData.g_k3 = 0.834010;
	//pUserData.g_p1 = 0.002612, pUserData.g_p2 = 0.009260;
	//pUserData.g_fx = 1458, pUserData.g_fy = 1463;
	//pUserData.g_cx = 648, pUserData.g_cy = 229;
	//pUserData.g_camera_position = 0.18;
	//pUserData.g_camera_height = 1.45;
	//pUserData.g_camera_head = 1.68;

	pUserData.camera_yaw = -2.28, pUserData.camera_pitch = 0.31, pUserData.camera_roll = 1.64; //veran
	pUserData.g_k1 = -0.48571, pUserData.g_k2 = 0.292982, pUserData.g_k3 = -0.54362;
	pUserData.g_p1 = -0.00503, pUserData.g_p2 = -0.00544;
	pUserData.g_fx = 1479, pUserData.g_fy = 1472;
	pUserData.g_cx = 671, pUserData.g_cy = 377;
	pUserData.g_camera_position = -0.35935;
	pUserData.g_camera_height = 1.3476;
	pUserData.g_camera_head = 2;

	//pUserData.camera_yaw = 6.478, pUserData.camera_pitch = -0.459, pUserData.camera_roll = 1.06567;	//tiguan
	//pUserData.g_k1 = -0.2908, pUserData.g_k2 = -0.052948, pUserData.g_k3 = 0.834010;
	//pUserData.g_p1 = 0.002612, pUserData.g_p2 = 0.009260;
	//pUserData.g_fx = 1458, pUserData.g_fy = 1463;
	//pUserData.g_cx = 648, pUserData.g_cy = 229;
	//pUserData.g_camera_position = 0.18;
	//pUserData.g_camera_height = 1.45;
	//pUserData.g_camera_head = 1.68;

	//pUserData.camera_yaw = -2.6, pUserData.camera_pitch = 1.51, pUserData.camera_roll = 0; //inlight
	//pUserData.g_k1 = -0, pUserData.g_k2 = 0, pUserData.g_k3 = -0;
	//pUserData.g_p1 = -0, pUserData.g_p2 = -0;
	//pUserData.g_fx = 1458.67, pUserData.g_fy = 1458.67;
	//pUserData.g_cx = 671, pUserData.g_cy = 377;
	//pUserData.g_camera_position = -0.65;
	//pUserData.g_camera_height = 0.8;
	//pUserData.g_camera_head = 0.1;


	pUserData.hv_clock = clock;
	pUserData.hv_printf = printf;
	pUserData.hv_rand = rand;

	hvGetLaneDetAPI(&lane_api, uFlag);
	HV_HANDLE * phLaneDet = new HV_HANDLE;
	lane_api.Init(phLaneDet, &pUserData);

	HV_S32 i, j, k, temp;

	//get transfer matrix between camera/world coordinate
	
	/*ofstream a0_txt;
	a0_txt.open("E:\\111.txt");*/

    std::string params_file = argc==2 ? argv[1] : "detection/model/lane/he/0115/lane_params.json";
	vector<string> file_name = {"test/video/20191231/tianjin/test_1.ts"};//get_files("E:\\video\\cnn\\fine");
	string out_folder = "E:\\video\\cnn\\111\\";
	HV_S32 start_file_num = 0;
    LaneProcess lane_processor(params_file.c_str());
	
	VideoCapture video_in;
	Mat frame, cur_frame;
	Mat resize_frame(IMG_HEIGHT, IMG_WIDTH, CV_8UC3);
	Mat cur_yuv(IMG_HEIGHT, IMG_WIDTH, CV_8UC3, Scalar(0, 0, 0));
	Mat ori_size_yuv(height, width, CV_8UC3, Scalar(0, 0, 0));

	namedWindow("out");
	char show_str[1024];
	CvFont font_show;
	cvInitFont(&font_show, CV_FONT_HERSHEY_COMPLEX, 1.0, 1.0, 0, 1, 8);
	int file_num = file_name.size();
	if (file_num == 0)
	{
		printf("no input file\n");
	}

	for (int file_cnt = start_file_num; file_cnt < file_num; file_cnt++)
	{
		printf("read_video: %s\n", file_name[file_cnt].c_str());
		video_in.open(file_name[file_cnt].c_str());
		string out_file = out_folder;
		size_t filename_pos = file_name[file_cnt].find_last_of("\\");
		size_t dot_pos = file_name[file_cnt].find_last_of(".");
		out_file += file_name[file_cnt].substr(filename_pos+1, dot_pos-filename_pos-1);
		out_file += "_out.avi";
		//out_file += file_name[file_cnt].substr(dot_pos);
		//cvVideoWriter *video_out = cvCreateVideoWriter(out_file.c_str(), CV_FOURCC('D', 'I', 'V', 'X'), 25.0, cvSize(1280, 720));
		VideoWriter video_out(out_file.c_str(), CV_FOURCC('D', 'I', 'V', 'X'), 25.0, cvSize(1280, 720));
		cur_frame_num = 0;
		while (video_in.read(frame))
		{
			lane_processor.PreProcess(frame);
            cur_frame = lane_processor.GetResult();
            
            resize(cur_frame, resize_frame, Size(IMG_WIDTH, IMG_HEIGHT), 0, 0, INTER_LINEAR);
			cur_frame_num++;
			
			extern HV_S32 g_framenum;
			g_framenum = cur_frame_num;
			if (cur_frame_num %1!=0)
			{
				continue;
			}
			if (cur_frame_num == 55)
			{
				HV_S32 qqq = 3;
			}
			cvtColor(resize_frame, cur_yuv, CV_BGR2YUV);
			//lane detect function will use pointer input
			Mat *input_frame = &cur_yuv; 
			HV_BYTE *buffer_in = new HV_BYTE[IMG_WIDTH*IMG_HEIGHT * 3 / 2];
			mat_to_yuv(buffer_in, input_frame, IMG_WIDTH, IMG_HEIGHT);
			//ori size buffer is used to draw lane
			HV_BYTE *ori_size_buffer = new HV_BYTE[width*height * 3 / 2];
			cvtColor(cur_frame, ori_size_yuv, CV_BGR2YUV);
			mat_to_yuv(ori_size_buffer, &ori_size_yuv, width, height);

			if (cur_frame_num < 0)
				continue;

			start = g_common_api.hv_clock();			
			HV_CODECBUFFER pInput;
			pInput.Buffer = buffer_in;
			pInput.Height = 720;
			pInput.Width = 1280;
			lane_api.SetInputData(*phLaneDet, &pInput);
			HV_VIDEO_OUTPUTINFO out_info;
			lane_api.GetOutputData(*phLaneDet, NULL, &out_info);

			HV_PTR buffer[2];
			buffer[0] = (void*)ori_size_buffer;
			buffer[1] = (void*)(ori_size_buffer + width*height);
			lane_api.DrawLane(buffer, (void*)(&out_info));

			if (out_info.detected[2] == 1)
			{
				HV_F32 cur_a0 = out_info.wrda0[2];
				HV_F32 cur_a1 = out_info.wrda1[2];
				HV_F32 cur_a2 = out_info.wrda2[2];
				HV_F32 cur_bot = out_info.wr_bot_end[2];
				HV_F32 cur_dist = cur_a0 + cur_a1 * cur_bot + cur_a2 * cur_bot * cur_bot;
				//a0_txt << cur_a0 << setw(20) << cur_dist << endl;
			}

			finish = g_common_api.hv_clock();
			g_common_api.hv_printf("%d: frameeeeeeeeeeeeeeee\n", finish - start);

			Mat cur_out(height, width, CV_8UC3, Scalar(0, 0, 0));
			yuv_to_mat(&cur_out, ori_size_buffer);
			//memcpy(cur_out.data, cur_yuv.data, width*height*sizeof(HV_BYTE) * 3);
			cvtColor(cur_out, cur_out, CV_YUV2BGR);
			//sprintf_s(show_str, "file: %s, frame: %d", file_name[file_cnt].c_str(), cur_frame_num);
			//cvPutText(cur_out, show_str, cvPoint(10, 600), &font_show, cvScalar(0, 255, 0));
			//just show color frame
			imshow("out", cur_out);
			cvWaitKey(1);
			video_out << cur_out;
			delete[] buffer_in;
			delete[] ori_size_buffer;
		}
	}

	return 0;
}

float fx = 1458;
float fy = 1463;
float cx = 648;
float cy = 229;
float k1 = -0.2908;
float k2 = -0.05294785;
float p1 = 0.0026125;
float p2 = 0.00926;
float k3 = 0.83401;
float pitch = -0.4;
float yaw = 6.65979;
float roll = 1.06567;
float Cam2Chamber = 1680;
float Cam2Center = 180;
float CamHeight = 1450;

float dt = 0.066;
float AngleOffset = -0.74;
float L = 2.5;
float K = 1.5;	
float maxTireAngle = 29.6;

float maxStirAngle = 540;

void AllInOneThread(VideoCapture cap, int argc, char* argv[]){
    std::string params_file = argc==2 ? argv[1] : "detection/model/yolov3/hasco-6/1219/yolo_params_hasco-6.json";
    YoLoProcess yolo_processor(params_file.c_str());
    
    std::string config_file = "tracktion/config/offline_parameters.xml";

#ifdef DISTANCEMEASURE
    ObjectEstimation Obj_Esti(fx, fy, cx, cy , k1, k2, p1, p2, k3, pitch, yaw, roll, Cam2Chamber, Cam2Center, CamHeight);
#else
    ObjectEstimation Obj_Esti(config_file);
#endif

    Obj_Esti.set_moveThreshold(7.5, 0.65);

	std::ifstream inputfile("tracktion/config/vehicle_status_log.txt", std::ios::in);

	DrivingPathEstimation driving_path_estimatin(L, K, maxTireAngle, maxStirAngle);

    int step = 1;
    cv::Mat frame, frame_track;
    std::map<int, Point2f> W;
    cv::Point2f CamVelocity;

    while (cap.read(frame))
    {
        step++;
        /******trajectory tracking******/
        #ifdef TIME_COUNT
        auto pre_in_time = chrono::system_clock::now();
        #endif
        if(step > 10)
        {
        	float Velocity, StirAngle;
		    GetInputData(inputfile, Velocity, StirAngle, AngleOffset);
		    driving_path_estimatin.UpdateCurrentData(Velocity, StirAngle, step);
		    W = driving_path_estimatin.UpdateCurrentTrajectory();
            CamVelocity = driving_path_estimatin.CamVelocity();
        }
        #ifdef TIME_COUNT
        auto pre_out_time = chrono::system_clock::now();
        auto pre_duration = (chrono::duration_cast<chrono::microseconds>(pre_out_time - pre_in_time)).count();
        std::cout << "driving_path_estimatin spend:    " << fixed << setprecision(3) <<  pre_duration/1000.0  << "ms" << std::endl;
        #endif

        /*get result from neural network*/
        #ifdef TIME_COUNT
        pre_in_time = chrono::system_clock::now();
        #endif
        yolo_processor.PreProcess(frame);
        std::vector<std::vector<float>> yolo_result = yolo_processor.GetResult();
        #ifdef TIME_COUNT
        pre_out_time = chrono::system_clock::now();
        pre_duration = (chrono::duration_cast<chrono::microseconds>(pre_out_time - pre_in_time)).count();
        std::cout << "net inference spend:    " << fixed << setprecision(3) <<  pre_duration/1000.0  << "ms" << std::endl;
        #endif

        /********object track********/
        #ifdef TIME_COUNT
        pre_in_time = chrono::system_clock::now();
        #endif
        frame_track = frame.clone();
        std::map<int, DETECTIONS> res = Obj_Esti.getdetections(yolo_result, frame, step);
        #ifdef TIME_COUNT
        pre_out_time = chrono::system_clock::now();
        pre_duration = (chrono::duration_cast<chrono::microseconds>(pre_out_time - pre_in_time)).count();
        std::cout << "getdetections spend:    " << fixed << setprecision(3) <<  pre_duration/1000.0  << "ms" << std::endl;
        #endif

        #ifdef TIME_COUNT
        pre_in_time = chrono::system_clock::now();
        #endif
        objects  objs = Obj_Esti.UpdateAll(res, W, CamVelocity);
        #ifdef TIME_COUNT
        pre_out_time = chrono::system_clock::now();
        pre_duration = (chrono::duration_cast<chrono::microseconds>(pre_out_time - pre_in_time)).count();
        std::cout << "UpdateAll spend:    " << fixed << setprecision(3) <<  pre_duration/1000.0  << "ms" << std::endl;
        #endif
        /********************************/   

        #ifdef TIME_COUNT
        pre_in_time = chrono::system_clock::now();
        #endif
        Obj_Esti.drawdetect(frame, res, W, objs);
        #ifdef TIME_COUNT
        pre_out_time = chrono::system_clock::now();
        pre_duration = (chrono::duration_cast<chrono::microseconds>(pre_out_time - pre_in_time)).count();
        std::cout << "drawdetect spend:    " << fixed << setprecision(3) <<  pre_duration/1000.0  << "ms" << std::endl << std::endl;
        #endif
    }
}

void test_obj_all(int argc, char* argv[]){
    std::string Video_Input = argc==3 ? argv[2] : "video/20191231/tianjin/test_1.ts";

    cv::VideoCapture cap(Video_Input);
    if (!cap.isOpened())
        std::cout <<"cap is not open ..." << endl;

    std::thread AllFrame(AllInOneThread, std::ref(cap), std::ref(argc), std::ref(argv));
    AllFrame.join();
}

void test_yolo(int argc, char* argv[]){
    std::string params_file = argc==2 ? argv[1] : "detection/model/yolov3/hasco-6/1219/yolo_params_hasco-6.json";
    string basePath;
    #if(YUV420OPEN)
	    basePath = "images/yuv/";
    #else
        basePath = "images/rgb/";
    #endif
    vector<string> images;
    ListImages(basePath, images);
    if (images.size() == 0) {
        cerr << "\nError: Not images exists in "<< basePath << endl;
    }           

    YoLoProcess yolo_processor(params_file.c_str());
    cv::Mat img;
    std::vector<std::vector<float>> yolo_result;
    unsigned long time1, time2;
    double amount1 = 0.0, amount2 = 0.0, amount3 = 0.0, amount4 = 0.0;
    for(auto image_name:images) 
    {
        cout << "\nimage: " << basePath + image_name << endl;
        #if(YUV420OPEN)
        {
            std::ifstream yuvfile(basePath + image_name, std::ios::in | std::ios::binary);
            cv::Mat yuvmat(720*3/2, 1280, CV_8UC1);
            yuvfile.read((char*)yuvmat.data, 1280*720*3/2);
            time1 = get_current_time();
            cv::cvtColor(yuvmat, img, CV_YUV2RGB_YV12);
            time2 = get_current_time();
            std::cout << "yuv2rgb spend:       " << fixed << setprecision(3) <<  (time2 - time1)/1000.0  << "ms" << std::endl;
            amount1 += ((time2 - time1)/1000.0);

            yuvfile.close();
        }
        #else
        img = cv::imread(basePath + image_name);
        #endif

        auto pre_in_time = chrono::system_clock::now();
        yolo_processor.PreProcess(img);
        auto pre_out_time = chrono::system_clock::now();
        auto pre_duration = (chrono::duration_cast<chrono::microseconds>(pre_out_time - pre_in_time)).count();
        std::cout << "PreProcess spend:    " << fixed << setprecision(3) <<  pre_duration/1000.0  << "ms" << std::endl;
        amount2 += (pre_duration/1000.0);
        
        time1 = get_current_time();
        yolo_result = yolo_processor.GetResult();
        time2 = get_current_time();
        std::cout << "GetResult spend:    " << fixed << setprecision(3) <<  (time2 - time1)/1000.0  << "ms" << std::endl;
        amount3 += ((time2 - time1)/1000.0);  

        time1 = get_current_time();
        yolo_processor.PostProcess(image_name);
        time2 = get_current_time();
        std::cout << "PostProcess spend:    " << fixed << setprecision(3) <<  (time2 - time1)/1000.0  << "ms" << std::endl;
        amount4 += ((time2 - time1)/1000.0); 
    }
    std::cout << std::endl;
    std::cout << "total images: " << images.size() << std::endl;
    std::cout << "average time of yuv2rgb:     " << fixed << setprecision(3) <<  amount1/images.size() << "ms" << std::endl;
    std::cout << "average time of PreProcess:  " << fixed << setprecision(3) <<  amount2/images.size() << "ms" << std::endl;
    std::cout << "average time of GetResult:   " << fixed << setprecision(3) <<  amount3/images.size() << "ms" << std::endl;
    std::cout << "average time of PostProcess: " << fixed << setprecision(3) <<  amount4/images.size() << "ms" << std::endl;
}

void test_envlight(int argc, char* argv[]){
    std::string params_file = argc==2 ? argv[1] : "detection/model/envlight_params.json";
    string basePath;
    #if(YUV420OPEN)
	    basePath = "images/yuv/";
    #else
        basePath = "images/rgb/";
    #endif
    vector<string> images;
    ListImages(basePath, images);
    if (images.size() == 0) {
        cerr << "\nError: Not images exists in "<< basePath << endl;
    }           

    EnvlightProcess envlight_processor(params_file.c_str());
    cv::Mat img;
    std::vector<float> envlight_result;
    unsigned long time1, time2;
    double amount1 = 0.0, amount2 = 0.0, amount3 = 0.0, amount4 = 0.0;
    for(auto image_name:images) 
    {
        cout << "\nimage: " << basePath + image_name << endl;
        #if(YUV420OPEN)
        {
            std::ifstream yuvfile(basePath + image_name, std::ios::in | std::ios::binary);
            cv::Mat yuvmat(720*3/2, 1280, CV_8UC1);
            yuvfile.read((char*)yuvmat.data, 1280*720*3/2);
            time1 = get_current_time();
            cv::cvtColor(yuvmat, img, CV_YUV2RGB_YV12);
            time2 = get_current_time();
            std::cout << "yuv2rgb spend:       " << fixed << setprecision(3) <<  (time2 - time1)/1000.0  << "ms" << std::endl;
            amount1 += ((time2 - time1)/1000.0);

            yuvfile.close();
        }
        #else
        img = cv::imread(basePath + image_name);
        #endif

        time1 = get_current_time();
        envlight_processor.PreProcess(img);
        time2 = get_current_time();
        std::cout << "PreProcess spend:    " << fixed << setprecision(3) <<  (time2 - time1)/1000.0  << "ms" << std::endl;
        amount2 += ((time2 - time1)/1000.0);
        
        time1 = get_current_time();
        envlight_result = envlight_processor.GetResult();
        time2 = get_current_time();
        std::cout << "GetResult spend:    " << fixed << setprecision(3) <<  (time2 - time1)/1000.0  << "ms" << std::endl;
        amount3 += ((time2 - time1)/1000.0); 

        time1 = get_current_time();
        envlight_processor.PostProcess(image_name);
        time2 = get_current_time();
        std::cout << "PostProcess spend:    " << fixed << setprecision(3) <<  (time2 - time1)/1000.0  << "ms" << std::endl;
        amount4 += ((time2 - time1)/1000.0);
    }
    std::cout << std::endl;
    std::cout << "total images: " << images.size() << std::endl;
    std::cout << "average time of yuv2rgb:     " << fixed << setprecision(3) <<  amount1/images.size() << "ms" << std::endl;
    std::cout << "average time of PreProcess:  " << fixed << setprecision(3) <<  amount2/images.size() << "ms" << std::endl;
    std::cout << "average time of GetResult:   " << fixed << setprecision(3) <<  amount3/images.size() << "ms" << std::endl;
    std::cout << "average time of PostProcess: " << fixed << setprecision(3) <<  amount4/images.size() << "ms" << std::endl;
}

void test_lane(int argc, char* argv[]){
    std::string params_file = argc==2 ? argv[1] : "detection/model/lane/he/lane_params.json";
    string basePath;
    #if(YUV420OPEN)
	    basePath = "images/yuv/";
    #else
        basePath = "images/images_prepped_test/";
    #endif
    vector<string> images;
    ListImages(basePath, images);
    if (images.size() == 0) {
        cerr << "\nError: Not images exists in "<< basePath << endl;
    }           

    LaneProcess lane_processor(params_file.c_str());
    cv::Mat img;
    cv::Mat lane_result;
    unsigned long time1, time2;
    double amount1 = 0.0, amount2 = 0.0, amount3 = 0.0, amount4 = 0.0;
    for(auto image_name:images) 
    {
        cout << "\nimage: " << basePath + image_name << endl;
        #if(YUV420OPEN)
        {
            std::ifstream yuvfile(basePath + image_name, std::ios::in | std::ios::binary);
            cv::Mat yuvmat(720*3/2, 1280, CV_8UC1);
            yuvfile.read((char*)yuvmat.data, 1280*720*3/2);
            time1 = get_current_time();
            cv::cvtColor(yuvmat, img, CV_YUV2RGB_YV12);
            time2 = get_current_time();
            std::cout << "yuv2rgb spend:       " << fixed << setprecision(3) <<  (time2 - time1)/1000.0  << "ms" << std::endl;
            amount1 += ((time2 - time1)/1000.0);

            yuvfile.close();
        }
        #else
        img = cv::imread(basePath + image_name);
        #endif

        time1 = get_current_time();
        lane_processor.PreProcess(img);
        time2 = get_current_time();
        std::cout << "PreProcess spend:    " << fixed << setprecision(3) <<  (time2 - time1)/1000.0  << "ms" << std::endl;
        amount2 += ((time2 - time1)/1000.0);
        
        time1 = get_current_time();
        lane_result = lane_processor.GetResult();
        time2 = get_current_time();
        std::cout << "GetResult spend:    " << fixed << setprecision(3) <<  (time2 - time1)/1000.0  << "ms" << std::endl;
        amount3 += ((time2 - time1)/1000.0); 

        time1 = get_current_time();
        lane_processor.PostProcess(image_name);
        time2 = get_current_time();
        std::cout << "PostProcess spend:    " << fixed << setprecision(3) <<  (time2 - time1)/1000.0  << "ms" << std::endl;
        amount4 += ((time2 - time1)/1000.0);
    }
    std::cout << std::endl;
    std::cout << "total images: " << images.size() << std::endl;
    std::cout << "average time of yuv2rgb:     " << fixed << setprecision(3) <<  amount1/images.size() << "ms" << std::endl;
    std::cout << "average time of PreProcess:  " << fixed << setprecision(3) <<  amount2/images.size() << "ms" << std::endl;
    std::cout << "average time of GetResult:   " << fixed << setprecision(3) <<  amount3/images.size() << "ms" << std::endl;
    std::cout << "average time of PostProcess: " << fixed << setprecision(3) <<  amount4/images.size() << "ms" << std::endl;
}

// #include <xilinx/segmentation/segmentation.hpp>
// #include <iostream>
// #include <memory>
// #include <glog/logging.h>
// #include <opencv2/core.hpp>
// #include <opencv2/highgui.hpp>
// #include <opencv2/imgproc.hpp>
// #include <demo/demo.hpp>
// #include <segmentation/process_result.hpp>
// int test_segmentation(int argc, char *argv[])
// {
//     return xilinx::demo::main_for_jpeg_demo(argc, argv, [] { return xilinx::segmentation::Segmentation8UC3::create(xilinx::segmentation::FPN); }, process_result);
// }

int main(int argc, char* argv[])
{
    cout << "\n++++++++++++++++++++++++++++++++++++" << endl;
    cout << "Please press key to select mode below:" << endl;
    cout << "'0' : test_yolo;"     << endl;
    cout << "'1' : test_envlight;" << endl;
    cout << "'2' : test_lane;"     << endl;
    cout << "'3' : test_obj_all;"  << endl;
    cout << "'4' : test_ldw_image;"<< endl;
    cout << "'5' : test_ldw_video;"<< endl;
    // cout << "'6':test_segmentation;" << endl;
    cout << "++++++++++++++++++++++++++++++++++++\n" << endl;
    int input_mode;
    switch (cin >> input_mode, input_mode)
    {
        case 0: test_yolo(argc, argv);     break;
        case 1: test_envlight(argc, argv); break;
        case 2: test_lane(argc, argv);     break;
        case 3: test_obj_all(argc, argv);  break;
        case 4: test_ldw_image(argc, argv);break;
        case 5: test_ldw_video(argc, argv);break;
        // case 6: test_segmentation(argc, argv);
    }

    return 0;
}