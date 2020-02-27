#include"Calibrater.h"

Calibrater::Calibrater()
{
    m_online_captured_image = cv::Mat_<uchar>::zeros(IMAGE_HEIGHT,IMAGE_WIDTH);
}

Calibrater::~Calibrater()
{

}

bool Calibrater::load_HRYT_configration(const std::string &config_file)
{
    XMLDocument xml_doc;
    std::cout <<"Hua ren yun tong configuration file "<< config_file << " is loading ... "  << std::endl;
    if (xml_doc.LoadFile(config_file.c_str()) != 0)
    {
        std::cout << "error: can't find " << config_file << std::endl;
        return false;
    }

    //Workshop Param
    XMLElement *elem_root = xml_doc.RootElement();
    XMLElement *elem_param = elem_root->FirstChildElement();
    const XMLAttribute *cur_attribute = elem_param->FirstAttribute();
    cur_attribute = cur_attribute->Next();
    m_workshop_parameter.Xc = float(atof(cur_attribute->Value()));

    elem_param = elem_param->NextSiblingElement();
    cur_attribute = elem_param->FirstAttribute();
    cur_attribute = cur_attribute->Next();
    m_workshop_parameter.Dc = float(atof(cur_attribute->Value()));

    elem_param = elem_param->NextSiblingElement();
    cur_attribute = elem_param->FirstAttribute();
    cur_attribute = cur_attribute->Next();
    m_workshop_parameter.Zc = float(atof(cur_attribute->Value()));

    elem_param = elem_param->NextSiblingElement();
    cur_attribute = elem_param->FirstAttribute();
    cur_attribute = cur_attribute->Next();
    m_workshop_parameter.Yb = float(atof(cur_attribute->Value()));

    elem_param = elem_param->NextSiblingElement();
    cur_attribute = elem_param->FirstAttribute();
    cur_attribute = cur_attribute->Next();
    m_workshop_parameter.D_camera = float(atof(cur_attribute->Value()));

    elem_param = elem_param->NextSiblingElement();
    cur_attribute = elem_param->FirstAttribute();
    cur_attribute = cur_attribute->Next();
    m_workshop_parameter.Zb = float(atof(cur_attribute->Value()));

    //Chessboard Param
    elem_root = elem_root->NextSiblingElement();
    elem_param = elem_root->FirstChildElement();
    cur_attribute = elem_param->FirstAttribute();
    cur_attribute = cur_attribute->Next();
    m_chessboard_parameter.Ds = float(atof(cur_attribute->Value()));

    elem_param = elem_param->NextSiblingElement();
    cur_attribute = elem_param->FirstAttribute();
    cur_attribute = cur_attribute->Next();
    m_chessboard_parameter.Db = float(atof(cur_attribute->Value()));

    elem_param = elem_param->NextSiblingElement();
    cur_attribute = elem_param->FirstAttribute();
    cur_attribute = cur_attribute->Next();
    m_chessboard_parameter.D_width = float(atof(cur_attribute->Value()));

    elem_param = elem_param->NextSiblingElement();
    cur_attribute = elem_param->FirstAttribute();
    cur_attribute = cur_attribute->Next();
    m_chessboard_parameter.D_height = float(atof(cur_attribute->Value()));

    elem_param = elem_param->NextSiblingElement();
    cur_attribute = elem_param->FirstAttribute();
    cur_attribute = cur_attribute->Next();
    m_chessboard_parameter.D_TH = float(atof(cur_attribute->Value()));

    elem_param = elem_param->NextSiblingElement();
    cur_attribute = elem_param->FirstAttribute();
    cur_attribute = cur_attribute->Next();
    m_chessboard_parameter.D_TW = float(atof(cur_attribute->Value()));

    elem_param = elem_param->NextSiblingElement();
    cur_attribute = elem_param->FirstAttribute();
    cur_attribute = cur_attribute->Next();
    m_chessboard_parameter.Dg = float(atof(cur_attribute->Value()));

    //File names
    elem_root = elem_root->NextSiblingElement();
    elem_param = elem_root->FirstChildElement();
    int npos = config_file.rfind("/");
    std::string param_dir = config_file.substr(0,npos);

    cur_attribute = elem_param->FirstAttribute();
    cur_attribute = cur_attribute->Next();
    std::string image_name = cur_attribute->Value();

    m_chessboard_image_path = param_dir + "/" + image_name;
    m_chessboard_corners_image_path = param_dir + "/" + "chessboard_corners.jpg";

    elem_param = elem_param->NextSiblingElement();
    cur_attribute = elem_param->FirstAttribute();
    cur_attribute = cur_attribute->Next();
    std::string offline_param_name = cur_attribute->Value();
    m_offline_parameters_saving_path = param_dir + "/" + offline_param_name;

    elem_param = elem_param->NextSiblingElement();
    cur_attribute = elem_param->FirstAttribute();
    cur_attribute = cur_attribute->Next();
    std::string online_param_name = cur_attribute->Value();
    m_online_parameters_saving_path = param_dir + "/" + online_param_name;

    //Internal Param
    elem_root = elem_root->NextSiblingElement();
    elem_param = elem_root->FirstChildElement();
    cur_attribute = elem_param->FirstAttribute();
    cur_attribute = cur_attribute->Next();
    m_intrinsic_parameter.fx = float(atof(cur_attribute->Value()));

    elem_param = elem_param->NextSiblingElement();
    cur_attribute = elem_param->FirstAttribute();
    cur_attribute = cur_attribute->Next();
    m_intrinsic_parameter.fy = float(atof(cur_attribute->Value()));

    elem_param = elem_param->NextSiblingElement();
    cur_attribute = elem_param->FirstAttribute();
    cur_attribute = cur_attribute->Next();
    m_intrinsic_parameter.cx = float(atof(cur_attribute->Value()));

    elem_param = elem_param->NextSiblingElement();
    cur_attribute = elem_param->FirstAttribute();
    cur_attribute = cur_attribute->Next();
    m_intrinsic_parameter.cy = float(atof(cur_attribute->Value()));

    elem_param = elem_param->NextSiblingElement();
    cur_attribute = elem_param->FirstAttribute();
    cur_attribute = cur_attribute->Next();
    m_intrinsic_parameter.k1 = float(atof(cur_attribute->Value()));

    elem_param = elem_param->NextSiblingElement();
    cur_attribute = elem_param->FirstAttribute();
    cur_attribute = cur_attribute->Next();
    m_intrinsic_parameter.k2 = float(atof(cur_attribute->Value()));

    elem_param = elem_param->NextSiblingElement();
    cur_attribute = elem_param->FirstAttribute();
    cur_attribute = cur_attribute->Next();
    m_intrinsic_parameter.k3 = float(atof(cur_attribute->Value()));

    elem_param = elem_param->NextSiblingElement();
    cur_attribute = elem_param->FirstAttribute();
    cur_attribute = cur_attribute->Next();
    m_intrinsic_parameter.p1 = float(atof(cur_attribute->Value()));

    elem_param = elem_param->NextSiblingElement();
    cur_attribute = elem_param->FirstAttribute();
    cur_attribute = cur_attribute->Next();
    m_intrinsic_parameter.p2 = float(atof(cur_attribute->Value()));

    m_intrinsic_parameter.IntrinsicMatrix = (cv::Mat_<float>(3, 3) <<
        m_intrinsic_parameter.fx, 0, m_intrinsic_parameter.cx
        , 0, m_intrinsic_parameter.fy, m_intrinsic_parameter.cy
        , 0, 0, 1.0);

    m_intrinsic_parameter.distCoeffs = (cv::Mat_<float>(1, 5) <<
        m_intrinsic_parameter.k1, m_intrinsic_parameter.k2
        , m_intrinsic_parameter.p1, m_intrinsic_parameter.p2,
        m_intrinsic_parameter.k3);

    std::cout<<"\t workshop_parameter.Xc : "<<m_workshop_parameter.Xc<<std::endl;
    std::cout<<"\t workshop_parameter.Dc : "<<m_workshop_parameter.Dc<<std::endl;
    std::cout<<"\t workshop_parameter.Zc : "<<m_workshop_parameter.Zc<<std::endl;
    std::cout<<"\t workshop_parameter.Yb : "<<m_workshop_parameter.Yb<<std::endl;
    std::cout<<"\t workshop_parameter.D_camera : "<<m_workshop_parameter.D_camera<<std::endl;
    std::cout<<"\t workshop_parameter.Zb : "<<m_workshop_parameter.Zb<<std::endl;

    std::cout<<"\t chessboard_parameter.Ds : "<<m_chessboard_parameter.Ds<<std::endl;
    std::cout<<"\t chessboard_parameter.Db : "<<m_chessboard_parameter.Db<<std::endl;
    std::cout<<"\t chessboard_parameter.D_width : "<<m_chessboard_parameter.D_width<<std::endl;
    std::cout<<"\t chessboard_parameter.D_height : "<<m_chessboard_parameter.D_height<<std::endl;
    std::cout<<"\t chessboard_parameter.D_TH : "<<m_chessboard_parameter.D_TH<<std::endl;
    std::cout<<"\t chessboard_parameter.D_TW : "<<m_chessboard_parameter.D_TW<<std::endl;
    std::cout<<"\t chessboard_parameter.Dg : "<<m_chessboard_parameter.Dg<<std::endl;

    std::cout<<"\t chessboard image path : " << m_chessboard_image_path << std::endl;
    std::cout<<"\t offline parameters file path : " << m_offline_parameters_saving_path << std::endl;
    std::cout<<"\t online parameters file path : " << m_online_parameters_saving_path << std::endl;
    std::cout<<"\t intrinsic_parameter.fx : "<<m_intrinsic_parameter.fx<<std::endl;
    std::cout<<"\t intrinsic_parameter.fy : "<<m_intrinsic_parameter.fy<<std::endl;
    std::cout<<"\t intrinsic_parameter.cx : "<<m_intrinsic_parameter.cx<<std::endl;
    std::cout<<"\t intrinsic_parameter.cy : "<<m_intrinsic_parameter.cy<<std::endl;
    std::cout<<"\t intrinsic_parameter.k1 : "<<m_intrinsic_parameter.k1<<std::endl;
    std::cout<<"\t intrinsic_parameter.k2 : "<<m_intrinsic_parameter.k2<<std::endl;
    std::cout<<"\t intrinsic_parameter.k3 : "<<m_intrinsic_parameter.k3<<std::endl;
    std::cout<<"\t intrinsic_parameter.p1 : "<<m_intrinsic_parameter.p1<<std::endl;
    std::cout<<"\t intrinsic_parameter.p2 : "<<m_intrinsic_parameter.p2<<std::endl;

    std::cout << config_file << " is loaded" << std::endl;

//    cv::Mat source = cv::Mat::ones(720,1280,CV_8UC1) * 128;
//    cv::cvtColor(source,source,CV_GRAY2BGR);
//    cv::Rect mask = cv::Rect(250,100,c_image.cols,c_image.rows);
//    c_image.copyTo(source(mask));
//    cv::imwrite("source.jpg",source);
    m_configure_loaded = true;
    return true;
}

bool Calibrater::load_offline_parameters(const std::string &offline_camera_parameters_file)
{
    XMLDocument xml_doc;
    std::cout << offline_camera_parameters_file << " is loading... "  << std::endl;
    if (xml_doc.LoadFile(offline_camera_parameters_file.c_str()) != 0)
    {
        std::cout << "error: can't find " << offline_camera_parameters_file << std::endl;
        return false;
    }
    else
    {
        std::cout << offline_camera_parameters_file << " is loaded" << std::endl;
    }


    XMLElement *elem_root = xml_doc.RootElement();
    XMLElement *elem_param = elem_root->FirstChildElement();
    const XMLAttribute *cur_attribute = elem_param->FirstAttribute();
    cur_attribute = cur_attribute->Next();
    m_intrinsic_parameter.fx = float(atof(cur_attribute->Value()));

    elem_param = elem_param->NextSiblingElement();
    cur_attribute = elem_param->FirstAttribute();
    cur_attribute = cur_attribute->Next();
    m_intrinsic_parameter.fy = float(atof(cur_attribute->Value()));

    elem_param = elem_param->NextSiblingElement();
    cur_attribute = elem_param->FirstAttribute();
    cur_attribute = cur_attribute->Next();
    m_intrinsic_parameter.cx = float(atof(cur_attribute->Value()));

    elem_param = elem_param->NextSiblingElement();
    cur_attribute = elem_param->FirstAttribute();
    cur_attribute = cur_attribute->Next();
    m_intrinsic_parameter.cy = float(atof(cur_attribute->Value()));

    elem_param = elem_param->NextSiblingElement();
    cur_attribute = elem_param->FirstAttribute();
    cur_attribute = cur_attribute->Next();
    m_intrinsic_parameter.k1 = float(atof(cur_attribute->Value()));

    elem_param = elem_param->NextSiblingElement();
    cur_attribute = elem_param->FirstAttribute();
    cur_attribute = cur_attribute->Next();
    m_intrinsic_parameter.k2 = float(atof(cur_attribute->Value()));

    elem_param = elem_param->NextSiblingElement();
    cur_attribute = elem_param->FirstAttribute();
    cur_attribute = cur_attribute->Next();
    m_intrinsic_parameter.k3 = float(atof(cur_attribute->Value()));

    elem_param = elem_param->NextSiblingElement();
    cur_attribute = elem_param->FirstAttribute();
    cur_attribute = cur_attribute->Next();
    m_intrinsic_parameter.p1 = float(atof(cur_attribute->Value()));

    elem_param = elem_param->NextSiblingElement();
    cur_attribute = elem_param->FirstAttribute();
    cur_attribute = cur_attribute->Next();
    m_intrinsic_parameter.p2 = float(atof(cur_attribute->Value()));

    std::string temp_string;
    elem_root = elem_root->NextSiblingElement();
    elem_param = elem_root->FirstChildElement();
    cur_attribute = elem_param->FirstAttribute();
    cur_attribute = cur_attribute->Next();
    m_offline_extinsic_parameter.camera_pitch_angle = float(atof(cur_attribute->Value()));
    cur_attribute = cur_attribute->Next();
    temp_string = cur_attribute->Value();
    if (temp_string == "down")
    {
        m_offline_extinsic_parameter.camera_pitch_angle = -m_offline_extinsic_parameter.camera_pitch_angle;
    }

    elem_param = elem_param->NextSiblingElement();
    cur_attribute = elem_param->FirstAttribute();
    cur_attribute = cur_attribute->Next();
    m_offline_extinsic_parameter.camera_yaw_angle = float(atof(cur_attribute->Value()));
    cur_attribute = cur_attribute->Next();
    temp_string = cur_attribute->Value();
    if (temp_string == "left")
    {
        m_offline_extinsic_parameter.camera_yaw_angle = -m_offline_extinsic_parameter.camera_yaw_angle;
    }

    elem_param = elem_param->NextSiblingElement();
    cur_attribute = elem_param->FirstAttribute();
    cur_attribute = cur_attribute->Next();
    m_offline_extinsic_parameter.camera_roll_angle = float(atof(cur_attribute->Value()));
    cur_attribute = cur_attribute->Next();
    temp_string = cur_attribute->Value();
    if (temp_string == "counterclockwise")
    {
        m_offline_extinsic_parameter.camera_roll_angle = -m_offline_extinsic_parameter.camera_roll_angle;
    }

    elem_param = elem_param->NextSiblingElement();
    cur_attribute = elem_param->FirstAttribute();
    cur_attribute = cur_attribute->Next();
    m_offline_extinsic_parameter.camera_height = float(atof(cur_attribute->Value()));
    m_offline_extinsic_parameter.camera_height = std::abs(m_offline_extinsic_parameter.camera_height) / 1000.0f;
    m_offline_extinsic_parameter.camera_world_y = -m_offline_extinsic_parameter.camera_height;

    elem_param = elem_param->NextSiblingElement();
    cur_attribute = elem_param->FirstAttribute();
    cur_attribute = cur_attribute->Next();
    m_offline_extinsic_parameter.camera_world_z = float(atof(cur_attribute->Value()));
    m_offline_extinsic_parameter.camera_world_z = -std::abs(m_offline_extinsic_parameter.camera_world_z) / 1000.0f;

    elem_param = elem_param->NextSiblingElement();
    cur_attribute = elem_param->FirstAttribute();
    cur_attribute = cur_attribute->Next();
    m_offline_extinsic_parameter.camera_world_x = float(atof(cur_attribute->Value()));
    m_offline_extinsic_parameter.camera_world_x = -m_offline_extinsic_parameter.camera_world_x / 1000.0f;

    m_intrinsic_parameter.IntrinsicMatrix = (cv::Mat_<float>(3, 3) <<
        m_intrinsic_parameter.fx, 0, m_intrinsic_parameter.cx
        , 0, m_intrinsic_parameter.fy, m_intrinsic_parameter.cy
        , 0, 0, 1.0);
    m_intrinsic_parameter.distCoeffs = (cv::Mat_<float>(1, 5) <<
        m_intrinsic_parameter.k1, m_intrinsic_parameter.k2
        , m_intrinsic_parameter.p1, m_intrinsic_parameter.p2,
        m_intrinsic_parameter.k3);

    const float PI = 3.1415926f;
    float arc_pitch_angle = m_offline_extinsic_parameter.camera_pitch_angle * PI/180.0f;
    float arc_yaw_angle = m_offline_extinsic_parameter.camera_yaw_angle * PI / 180.0f;
    float arc_roll_angle = m_offline_extinsic_parameter.camera_roll_angle * PI / 180.0f;

    cv::Mat pitchMatrix = (cv::Mat_<float>(3, 3) << 1, 0, 0
        , 0, std::cos(arc_pitch_angle), std::sin(arc_pitch_angle)
        , 0, -std::sin(arc_pitch_angle), std::cos(arc_pitch_angle));
    cv::Mat yaw_Matrix = (cv::Mat_<float>(3, 3) << std::cos(arc_yaw_angle), 0, -std::sin(arc_yaw_angle)
        , 0, 1, 0
        , std::sin(arc_yaw_angle), 0, std::cos(arc_yaw_angle));
    cv::Mat roll_Matrix = (cv::Mat_<float>(3, 3) << std::cos(arc_roll_angle), std::sin(arc_roll_angle), 0
        , -std::sin(arc_roll_angle), std::cos(arc_roll_angle), 0
        , 0, 0, 1);

    m_offline_extinsic_parameter.rotateMatrix = roll_Matrix*yaw_Matrix*pitchMatrix;
    m_offline_extinsic_parameter.rotateMatrix_inv = m_offline_extinsic_parameter.rotateMatrix.inv();
    m_offline_extinsic_parameter.cameraLocationInWorld = (cv::Mat_<float>(3, 1) <<
        m_offline_extinsic_parameter.camera_world_x, -m_offline_extinsic_parameter.camera_height, m_offline_extinsic_parameter.camera_world_z);
    m_offline_extinsic_parameter.ground_plane_world_y = 0;

    cv::Point2f offline_vanish_point;
    GroundCoordinateToPixelPointOffline(offline_vanish_point,cv::Point2f(0,10000));
    m_vanish_point_of_lanes=offline_vanish_point;
    return true;
}


bool Calibrater::OfflineCalibrationOfHRYT(const std::string &config_file)
{
    bool open_config = this->load_HRYT_configration(config_file);
    if(!open_config) return 0;

    if(!m_configure_loaded)
    {
        std::cout<<"error : configure file is not loaded!"<<std::endl;
    }
    std::cout<<std::endl<<"Start offline calibration by chessboard image. "<<std::endl;
    m_camera_in_HRYT_System = cv::Point3f(0 - m_workshop_parameter.Xc
                                       ,m_workshop_parameter.Dc
                                       ,m_workshop_parameter.Zc) / 1000.f;
    std::cout<<"Chessboard image : "<<m_chessboard_image_path<<" is loading ..."<<std::endl;
    std::string suffix_name = m_chessboard_image_path.substr(m_chessboard_image_path.find_last_of(".") + 1,-1);
    cv::Mat chess_image;
    if(suffix_name == "jpg"
            || suffix_name == "png")
    {
        chess_image = cv::imread(m_chessboard_image_path);
    }
    else if(suffix_name == "420"){
        chess_image = this->LoadYUV420FileAsBGR(m_chessboard_image_path,IMAGE_HEIGHT,IMAGE_WIDTH);
    }
    if(chess_image.empty())
    {
        std::cout<<"error : failed to load "<<m_chessboard_image_path<<std::endl;
        return false;
    }
    else {
        std::cout<<m_chessboard_image_path<<" is loaded"<<std::endl;
    }
    int max_grid_size = m_chessboard_parameter.Ds / m_workshop_parameter.D_camera * m_intrinsic_parameter.fx;
    std::cout<<"chessboard grid size :  "<<max_grid_size<< " pixels"<< std::endl;
    std::cout<<"real corner location in Hua Ren Yun Tong coordinate camera system from left to right: " << std::endl;
    m_real_corners_in_HRYT_System.resize(4);
    m_real_corners_in_HRYT_System[0] = cv::Point3f(m_workshop_parameter.D_camera - m_workshop_parameter.Xc
                                          ,m_chessboard_parameter.Db / 2.f + m_chessboard_parameter.D_width / 6.f
                                          ,m_workshop_parameter.Zb) / 1000.f;
    m_real_corners_in_HRYT_System[1] = cv::Point3f(m_workshop_parameter.D_camera - m_workshop_parameter.Xc
                                          ,m_chessboard_parameter.Db / 2.f - m_chessboard_parameter.D_width / 6.f
                                          ,m_workshop_parameter.Zb) / 1000.f;
    m_real_corners_in_HRYT_System[2] = cv::Point3f(m_workshop_parameter.D_camera - m_workshop_parameter.Xc
                                          ,-m_chessboard_parameter.Db / 2.f + m_chessboard_parameter.D_width / 6.f
                                          ,m_workshop_parameter.Zb) / 1000.f;
    m_real_corners_in_HRYT_System[3] = cv::Point3f(m_workshop_parameter.D_camera - m_workshop_parameter.Xc
                                          ,-m_chessboard_parameter.Db / 2.f - m_chessboard_parameter.D_width / 6.f
                                          ,m_workshop_parameter.Zb) / 1000.f;
    std::cout<<"\t corners 1 : "<<m_real_corners_in_HRYT_System[0] - m_camera_in_HRYT_System <<"m"<<std::endl;
    std::cout<<"\t corners 2 : "<<m_real_corners_in_HRYT_System[1] - m_camera_in_HRYT_System <<"m"<<std::endl;
    std::cout<<"\t corners 3 : "<<m_real_corners_in_HRYT_System[2] - m_camera_in_HRYT_System <<"m"<<std::endl;
    std::cout<<"\t corners 4 : "<<m_real_corners_in_HRYT_System[3] - m_camera_in_HRYT_System <<"m"<<std::endl;

    cv::Mat gray_img;
    cv::cvtColor(chess_image,gray_img,CV_BGR2GRAY);

    max_grid_size = std::max(20 , max_grid_size);
    bool find_4_corners = false;
    for(int g_s = max_grid_size * 1.5;g_s > max_grid_size ;g_s--)
    {
        find_4_corners = ChessCornerDetection(m_observed_corners_in_image,gray_img,max_grid_size,30);
        if(find_4_corners) break;
    }
    if(!find_4_corners)
    {
        std::cout<<"error : failed to locate corner points on chessboard image. Please recheck the calibration enviroment!"<<std::endl;
        return false;
    }

    std::cout<<"observed corner location in image coordinate system from left to right: " << std::endl;
    std::cout<<"\t corners 1 : "<<m_observed_corners_in_image[0]<<std::endl;
    std::cout<<"\t corners 2 : "<<m_observed_corners_in_image[1]<<std::endl;
    std::cout<<"\t corners 3 : "<<m_observed_corners_in_image[2]<<std::endl;
    std::cout<<"\t corners 4 : "<<m_observed_corners_in_image[3]<<std::endl;

    std::vector<cv::Point3f> real_corners_in_camera(4);
    ExtinsicParameter::HRYTCoordinate2WorldCoordinate(real_corners_in_camera[0],m_real_corners_in_HRYT_System[0] - m_camera_in_HRYT_System);
    ExtinsicParameter::HRYTCoordinate2WorldCoordinate(real_corners_in_camera[1],m_real_corners_in_HRYT_System[1] - m_camera_in_HRYT_System);
    ExtinsicParameter::HRYTCoordinate2WorldCoordinate(real_corners_in_camera[2],m_real_corners_in_HRYT_System[2] - m_camera_in_HRYT_System);
    ExtinsicParameter::HRYTCoordinate2WorldCoordinate(real_corners_in_camera[3],m_real_corners_in_HRYT_System[3] - m_camera_in_HRYT_System);

    std::vector<cv::Point3f> real_corners_in_uv(4);
    std::vector<cv::Point2f> observed_corners_in_uv(4);
    real_corners_in_uv[0] = real_corners_in_camera[0] / real_corners_in_camera[0].z;
    real_corners_in_uv[1] = real_corners_in_camera[1] / real_corners_in_camera[1].z;
    real_corners_in_uv[2] = real_corners_in_camera[2] / real_corners_in_camera[2].z;
    real_corners_in_uv[3] = real_corners_in_camera[3] / real_corners_in_camera[3].z;
    m_intrinsic_parameter.PixelPoint2UVPlanePoint(observed_corners_in_uv[0],m_observed_corners_in_image[0]);
    m_intrinsic_parameter.PixelPoint2UVPlanePoint(observed_corners_in_uv[1],m_observed_corners_in_image[1]);
    m_intrinsic_parameter.PixelPoint2UVPlanePoint(observed_corners_in_uv[2],m_observed_corners_in_image[2]);
    m_intrinsic_parameter.PixelPoint2UVPlanePoint(observed_corners_in_uv[3],m_observed_corners_in_image[3]);

    cv::Mat_<float> mtx_real_corners_in_uv = cv::Mat_<float>::zeros(2,4);
    cv::Mat_<float> mtx_observed_corners_in_uv = cv::Mat_<float>::zeros(2,4);
    for(int i=0;i<4;i++)
    {
        mtx_real_corners_in_uv(0,i) = real_corners_in_uv[i].x;
        mtx_real_corners_in_uv(1,i) = real_corners_in_uv[i].y;
        mtx_observed_corners_in_uv(0,i) = observed_corners_in_uv[i].x;
        mtx_observed_corners_in_uv(1,i) = observed_corners_in_uv[i].y;
    }

    cv::Mat mtx_real_corners_in_camera = cv::Mat_<cv::Vec3f>(4,1);
    cv::Mat mtx_corners_in_image = cv::Mat_<cv::Vec2f>(4,1);
    for(int i=0;i<4;i++)
    {
        mtx_real_corners_in_camera.at<cv::Vec3f>(i)[0] = real_corners_in_camera[i].x;
        mtx_real_corners_in_camera.at<cv::Vec3f>(i)[1] = real_corners_in_camera[i].y;
        mtx_real_corners_in_camera.at<cv::Vec3f>(i)[2] = real_corners_in_camera[i].z;
        mtx_corners_in_image.at<cv::Vec2f>(i)[0] = m_observed_corners_in_image[i].x;
        mtx_corners_in_image.at<cv::Vec2f>(i)[1] = m_observed_corners_in_image[i].y;
    }

    cv::Point3f real_corners_center_in_camera = (real_corners_in_camera[0] + real_corners_in_camera[1]
                                            + real_corners_in_camera[2] + real_corners_in_camera[3]) / 4.f;
    cv::Point2f real_corners_center_in_uv = cv::Point2f(real_corners_center_in_camera.x / real_corners_center_in_camera.z
                                                        ,real_corners_center_in_camera.y / real_corners_center_in_camera.z);

    //solve roll angle
    cv::Mat Y = cv::Mat::zeros(4, 1, CV_32FC1);
    cv::Mat X = cv::Mat::zeros(4, 2, CV_32FC1);
    for(int i=0;i<4;i++)
    {
        Y.at<float>(i,0) = observed_corners_in_uv[i].y;
        X.at<float>(i,0) = observed_corners_in_uv[i].x;
        X.at<float>(i,1) = 1;
    }

    cv::Mat A;
    cv::solve(X,Y,A,cv::DECOMP_SVD);
    float roll_radian = 0 - atan(A.at<float>(0));
    cv::Mat roll_Matrix = (cv::Mat_<float>(3, 3) << std::cos(roll_radian), std::sin(roll_radian), 0
        , -std::sin(roll_radian), std::cos(roll_radian), 0
        , 0, 0, 1);

    cv::undistortPoints(mtx_corners_in_image,mtx_corners_in_image,m_intrinsic_parameter.IntrinsicMatrix,m_intrinsic_parameter.distCoeffs);

    cv::Mat mtx_corners_in_image_3d = cv::Mat_<float>::zeros(3,4);
    cv::Mat mtx_corners_center_from_image_3d= cv::Mat_<float>::zeros(3,1);
    for(int i=0;i<4;i++)
    {
        mtx_corners_in_image_3d.at<float>(0,i) = mtx_corners_in_image.at<cv::Vec2f>(i)[0];
        mtx_corners_in_image_3d.at<float>(1,i) = mtx_corners_in_image.at<cv::Vec2f>(i)[1];
        mtx_corners_in_image_3d.at<float>(2,i) = 1;

        mtx_corners_center_from_image_3d.at<float>(0) += mtx_corners_in_image_3d.at<float>(0,i);
        mtx_corners_center_from_image_3d.at<float>(1) += mtx_corners_in_image_3d.at<float>(1,i);
        mtx_corners_center_from_image_3d.at<float>(2) += mtx_corners_in_image_3d.at<float>(2,i);
    }
    mtx_corners_center_from_image_3d = mtx_corners_center_from_image_3d /4.f;
    cv::Mat mtx_corners_in_image_3d_src = mtx_corners_in_image_3d.clone();
    mtx_corners_in_image_3d= roll_Matrix.inv() * mtx_corners_in_image_3d;

    //solve pitch angle
    float center_y_from_image = (mtx_corners_in_image_3d.at<float>(1,0) + mtx_corners_in_image_3d.at<float>(1,1)
                      + mtx_corners_in_image_3d.at<float>(1,2) + mtx_corners_in_image_3d.at<float>(1,3)) / 4.f;

    float pitch_radian = (atan(center_y_from_image - real_corners_center_in_uv.y));
    cv::Mat mtx_corners_in_camera_3d = cv::Mat_<float>::zeros(3,4);
    for(int i=0;i<4;i++)
    {
        mtx_corners_in_camera_3d.at<float>(0,i) = real_corners_in_camera[i].x / real_corners_in_camera[i].z;
        mtx_corners_in_camera_3d.at<float>(1,i) = real_corners_in_camera[i].y / real_corners_in_camera[i].z;
        mtx_corners_in_camera_3d.at<float>(2,i) = 1.f;
    }
    cv::Mat mtx_corners_in_camera_3d_src = mtx_corners_in_camera_3d.clone();
    cv::Mat pitchMatrix = (cv::Mat_<float>(3, 3) << 1, 0, 0
        , 0, std::cos(pitch_radian), std::sin(pitch_radian)
        , 0, -std::sin(pitch_radian), std::cos(pitch_radian));

    mtx_corners_in_camera_3d = pitchMatrix * mtx_corners_in_camera_3d;
    for(int i=0;i<4;i++)
    {
        mtx_corners_in_camera_3d.at<float>(0,i) /= mtx_corners_in_camera_3d.at<float>(2,i);
        mtx_corners_in_camera_3d.at<float>(1,i) /= mtx_corners_in_camera_3d.at<float>(2,i);
        mtx_corners_in_camera_3d.at<float>(2,i) /= mtx_corners_in_camera_3d.at<float>(2,i);
    }

    //solve yaw angle
    float center_x_from_image = (mtx_corners_in_image_3d.at<float>(0,0) + mtx_corners_in_image_3d.at<float>(0,1)
                      + mtx_corners_in_image_3d.at<float>(0,2) + mtx_corners_in_image_3d.at<float>(0,3)) / 4.f;

    float center_x_from_camera = (mtx_corners_in_camera_3d.at<float>(0,0) + mtx_corners_in_camera_3d.at<float>(0,1)
                      + mtx_corners_in_camera_3d.at<float>(0,2) + mtx_corners_in_camera_3d.at<float>(0,3)) / 4.f;

    float yaw_radian = -(atan((center_x_from_image - center_x_from_camera)));

    cv::Mat yaw_Matrix = (cv::Mat_<float>(3, 3) << std::cos(yaw_radian), 0, -std::sin(yaw_radian)
        , 0, 1, 0
        , std::sin(yaw_radian), 0, std::cos(yaw_radian));

     mtx_corners_in_image_3d= yaw_Matrix.inv() * mtx_corners_in_image_3d;

//    cv::Point2f observed_corners_center_in_uv = (observed_corners_in_uv[0] + observed_corners_in_uv[1]
//                                            + observed_corners_in_uv[2] + observed_corners_in_uv[3]) /4.f;

    m_offline_extinsic_parameter.camera_roll_angle = roll_radian * 180 / 3.1415926;
    m_offline_extinsic_parameter.camera_yaw_angle = yaw_radian * 180 / 3.1415926;
    m_offline_extinsic_parameter.camera_pitch_angle = pitch_radian * 180 / 3.1415926;
    m_offline_extinsic_parameter.rotateMatrix = roll_Matrix*yaw_Matrix*pitchMatrix;

////    mtx_corners_in_camera_3d_src = m_offline_extinsic_parameter.rotateMatrix * mtx_corners_in_camera_3d_src;
    mtx_corners_in_image_3d_src = m_offline_extinsic_parameter.rotateMatrix.inv() * mtx_corners_in_image_3d_src;
    mtx_corners_center_from_image_3d = m_offline_extinsic_parameter.rotateMatrix.inv() * mtx_corners_center_from_image_3d;

    std::cout<<"estimate external parameters : " << std::endl;
    std::cout<<"\t pitch angle (up): "<<m_offline_extinsic_parameter.camera_pitch_angle<<" degree"<<std::endl;
    std::cout<<"\t yaw angle (right) : "<<m_offline_extinsic_parameter.camera_yaw_angle<<" degree"<<std::endl;
    std::cout<<"\t roll angle (clockwise): "<<m_offline_extinsic_parameter.camera_roll_angle<<" degree"<<std::endl<<std::endl;

    //write out camera parameters
    cv::Mat draw_img = gray_img.clone();
    cv::cvtColor(draw_img,draw_img,CV_GRAY2BGR);
    #ifdef AISDK_INTERFACE
    #else
    cv::drawChessboardCorners(draw_img,cv::Size(1,4), cv::Mat(m_observed_corners_in_image), true);
    #endif

    cv::Mat draw_undistort_img;
    cv::undistort(draw_img, draw_undistort_img, m_intrinsic_parameter.IntrinsicMatrix, m_intrinsic_parameter.distCoeffs);

    cv::Mat draw_rotate_img;
    m_offline_extinsic_parameter.camera_world_x = -m_workshop_parameter.Dc / 1000;
    m_offline_extinsic_parameter.camera_world_y = -m_workshop_parameter.Zc / 1000;
    m_offline_extinsic_parameter.camera_world_z = m_workshop_parameter.Xc / 1000;

    cv::Mat A1 = (cv::Mat_<float>(4,3) <<
              1 / m_intrinsic_parameter.fx, 0, -m_intrinsic_parameter.cx / m_intrinsic_parameter.fx,
              0, 1 /m_intrinsic_parameter.fy, -m_intrinsic_parameter.cy / m_intrinsic_parameter.fy,
                  0,0,1,
              0, 0,    1);
    // Rotation matrices around the X, Y, and Z axis
    cv::Mat RX = (cv::Mat_<float>(4, 4) <<
              1,          0,           0, 0,
              0, cos(pitch_radian), sin(pitch_radian), 0,
              0, -sin(pitch_radian),  cos(pitch_radian), 0,
              0,          0,           0, 1);
    cv::Mat RY = (cv::Mat_<float>(4, 4) <<
              cos(yaw_radian), 0, -sin(yaw_radian), 0,
              0, 1,          0, 0,
              sin(yaw_radian), 0,  cos(yaw_radian), 0,
              0, 0,          0, 1);
    cv::Mat RZ = (cv::Mat_<float>(4, 4) <<
              cos(roll_radian), sin(roll_radian), 0, 0,
              -sin(roll_radian),  cos(roll_radian), 0, 0,
              0,          0,           1, 0,
              0,          0,           0, 1);
    // Composed rotation matrix with (RX, RY, RZ)
    cv::Mat R = RX.inv() * RY.inv() * RZ.inv();
    // Translation matrix
    cv::Mat T = (cv::Mat_<float>(4, 4) <<
             1, 0, 0, -m_workshop_parameter.Dc / m_workshop_parameter.D_camera,
             0, 1, 0, (m_workshop_parameter.Zb - m_workshop_parameter.Zc) / m_workshop_parameter.D_camera,
             0, 0, 1, 0,
             0, 0, 0, 1);
    // 3D -> 2D matrix
    cv::Mat A2 = (cv::Mat_<float>(3,4) <<
              m_intrinsic_parameter.fx, 0, IMAGE_WIDTH/2, 0,
              0, m_intrinsic_parameter.fy, IMAGE_HEIGHT/2, 0,
              0, 0,   1, 0);

    cv::Mat trans_mtx = A2 * T * R * A1;
    cv::warpPerspective(draw_undistort_img, draw_rotate_img, trans_mtx, draw_undistort_img.size(), cv::INTER_CUBIC);

//    cv::Point2f image_center = (observed_corners_in_uv[0] +observed_corners_in_uv[1]
//            + observed_corners_in_uv[2] + observed_corners_in_uv[3])/4.f;

//    cv::Mat test_point = (cv::Mat_<float>(4, 1) << image_center.x,image_center.y,1,1);
//    cv::Mat rlt_point = T * R * test_point;
//    rlt_point.at<float>(0) /= rlt_point.at<float>(2);
//    rlt_point.at<float>(1) /= rlt_point.at<float>(2);
//    rlt_point.at<float>(2) /= rlt_point.at<float>(2);

    cv::line(draw_rotate_img,cv::Point(0,0),cv::Point(IMAGE_WIDTH-1,IMAGE_HEIGHT-1),cv::Scalar(0,255,0),1,cv::LINE_AA);
    cv::line(draw_rotate_img,cv::Point(IMAGE_WIDTH-1,0),cv::Point(0,IMAGE_HEIGHT-1),cv::Scalar(0,255,0),1,cv::LINE_AA);
    cv::imwrite(m_chessboard_corners_image_path,draw_rotate_img);

    this->SaveOfflineCameraParameters();
    std::cout<<"camera offline calibration is done!"<<std::endl;
    return true;
}

bool Calibrater::OnlineCalibration(const uchar *ptr, int image_width, int image_height)
{
    memcpy(m_online_captured_image.data,ptr,sizeof(uchar) * IMAGE_HEIGHT * IMAGE_WIDTH);
    cv::Point2f offline_vanish_point;
    GroundCoordinateToPixelPointOffline(offline_vanish_point,cv::Point2f(0,10000));

    cv::Point base_point = offline_vanish_point;

    cv::Rect lane_area;
    lane_area = cv::Rect(0,offline_vanish_point.y + 0.2f * (image_height - base_point.y),image_width , 0.8f * (image_height - base_point.y));

    cv::Mat lane_image =m_online_captured_image(lane_area).clone();
    cv::Canny(lane_image,lane_image,30,80);

    std::vector<cv::Vec4f> lane_lines;
    cv::HoughLinesP(lane_image,lane_lines,1,3.1415926 / 180.f,30,40,10);

    std::vector<cv::Vec4f> lane_lines_filtered;
    for(int i=0;i<lane_lines.size();i++)
    {
        cv::Point2f pt1 = lane_area.tl();
        pt1 = cv::Point2f(lane_lines[i][0],lane_lines[i][1]) + pt1;
        cv::Point2f pt2 = lane_area.tl();
        pt2 = cv::Point2f(lane_lines[i][2],lane_lines[i][3]) + pt2;
        this->PixelPointToGroundCoordinateOffline(pt1,pt1);
        this->PixelPointToGroundCoordinateOffline(pt2,pt2);

        cv::Point2f v_pt = pt2 - pt1;
        if(abs(v_pt.x) < abs(v_pt.y) * 0.2)
        {
            lane_lines_filtered.push_back(lane_lines[i]
                 +cv::Vec4f(lane_area.tl().x,lane_area.tl().y,lane_area.tl().x,lane_area.tl().y));
        }
    }

    cv::Point2f vanish_point_of_lanes;
    bool vanish_point_founded = VanishPointByLaneLines(vanish_point_of_lanes,lane_lines_filtered,offline_vanish_point);

    cv::Mat show_img;
    cv::cvtColor(m_online_captured_image,show_img,CV_GRAY2BGR);
    cv::cvtColor(lane_image,lane_image,CV_GRAY2BGR);

    for(int i=0;i<lane_lines_filtered.size();i++)
    {
        cv::line(show_img,cv::Point(lane_lines_filtered[i][0],lane_lines_filtered[i][1]),cv::Point(lane_lines_filtered[i][2],lane_lines_filtered[i][3]),cv::Scalar(0,0,255));
    }
    cv::circle(show_img,offline_vanish_point,4,cv::Scalar(0,0,255),-1);

    if(vanish_point_founded)
    {
        m_vanish_point_of_lanes = (m_vanish_point_of_lanes * m_vanish_pt_count + vanish_point_of_lanes)/(m_vanish_pt_count + 1.f);
    }
    cv::circle(show_img,m_vanish_point_of_lanes,4,cv::Scalar(0,255,0),-1);
//    cv::imshow("lane_image",lane_image);
    cv::imshow("Online calibration",show_img);


//    memcpy(m_online_captured_image.data,ptr,sizeof(uchar) * IMAGE_WIDTH * IMAGE_WIDTH);
//    cv::Mat source = cv::Mat_<uchar>::zeros(IMAGE_HEIGHT,IMAGE_WIDTH);
//    memcpy(m_online_captured_image.data,ptr,sizeof(uchar) * IMAGE_HEIGHT * IMAGE_WIDTH);

//    cv::Point2f offline_vanish_point;
//    GroundCoordinateToPixelPointOffline(offline_vanish_point,cv::Point2f(0,10000));

//    cv::Rect pole_area,lane_area;
//    lane_area = cv::Rect(0,offline_vanish_point.y + 0.2f * (image_height - offline_vanish_point.y),image_width , 0.8f * (image_height - offline_vanish_point.y));

//    cv::Mat lane_image =source(lane_area).clone();

//    cv::Canny(lane_image,lane_image,30,80);

//    std::vector<cv::Vec4f> lane_lines;
//    cv::HoughLinesP(lane_image,lane_lines,1,3.1415926 / 180.f,30,40,10);

//    std::vector<cv::Vec4f> lane_lines_filtered;
//    for(int i=0;i<lane_lines.size();i++)
//    {
//        cv::Point2f pt1 = lane_area.tl();
//        pt1 = cv::Point2f(lane_lines[i][0],lane_lines[i][1]) + pt1;
//        cv::Point2f pt2 = lane_area.tl();
//        pt2 = cv::Point2f(lane_lines[i][2],lane_lines[i][3]) + pt2;
//        this->PixelPointToGroundCoordinateOffline(pt1,pt1);
//        this->PixelPointToGroundCoordinateOffline(pt2,pt2);

//        cv::Point2f v_pt = pt2 - pt1;
//        if(abs(v_pt.x) < abs(v_pt.y) * 0.2)
//        {
//            lane_lines_filtered.push_back(lane_lines[i]
//                 +cv::Vec4f(lane_area.tl().x,lane_area.tl().y,lane_area.tl().x,lane_area.tl().y));
//        }
//    }

//    cv::Point2f vanish_point_of_lanes;
//    bool vanish_point_founded = VanishPointByLaneLines(vanish_point_of_lanes,lane_lines_filtered,offline_vanish_point);

//    cv::cvtColor(source,source,CV_GRAY2BGR);
//    cv::cvtColor(lane_image,lane_image,CV_GRAY2BGR);

//    for(int i=0;i<lane_lines_filtered.size();i++)
//    {
//        cv::line(source,cv::Point(lane_lines_filtered[i][0],lane_lines_filtered[i][1]),cv::Point(lane_lines_filtered[i][2],lane_lines_filtered[i][3]),cv::Scalar(0,0,255));
//    }
//    cv::circle(source,offline_vanish_point,4,cv::Scalar(0,0,255),-1);

//    if(vanish_point_founded)
//    {
//        m_vanish_point_of_lanes = (m_vanish_point_of_lanes * m_vanish_pt_count + vanish_point_of_lanes)/(m_vanish_pt_count + 1.f);
//    }
//    cv::circle(source,m_vanish_point_of_lanes,4,cv::Scalar(0,255,0),-1);
//    cv::imshow("lane_image",lane_image);
//    cv::imshow("Online calibration",source);
    return true;
}

bool Calibrater::VanishPointByLaneLines(cv::Point2f &vanish_point,const std::vector<cv::Vec4f> &lane_lines,const cv::Point2f offline_vanish_point)
{
//    cv::Mat test = cv::Mat::zeros(720,1280,CV_8UC3);

    std::vector<cv::Point2f> cross_points;
    for(int i=0;i<lane_lines.size();i++)
    {
        for(int j=i + 1;j<lane_lines.size();j++)
        {
            cv::Point2f c_pt;
            CrossPointOfTwoLanes(c_pt
                                 ,cv::Point2f(lane_lines[i][0],lane_lines[i][1])
                                ,cv::Point2f(lane_lines[i][2],lane_lines[i][3])
                                ,cv::Point2f(lane_lines[j][0],lane_lines[j][1])
                                ,cv::Point2f(lane_lines[j][2],lane_lines[j][3]));
            cross_points.push_back(c_pt);
//            cv::circle(test,c_pt,1,cv::Scalar(255,0,0));
        }
    }
//    cv::imshow("test",test);
    float radius = 40;
    cv::Point2f cur_centoid = offline_vanish_point;
    for(int iter=0;iter<10;iter++)
    {
        cv::Point2f new_centroid=cv::Point2f(0,0);
        float count_in_radius=0;
        for(int j=0;j<cross_points.size();j++)
        {
            if(cv::norm(cross_points[j] - cur_centoid) < radius)
            {
                new_centroid += cross_points[j];
                count_in_radius++;
            }
        }
        if(count_in_radius!=0)
        {
            cur_centoid = new_centroid / count_in_radius;
        }
    }

    if(cv::norm(cur_centoid - offline_vanish_point) > 40)
    {
        vanish_point = offline_vanish_point;
        return false;
    }
    else {
        vanish_point = cur_centoid;
        return true;
    }
}

void Calibrater::CrossPointOfTwoLanes(cv::Point2f &cross_point, cv::Point2f lineA1, cv::Point2f lineA2, cv::Point2f lineB1, cv::Point2f lineB2)
{
    float X1 = lineA1.x - lineA2.x;
    float Y1 = lineA1.y - lineA2.y;
    float X2 = lineB1.x - lineB2.x;
    float Y2 = lineB1.y - lineB2.y;
    if (X1*Y2 == X2*Y1)
    {
        cross_point = cv::Point2f((lineA2.x + lineB1.x) / 2, (lineA2.y + lineB1.y) / 2);
        return;
    }

    float A = X1*lineA1.y - Y1*lineA1.x;
    float B = X2*lineB1.y - Y2*lineB1.x;

    cross_point.y = (A*Y2 - B*Y1) / (X1*Y2 - X2*Y1);
    cross_point.x = (B*X1-A*X2) / (Y1*X2 - Y2*X1);
}



bool Calibrater::SaveOfflineCameraParameters()
{
    const char* xmlPath = m_offline_parameters_saving_path.c_str();

    const char* declaration = "<?xml version=\"1.0\" encoding=\"UTF-8\"?>";
    XMLDocument doc;
    doc.Parse(declaration);

    XMLElement* internal_items = doc.NewElement("items");
    internal_items->SetAttribute("Name","Internal Param");
    doc.InsertEndChild(internal_items);

    XMLElement* internal_item = doc.NewElement("item");
    internal_item->SetAttribute("Param","CamFocal_x");
    internal_item->SetAttribute("value",m_intrinsic_parameter.fx);
    internal_item->InsertFirstChild(doc.NewComment("intrinsic_parameter.fx"));
    internal_items->InsertEndChild(internal_item);

    internal_item = doc.NewElement("item");
    internal_item->SetAttribute("Param","CamFocal_y");
    internal_item->SetAttribute("value",m_intrinsic_parameter.fy);
    internal_item->InsertFirstChild(doc.NewComment("intrinsic_parameter.fy"));
    internal_items->InsertEndChild(internal_item);

    internal_item = doc.NewElement("item");
    internal_item->SetAttribute("Param","PrincPointC_real_x");
    internal_item->SetAttribute("value",m_intrinsic_parameter.cx);
    internal_item->InsertFirstChild(doc.NewComment("intrinsic_parameter.cx"));
    internal_items->InsertEndChild(internal_item);

    internal_item = doc.NewElement("item");
    internal_item->SetAttribute("Param","PrincPointC_real_y");
    internal_item->SetAttribute("value",m_intrinsic_parameter.cy);
    internal_item->InsertFirstChild(doc.NewComment("intrinsic_parameter.cy"));
    internal_items->InsertEndChild(internal_item);

    internal_item = doc.NewElement("item");
    internal_item->SetAttribute("Param","DisCoef_k1");
    internal_item->SetAttribute("value",m_intrinsic_parameter.k1);
    internal_item->InsertFirstChild(doc.NewComment("intrinsic_parameter.k1"));
    internal_items->InsertEndChild(internal_item);

    internal_item = doc.NewElement("item");
    internal_item->SetAttribute("Param","DisCoef_k2");
    internal_item->SetAttribute("value",m_intrinsic_parameter.k2);
    internal_item->InsertFirstChild(doc.NewComment("intrinsic_parameter.k2"));
    internal_items->InsertEndChild(internal_item);

    internal_item = doc.NewElement("item");
    internal_item->SetAttribute("Param","DisCoef_k3");
    internal_item->SetAttribute("value",m_intrinsic_parameter.k3);
    internal_item->InsertFirstChild(doc.NewComment("intrinsic_parameter.k3"));
    internal_items->InsertEndChild(internal_item);

    internal_item = doc.NewElement("item");
    internal_item->SetAttribute("Param","DisCoef_p1");
    internal_item->SetAttribute("value",m_intrinsic_parameter.p1);
    internal_item->InsertFirstChild(doc.NewComment("intrinsic_parameter.p1"));
    internal_items->InsertEndChild(internal_item);

    internal_item = doc.NewElement("item");
    internal_item->SetAttribute("Param","DisCoef_p1");
    internal_item->SetAttribute("value",m_intrinsic_parameter.p2);
    internal_item->InsertFirstChild(doc.NewComment("intrinsic_parameter.p2"));
    internal_items->InsertEndChild(internal_item);

    XMLElement* external_items = doc.NewElement("items");
    external_items->SetAttribute("Name","External Param");
    external_items->SetAttribute("remark","From the driver perspective");
    external_items->InsertFirstChild(doc.NewComment("Eula order: world -> pitch -> yaw -> roll -> camera"));
    doc.InsertEndChild(external_items);

    XMLElement* external_item = doc.NewElement("item");
    external_item->SetAttribute("Param","pitch");
    external_item->SetAttribute("value",m_offline_extinsic_parameter.camera_pitch_angle);
    external_item->InsertFirstChild(doc.NewComment("extinsic_parameter.camera_pitch_angle"));
    external_item->SetAttribute("remark","up");
    external_items->InsertEndChild(external_item);

    external_item = doc.NewElement("item");
    external_item->SetAttribute("Param","yaw");
    external_item->SetAttribute("value",m_offline_extinsic_parameter.camera_yaw_angle);
    external_item->InsertFirstChild(doc.NewComment("extinsic_parameter.camera_yaw_angle"));
    external_item->SetAttribute("remark","right");
    external_items->InsertEndChild(external_item);

    external_item = doc.NewElement("item");
    external_item->SetAttribute("Param","roll");
    external_item->SetAttribute("value",m_offline_extinsic_parameter.camera_roll_angle);
    external_item->InsertFirstChild(doc.NewComment("extinsic_parameter.camera_roll_angle"));
    external_item->SetAttribute("remark","clockwise");
    external_items->InsertEndChild(external_item);

    external_item = doc.NewElement("item");
    external_item->SetAttribute("Param","CamHeight");
    external_item->SetAttribute("value",m_workshop_parameter.Zc);
    external_item->InsertFirstChild(doc.NewComment("相机高度"));
    external_item->SetAttribute("unit","mm");
    external_item->SetAttribute("remark","");
    external_items->InsertEndChild(external_item);

    external_item = doc.NewElement("item");
    external_item->SetAttribute("Param","Cam2VHDis");
    external_item->SetAttribute("value",m_workshop_parameter.Xc);
    external_item->InsertFirstChild(doc.NewComment("相机距离车头的垂直距离"));
    external_item->SetAttribute("unit","mm");
    external_item->SetAttribute("remark","");
    external_items->InsertEndChild(external_item);

    external_item = doc.NewElement("item");
    external_item->SetAttribute("Param","Cam2VMDis");
    external_item->SetAttribute("value",m_workshop_parameter.Dc);
    external_item->InsertFirstChild(doc.NewComment("相机与车辆中心线的偏离"));
    external_item->SetAttribute("unit","mm");
    external_item->SetAttribute("remark","+ : left  - : right");
    external_items->InsertEndChild(external_item);

    doc.SaveFile(xmlPath);
    return true;
}

cv::Mat Calibrater::LoadYUV420FileAsBGR(const std::string &file_path,int image_height,int image_width)
{
    cv::Mat bgr_img;
    FILE* fp = fopen(file_path.c_str(),"rb" );
    if ( !fp ) {
        return bgr_img;
    }    
    uchar * src_420_cross_y = (uchar*)malloc(sizeof(uchar) * image_height * image_width);
    uchar * src_420_cross_u = (uchar*)malloc(sizeof(uchar) * image_height * image_width / 4);
    uchar * src_420_cross_v = (uchar*)malloc(sizeof(uchar) * image_height * image_width / 4);
    int y_count = fread(src_420_cross_y, sizeof(uchar),  image_height * image_width, fp);
    int u_count = fread(src_420_cross_u,sizeof(uchar),  image_height * image_width / 4, fp);
    int v_count = fread(src_420_cross_v,sizeof(uchar),  image_height * image_width / 4, fp);
    fclose( fp );

    cv::Mat dst_yuv_444 = cv::Mat_<cv::Vec3b>::zeros(image_height,image_width);
    std::vector<cv::Mat> dst_yuv_444_list(3);
    cv::split(dst_yuv_444, dst_yuv_444_list);

    memcpy(dst_yuv_444_list[0].data, src_420_cross_y, dst_yuv_444.rows * dst_yuv_444.cols * sizeof(unsigned char));

    cv::Mat src_420_u = cv::Mat_<uchar>::zeros(image_height / 2,image_width / 2);
    cv::Mat src_420_v = cv::Mat_<uchar>::zeros(image_height / 2,image_width / 2);
    memcpy(src_420_u.data, src_420_cross_u, dst_yuv_444.rows * dst_yuv_444.cols / 4 * sizeof(unsigned char));
    memcpy(src_420_v.data, src_420_cross_v, dst_yuv_444.rows * dst_yuv_444.cols / 4 * sizeof(unsigned char));
    cv::resize(src_420_u,dst_yuv_444_list[1],dst_yuv_444.size());
    cv::resize(src_420_v,dst_yuv_444_list[2],dst_yuv_444.size());
    cv::merge(dst_yuv_444_list, dst_yuv_444);

    cv::cvtColor(dst_yuv_444,bgr_img,CV_YUV2BGR);
    return bgr_img;
}

int Calibrater::PixelPointToGroundCoordinateOffline(cv::Point2f &ground_point, cv::Point2f pixel_point) const
{
    cv::Point2f uv_point;
    m_intrinsic_parameter.PixelPoint2UVPlanePoint(uv_point, pixel_point);
    m_offline_extinsic_parameter.UVPoint2GroundCoordinate(ground_point, uv_point);
}

int Calibrater::GroundCoordinateToPixelPointOffline(cv::Point2f &pixel_point, cv::Point2f ground_point) const
{
    cv::Point2f uv_point;
    m_offline_extinsic_parameter.GroundCoordinate2UVPoint(uv_point, ground_point);
    m_intrinsic_parameter.UVPlanePoint2PixelPoint(pixel_point, uv_point);
    return 1;
}

bool Calibrater::PixelPointToHRYTGroundPointOffline(cv::Point2f &hryt_ground_point, cv::Point2f pixel_point) const
{
    cv::Point2f uv_point;
    m_intrinsic_parameter.PixelPoint2UVPlanePoint(uv_point, pixel_point);
    cv::Point3f world_point;
    m_offline_extinsic_parameter.UVPoint2WorldCoordinate(world_point,uv_point,0);
    cv::Point3f hryt_3d_point;
    m_offline_extinsic_parameter.WorldCoordinate2HRYTCoordinate(hryt_3d_point,world_point);
    hryt_ground_point.x = hryt_3d_point.x;
    hryt_ground_point.y = hryt_3d_point.y;
    return true;
}

bool Calibrater::HRYTGroundPointToPixelPointOffline(cv::Point2f &pixel_point, cv::Point2f hryt_ground_point) const
{
    cv::Point3f hryt_3d_point;
    hryt_3d_point.x = hryt_ground_point.x;
    hryt_3d_point.y = hryt_ground_point.y;
    hryt_3d_point.z = 0;
    cv::Point3f world_point;
    m_offline_extinsic_parameter.HRYTCoordinate2WorldCoordinate(world_point,hryt_3d_point);
    cv::Point2f uv_point;
    m_offline_extinsic_parameter.WorldCoordinate2UVPoint(uv_point,world_point);
    m_intrinsic_parameter.UVPlanePoint2PixelPoint(pixel_point,uv_point);
    return true;
}

bool Calibrater::PixelPointToHRYT3DPointOffline(cv::Point3f &hryt_3d_point, cv::Point2f pixel_point, float point_zloc_hryt) const
{
    cv::Point2f uv_point;
    m_intrinsic_parameter.PixelPoint2UVPlanePoint(uv_point, pixel_point);
    cv::Point3f world_point;
    m_offline_extinsic_parameter.UVPoint2WorldCoordinate(world_point,uv_point,point_zloc_hryt);
    m_offline_extinsic_parameter.WorldCoordinate2HRYTCoordinate(hryt_3d_point,world_point);
    return true;
}

bool Calibrater::HRYT3DPointToPixelPointOffline(cv::Point2f& pixel_point, cv::Point3f  hryt_3d_point) const
{
    cv::Point3f world_point;
    m_offline_extinsic_parameter.HRYTCoordinate2WorldCoordinate(world_point,hryt_3d_point);
    cv::Point2f uv_point;
    m_offline_extinsic_parameter.WorldCoordinate2UVPoint(uv_point,world_point);
    m_intrinsic_parameter.UVPlanePoint2PixelPoint(pixel_point,uv_point);
    return true;
}

bool Calibrater::PixelPedestrainBBoxToHRYTHeadPointOffline(cv::Point3f &hryt_head_point, cv::Rect bounding_box) const
{
    cv::Point2f top_center_pixel = cv::Point2f(bounding_box.x + bounding_box.width / 2, bounding_box.y);
    cv::Point2f bottom_center_pixel = cv::Point2f(bounding_box.x + bounding_box.width / 2, bounding_box.y + bounding_box.height);
    cv::Point2f top_center_uv, bottom_center_uv;
    m_intrinsic_parameter.PixelPoint2UVPlanePoint(top_center_uv, top_center_pixel);
    m_intrinsic_parameter.PixelPoint2UVPlanePoint(bottom_center_uv, bottom_center_pixel);
    cv::Point3f head_center_world;
    m_offline_extinsic_parameter.UVPoint2WorldCoordinate(head_center_world,bottom_center_uv,0);
    float scale = head_center_world.z - m_offline_extinsic_parameter.camera_world_z;
    float height = std::abs(top_center_uv.y - bottom_center_uv.y) * scale;
    head_center_world.y = 0 - height;
    m_offline_extinsic_parameter.WorldCoordinate2HRYTCoordinate(hryt_head_point,head_center_world);
    return true;
}

bool Calibrater::accend_by_x(const cv::Point2f &pt1, const cv::Point2f &pt2)
{
    return pt1.x < pt2.x;
}

bool Calibrater::ChessCornerDetection(std::vector<cv::Point2f> &corners, const cv::Mat &gray_source, int max_grid_size, float max_rotate_angle)
{
    corners.clear();
    cv::Mat_<int> int_img;
    cv::integral(gray_source,int_img);
    cv::Mat_<int> likelihood_img=cv::Mat_<int>::zeros(gray_source.rows,gray_source.cols);

    int grid_size = max_grid_size / 2;
    int radius = grid_size * tan(max_rotate_angle * 180 / 3.1415926);

    cv::Rect one_patten[4];
    one_patten[0] = cv::Rect(radius,radius,grid_size,grid_size);
    one_patten[1] = one_patten[0] - cv::Point(one_patten[0].x * 2 + one_patten[0].width,one_patten[0].y * 2 + one_patten[0].height);
    one_patten[2] = one_patten[0] - cv::Point(0 , one_patten[0].y * 2 + one_patten[0].height);
    one_patten[3] = one_patten[0] - cv::Point(one_patten[0].x * 2 + one_patten[0].width , 0);
    cv::Rect patten_boundary = one_patten[0] | one_patten[1] | one_patten[2] |one_patten[3];
    float patten_value[4];

    cv::Rect effectiveregion=cv::Rect(0,0,likelihood_img.cols,likelihood_img.rows);
    for(int r=0;r < likelihood_img.rows;r++)
    {
        for(int c = 0;c < likelihood_img.cols;c++)
        {
            cv::Rect cur_boundary = patten_boundary + cv::Point(c,r);
            if(!((cur_boundary | effectiveregion)==effectiveregion)) continue;

            cv::Rect cur_patten;
            for(int k=0;k<4;k++)
            {
                cur_patten = one_patten[k] + cv::Point(c,r);
                patten_value[k] = int_img(cur_patten.tl()) + int_img(cur_patten.br())
                        -int_img(cv::Point(cur_patten.tl().x,cur_patten.br().y))
                        -int_img(cv::Point(cur_patten.br().x,cur_patten.tl().y));
            }
            likelihood_img(cv::Point(c,r)) = patten_value[0] + patten_value[1] - patten_value[2] - patten_value[3];
//            likelihood_img(cv::Point(c,r)) = std::min(patten_value[0], patten_value[1]) - std::max(patten_value[2], patten_value[3]);
        }
    }

    cv::Mat binary_right,binary_left,binary_img;
    double min_value,max_value;
    cv::minMaxLoc(likelihood_img,&min_value,&max_value);
    binary_right = likelihood_img > max_value * 0.9;
    binary_left = likelihood_img < min_value * 0.9;
    binary_img = binary_left | binary_right;

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(binary_img,contours,cv::RETR_EXTERNAL,cv::CHAIN_APPROX_NONE);
    if(contours.size()!=4)
    {
        return false;
    }

    for(int i=0;i<contours.size();i++)
    {
        cv::Moments cm = cv::moments(contours[i]);
        corners.push_back(cv::Point2f(cm.m10/cm.m00,cm.m01/cm.m00));
    }

    std::sort(corners.begin(),corners.end(),Calibrater::accend_by_x);

    //check if the corners is validate
    float len1 = cv::norm(corners[1]-corners[0]);
//    float len2 = cv::norm(corners[1]-corners[0]);
    float len3 = cv::norm(corners[1]-corners[0]);
    if(len1 / len3 < 0.5 || len3 / len1 < 0.5)
    {
        return false;
    }

    int sub_grid_size = (len1 + len3) /2;
    #ifdef AISDK_INTERFACE
    #else
    cv::find4QuadCornerSubpix(gray_source,corners,cv::Size(sub_grid_size,sub_grid_size));
    #endif
    return true;
}



bool Calibrater::ExtinsicParameter::InitByWorldLocation(float camera_world_x, float camera_world_y, float camera_world_z, float pitch_angle, float yaw_angle, float roll_angle)
{
    this->camera_world_x = camera_world_x;
    this->camera_world_y = camera_world_y;
    this->camera_height = -this->camera_world_y;
    this->camera_world_z = camera_world_z;
    const float PI = 3.1415926f;
    float arc_pitch_angle = pitch_angle * PI/180.0f;
    float arc_yaw_angle = yaw_angle * PI / 180.0f;
    float arc_roll_angle = roll_angle * PI / 180.0f;

    cv::Mat pitchMatrix = (cv::Mat_<float>(3, 3) << 1, 0, 0
        , 0, std::cos(arc_pitch_angle), std::sin(arc_pitch_angle)
        , 0, -std::sin(arc_pitch_angle), std::cos(arc_pitch_angle));
    cv::Mat yaw_Matrix = (cv::Mat_<float>(3, 3) << std::cos(arc_yaw_angle), 0, -std::sin(arc_yaw_angle)
        , 0, 1, 0
        , std::sin(arc_yaw_angle), 0, std::cos(arc_yaw_angle));
    cv::Mat roll_Matrix = (cv::Mat_<float>(3, 3) << std::cos(arc_roll_angle), std::sin(arc_roll_angle), 0
        , -std::sin(arc_roll_angle), std::cos(arc_roll_angle), 0
        , 0, 0, 1);

    this->rotateMatrix = roll_Matrix*yaw_Matrix*pitchMatrix;
    this->rotateMatrix_inv = this->rotateMatrix.inv();
    this->cameraLocationInWorld = (cv::Mat_<float>(3, 1) <<
        this->camera_world_x, -this->camera_height, this->camera_world_z);
    this->ground_plane_world_y = 0;
    return true;
}

bool Calibrater::ExtinsicParameter::InitByHRYTLocation(float camera_hryt_x, float camera_hryt_y, float camera_hryt_z, float pitch_angle, float yaw_angle, float roll_angle)
{
    this->camera_world_x = -camera_hryt_y;
    this->camera_world_y = -camera_hryt_z;
    this->camera_height = -this->camera_world_y;
    this->camera_world_z = camera_hryt_x;

    const float PI = 3.1415926f;
    float arc_pitch_angle = pitch_angle * PI/180.0f;
    float arc_yaw_angle = yaw_angle * PI / 180.0f;
    float arc_roll_angle = roll_angle * PI / 180.0f;

    cv::Mat pitchMatrix = (cv::Mat_<float>(3, 3) << 1, 0, 0
        , 0, std::cos(arc_pitch_angle), std::sin(arc_pitch_angle)
        , 0, -std::sin(arc_pitch_angle), std::cos(arc_pitch_angle));
    cv::Mat yaw_Matrix = (cv::Mat_<float>(3, 3) << std::cos(arc_yaw_angle), 0, -std::sin(arc_yaw_angle)
        , 0, 1, 0
        , std::sin(arc_yaw_angle), 0, std::cos(arc_yaw_angle));
    cv::Mat roll_Matrix = (cv::Mat_<float>(3, 3) << std::cos(arc_roll_angle), std::sin(arc_roll_angle), 0
        , -std::sin(arc_roll_angle), std::cos(arc_roll_angle), 0
        , 0, 0, 1);

    this->rotateMatrix = roll_Matrix*yaw_Matrix*pitchMatrix;
    this->rotateMatrix_inv = this->rotateMatrix.inv();
    this->cameraLocationInWorld = (cv::Mat_<float>(3, 1) <<
        this->camera_world_x, -this->camera_height, this->camera_world_z);
    this->ground_plane_world_y = 0;
    return true;
}

void Calibrater::ExtinsicParameter::UVPoint2GroundCoordinate(cv::Point2f &ground_point, cv::Point2f uv_point) const
{
    cv::Mat_<float> point_in_camera_coordinate = (cv::Mat_<float>(3, 1) << uv_point.x, uv_point.y, 1);
    point_in_camera_coordinate = rotateMatrix_inv  * point_in_camera_coordinate;
    float scale = (0 - cameraLocationInWorld.at<float>(1)) / point_in_camera_coordinate.at<float>(1);
    point_in_camera_coordinate = point_in_camera_coordinate * scale;

    point_in_camera_coordinate = point_in_camera_coordinate + cameraLocationInWorld;
    ground_point.x = point_in_camera_coordinate.at<float>(0, 0);
    ground_point.y = point_in_camera_coordinate.at<float>(2, 0);
}

void Calibrater::ExtinsicParameter::GroundCoordinate2UVPoint(cv::Point2f &uv_point, cv::Point2f ground_point) const
{
    cv::Mat_<float> point_in_camera_coordinate = (cv::Mat_<float>(3, 1) << ground_point.x, this->ground_plane_world_y, ground_point.y);

    point_in_camera_coordinate = point_in_camera_coordinate - cameraLocationInWorld;
    point_in_camera_coordinate = rotateMatrix * point_in_camera_coordinate;
    uv_point.x = point_in_camera_coordinate.at<float>(0, 0) / point_in_camera_coordinate.at<float>(2, 0);
    uv_point.y = point_in_camera_coordinate.at<float>(1, 0) / point_in_camera_coordinate.at<float>(2, 0);
}

void Calibrater::ExtinsicParameter::UVPoint2WorldCoordinate(cv::Point3f &world_coordinate, cv::Point2f uv_point, float height_on_ground) const
{
    cv::Mat_<float> point_in_camera_coordinate = (cv::Mat_<float>(3, 1) << uv_point.x, uv_point.y, 1);
    point_in_camera_coordinate = rotateMatrix_inv  * point_in_camera_coordinate;
    float scale;
    if (-height_on_ground == cameraLocationInWorld.at<float>(1))
    {
        scale = FLT_MAX;
    }
    else
    {
        scale = (-height_on_ground - cameraLocationInWorld.at<float>(1)) / point_in_camera_coordinate.at<float>(1);
    }
    point_in_camera_coordinate = point_in_camera_coordinate * scale;

    point_in_camera_coordinate = point_in_camera_coordinate + cameraLocationInWorld;
    world_coordinate.x = point_in_camera_coordinate.at<float>(0, 0);
    world_coordinate.y = point_in_camera_coordinate.at<float>(1, 0);
    world_coordinate.z = point_in_camera_coordinate.at<float>(2, 0);
}

void Calibrater::ExtinsicParameter::WorldCoordinate2UVPoint(cv::Point2f &uv_point, cv::Point3f world_coordinate) const
{
    cv::Mat_<float> point_in_camera_coordinate = (cv::Mat_<float>(3, 1) << world_coordinate.x, world_coordinate.y, world_coordinate.z);

    point_in_camera_coordinate = point_in_camera_coordinate - cameraLocationInWorld;
    point_in_camera_coordinate = rotateMatrix * point_in_camera_coordinate;
    uv_point.x = point_in_camera_coordinate.at<float>(0, 0) / point_in_camera_coordinate.at<float>(2, 0);
    uv_point.y = point_in_camera_coordinate.at<float>(1, 0) / point_in_camera_coordinate.at<float>(2, 0);
}

void Calibrater::ExtinsicParameter::HRYTCoordinate2WorldCoordinate(cv::Point3f &world_coordinate, cv::Point3f HRYT_coordinate)
{
    world_coordinate.x = -HRYT_coordinate.y;
    world_coordinate.y = -HRYT_coordinate.z;
    world_coordinate.z = HRYT_coordinate.x;
}

void Calibrater::ExtinsicParameter::WorldCoordinate2HRYTCoordinate(cv::Point3f &HRYT_coordinate, cv::Point3f world_coordinate)
{
    HRYT_coordinate.x = world_coordinate.z;
    HRYT_coordinate.y = -world_coordinate.x;
    HRYT_coordinate.z = -world_coordinate.y;
}

bool Calibrater::IntrinsicParameter::Init(float fx, float fy, float cx, float cy, float k1, float k2, float k3, float p1, float p2)
{
    this->fx = fx;
    this->fy = fy;
    this->cx = cx;
    this->cy = cy;
    this->k1 = k1;
    this->k2 = k2;
    this->k3 = k3;
    this->p1 = p1;
    this->p2 = p2;

    this->IntrinsicMatrix = (cv::Mat_<float>(3, 3) <<
        this->fx, 0, this->cx
        , 0, this->fy, this->cy
        , 0, 0, 1.0);

    this->distCoeffs = (cv::Mat_<float>(1, 5) <<
        this->k1, this->k2
        , this->p1, this->p2,
        this->k3);

    this->mtx_empty = cv::Mat_<float>::zeros(3, 1);
    return true;
}

void Calibrater::IntrinsicParameter::UVPlanePoint2PixelPoint(cv::Point2f &pixel_point, cv::Point2f uv_point) const
{
    std::vector<cv::Point3f> src_point_list(1);
    src_point_list[0].x = uv_point.x;
    src_point_list[0].y = uv_point.y;
    src_point_list[0].z = 0;
    std::vector<cv::Point2f> dst_point_list(1);
    #ifdef AISDK_INTERFACE
    #else
    cv::projectPoints(src_point_list, mtx_empty, mtx_empty, IntrinsicMatrix, distCoeffs, dst_point_list);
    #endif
    pixel_point = dst_point_list[0];
}

void Calibrater::IntrinsicParameter::PixelPoint2UVPlanePoint(cv::Point2f &uv_point, cv::Point2f pixel_point) const
{
    std::vector<cv::Point2f> point_list_src(1), point_list_dst(1);
    point_list_src[0] = pixel_point;

    cv::undistortPoints(point_list_src, point_list_dst, IntrinsicMatrix, distCoeffs);
    uv_point = point_list_dst[0];
}
