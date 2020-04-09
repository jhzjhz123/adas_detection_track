#include "Calibrater.h"
OnlineCalibrater::OnlineCalibrater()
{

}

bool OnlineCalibrater::load_offline_parameters(const std::string &offline_camera_parameters_file)
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
    return true;
}

void OnlineCalibrater::PixelPointToGroundCoordinateOffline(cv::Point2f &ground_point, cv::Point2f pixel_point) const
{
    cv::Point2f uv_point;
    m_intrinsic_parameter.PixelPoint2UVPlanePoint(uv_point, pixel_point);
    m_offline_extinsic_parameter.UVPoint2GroundCoordinate(ground_point, uv_point);
}

void OnlineCalibrater::GroundCoordinateToPixelPointOffline(cv::Point2f &pixel_point, cv::Point2f ground_point) const
{
    cv::Point2f uv_point;
    m_offline_extinsic_parameter.GroundCoordinate2UVPoint(uv_point, ground_point);
    m_intrinsic_parameter.UVPlanePoint2PixelPoint(pixel_point, uv_point);
}

void OnlineCalibrater::PixelPointToWorldCoordinate(cv::Point3f &world_coordinate, cv::Point2f pixel_point, float height_on_ground) const
{
    cv::Point2f uv_point;
    m_intrinsic_parameter.PixelPoint2UVPlanePoint(uv_point, pixel_point);
    m_offline_extinsic_parameter.UVPoint2WorldCoordinate(world_coordinate, uv_point, height_on_ground);
}

void OnlineCalibrater::WorldCoordinateToPixelPoint(cv::Point2f &pixel_point, cv::Point3f world_coordinate) const
{
    cv::Point2f uv_point;
    m_offline_extinsic_parameter.WorldCoordinate2UVPoint(uv_point, world_coordinate);
    m_intrinsic_parameter.UVPlanePoint2PixelPoint(pixel_point, uv_point);
}

void OnlineCalibrater::PixelPointToGroundCoordinate(cv::Point2f &ground_point, cv::Point2f pixel_point) const
{
    cv::Point2f uv_point;
    m_intrinsic_parameter.PixelPoint2UVPlanePoint(uv_point, pixel_point);
    m_offline_extinsic_parameter.UVPoint2GroundCoordinate(ground_point, uv_point);
}

void OnlineCalibrater::GroundCoordinateToPixelPoint(cv::Point2f &pixel_point, cv::Point2f ground_point) const
{
    cv::Point2f uv_point;
    m_offline_extinsic_parameter.GroundCoordinate2UVPoint(uv_point, ground_point);
    m_intrinsic_parameter.UVPlanePoint2PixelPoint(pixel_point, uv_point);
}

bool IntrinsicParameter::Init(cv::Mat cameraMatrix, cv::Mat dist_coeffs)
{
    this->IntrinsicMatrix = cameraMatrix;
    this->distCoeffs = dist_coeffs;
    this->mtx_empty = cv::Mat_<float>::zeros(3, 1);

    this->fx = this->IntrinsicMatrix(0,0);
    this->fy = this->IntrinsicMatrix(1,1);
    this->cx = this->IntrinsicMatrix(0,2);
    this->cy = this->IntrinsicMatrix(1,2);
    this->k1 = this->distCoeffs(0);
    this->k2 = this->distCoeffs(1);
    this->k3 = this->distCoeffs(4);
    this->p1 = this->distCoeffs(2);
    this->p2 = this->distCoeffs(3);
    return true;
}

bool IntrinsicParameter::Init(float fx, float fy, float cx, float cy, float k1, float k2, float k3, float p1, float p2)
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

void IntrinsicParameter::UVPlanePoint2PixelPoint(cv::Point2f &pixel_point, cv::Point2f uv_point) const
{
    std::vector<cv::Point3f> src_point_list(1);
    src_point_list[0].x = uv_point.x;
    src_point_list[0].y = uv_point.y;
    src_point_list[0].z = 0;
    std::vector<cv::Point2f> dst_point_list(1);
    cv::projectPoints(src_point_list, mtx_empty, mtx_empty, IntrinsicMatrix, distCoeffs, dst_point_list);
    pixel_point = dst_point_list[0];
}

void IntrinsicParameter::PixelPoint2UVPlanePoint(cv::Point2f &uv_point, cv::Point2f pixel_point) const
{
    std::vector<cv::Point2f> point_list_src(1), point_list_dst(1);
    point_list_src[0] = pixel_point;

    cv::undistortPoints(point_list_src, point_list_dst, IntrinsicMatrix, distCoeffs);
    uv_point = point_list_dst[0];
}

bool ExtinsicParameter::InitByWorldLocation(float camera_world_x, float camera_world_y, float camera_world_z, float pitch_angle, float yaw_angle, float roll_angle)
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

void ExtinsicParameter::UVPoint2GroundCoordinate(cv::Point2f &ground_point, cv::Point2f uv_point) const
{
    cv::Mat_<float> point_in_camera_coordinate = (cv::Mat_<float>(3, 1) << uv_point.x, uv_point.y, 1);
    point_in_camera_coordinate = rotateMatrix_inv  * point_in_camera_coordinate;
    float scale = (0 - cameraLocationInWorld.at<float>(1)) / point_in_camera_coordinate.at<float>(1);
    point_in_camera_coordinate = point_in_camera_coordinate * scale;

    point_in_camera_coordinate = point_in_camera_coordinate + cameraLocationInWorld;
    ground_point.x = point_in_camera_coordinate.at<float>(0, 0);
    ground_point.y = point_in_camera_coordinate.at<float>(2, 0);
}

void ExtinsicParameter::GroundCoordinate2UVPoint(cv::Point2f &uv_point, cv::Point2f ground_point) const
{
    cv::Mat_<float> point_in_camera_coordinate = (cv::Mat_<float>(3, 1) << ground_point.x, this->ground_plane_world_y, ground_point.y);

    point_in_camera_coordinate = point_in_camera_coordinate - cameraLocationInWorld;
    point_in_camera_coordinate = rotateMatrix * point_in_camera_coordinate;
    uv_point.x = point_in_camera_coordinate.at<float>(0, 0) / point_in_camera_coordinate.at<float>(2, 0);
    uv_point.y = point_in_camera_coordinate.at<float>(1, 0) / point_in_camera_coordinate.at<float>(2, 0);
}

void ExtinsicParameter::UVPoint2WorldCoordinate(cv::Point3f &world_coordinate, cv::Point2f uv_point, float height_on_ground) const
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

void ExtinsicParameter::WorldCoordinate2UVPoint(cv::Point2f &uv_point, cv::Point3f world_coordinate) const
{
    cv::Mat_<float> point_in_camera_coordinate = (cv::Mat_<float>(3, 1) << world_coordinate.x, world_coordinate.y, world_coordinate.z);

    point_in_camera_coordinate = point_in_camera_coordinate - cameraLocationInWorld;
    point_in_camera_coordinate = rotateMatrix * point_in_camera_coordinate;
    uv_point.x = point_in_camera_coordinate.at<float>(0, 0) / point_in_camera_coordinate.at<float>(2, 0);
    uv_point.y = point_in_camera_coordinate.at<float>(1, 0) / point_in_camera_coordinate.at<float>(2, 0);
}

void OfflineCalibrater::readYUV420File(cv::Mat &img,const std::string file_name)
{
    int img_width=1280;
    int img_height = 720;
    int yuv_buffer_size, bgr_buffer_size;
    char *yuv_data_buffer;
    char *bgr_data_buffer;

    yuv_buffer_size = (img_width * img_height * 3) >> 1;
    yuv_data_buffer = (char *)malloc(sizeof(unsigned char) * yuv_buffer_size);

    std::ifstream fin;
//    cout << "\nLoad images : " << image_name << endl;

    fin.open(file_name);
    fin.read(yuv_data_buffer, yuv_buffer_size);
    fin.close();

    cv::Mat src_yuv420_mat((img_height >> 1)*3, img_width, CV_8UC1, yuv_data_buffer);
    cv::Mat bgrImg(img_height, img_width, CV_8UC3);

    cv::cvtColor(src_yuv420_mat, bgrImg, CV_YUV2BGR_I420);
    bgrImg.copyTo(img);
}


bool OfflineCalibrater::IntrinsicCalibration(const std::string& setting_file)
{
    LoadIntrinsicSettingsFile(setting_file);

    std::vector<std::string> graphSuccess;

    cv::Mat curGraph;  // current image
    cv::Mat gray;      // gray image of current image

    int imageCount = m_intrinsic_image_files.size();
    int imageCountSuccess = 0;
    cv::Size image_size;

    std::vector<cv::Point2f> corners;                  // one image corner list
    std::vector<std::vector<cv::Point2f> > seqImageCorners; // n images corner list

    if ( m_intrinsic_image_files.size() < 3 )
    {
        std::cout << "Calibrate failed, with less than three images!" << std::endl;
        return false;
    }

    for ( int i=0; i<m_intrinsic_image_files.size(); i++ )
    {
        std::string ext = m_intrinsic_image_files[i].substr(m_intrinsic_image_files[i].rfind('.') + 1,m_intrinsic_image_files[i].size());
        transform(ext.begin(), ext.end(), ext.begin(), ::tolower);
        if(ext.compare("420")==0)
        {
            readYUV420File(curGraph,m_intrinsic_image_files[i]);
        }
        else
        {
            curGraph = cv::imread(m_intrinsic_image_files[i]);
        }

        if ( curGraph.channels() == 3 )
            cv::cvtColor( curGraph, gray, CV_BGR2GRAY );
        else
            curGraph.copyTo( gray );

        // for every image, empty the corner list
        std::vector<cv::Point2f>().swap( corners );

        // corners detection
        bool success = cv::findChessboardCorners( curGraph, m_intrinsic_corners_size, corners );

        if ( success ) // succeed
        {
            std::cout << i << " find corners in " << m_intrinsic_image_files[i] << " succeed"<< std::endl;

            int row = curGraph.rows;
            int col = curGraph.cols;

            graphSuccess.push_back( m_intrinsic_image_files[i] );
            imageCountSuccess ++;

            image_size = cv::Size( col, row );

            // find sub-pixels
            cv::cornerSubPix(
                gray,
                corners,
                cv::Size( 11, 11 ),
                cv::Size( -1, -1 ),
                cv::TermCriteria( CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1 ) );
            seqImageCorners.push_back( corners );

            if(m_show_processing)
            {
                // draw corners and show them in current image
                cv::Mat imageDrawCorners;
                if ( curGraph.channels() == 3 )
                    curGraph.copyTo( imageDrawCorners );
                else
                    cv::cvtColor( curGraph, imageDrawCorners, CV_GRAY2RGB );

                for ( int j = 0; j < corners.size(); j ++)
                {
                    cv::Point2f dotPoint = corners[j];
                    cv::circle( imageDrawCorners, dotPoint, 3.0, cv::Scalar( 0, 255, 0 ), -1 );
                    cv::Point2f pt_m = dotPoint + cv::Point2f(4,4);
                    char text[100];
                    sprintf( text, "%d", j+1 );  // corner indexes which start from 1
                    cv::putText( imageDrawCorners, text, pt_m, 1, 0.5, cv::Scalar( 255, 0, 255 ) );
                }
                cv::imshow("imageDrawCorners",imageDrawCorners);
                cv::waitKey(-1);
            }
        }
        else // failed
        {
            std::cout << m_intrinsic_image_files[i] << " corner detect failed!" << std::endl;
        }
    }
    cv::destroyAllWindows();
    std::cout << "Corner detect done!" << std::endl
                << imageCountSuccess << " images succeed! " << std::endl;

    if ( imageCountSuccess < 3 )
    {
        std::cout << "Calibrated success " << imageCountSuccess
            << " images, less than 3 images." << std::endl;
        return false;
    }
    else
    {
        std::cout << "Start calibration ..." << std::endl;

        cv::Point3f point3D;
        std::vector<cv::Point3f> objectPoints;
        cv::Mat distCoeffs = cv::Mat(1, 5, CV_32FC1, cv::Scalar::all(0));
        std::vector<double> rotation;
        std::vector<double> translation;

        std::vector<std::vector<cv::Point3f>> seqObjectPoints;
        std::vector<cv::Mat> rvecsMat;
        std::vector<cv::Mat> tvecsMat;
        cv::Mat cameraMatrix = cv::Mat(3, 3, CV_32FC1, cv::Scalar::all(0));

        // calibration pattern points in the calibration pattern coordinate space
        for ( int t=0; t<imageCountSuccess; t++ )
        {
            objectPoints.clear();
            for ( int i=0; i<m_intrinsic_corners_size.height; i++ )
            {
                for ( int j=0; j<m_intrinsic_corners_size.width; j++ )
                {
                    point3D.x = i * m_intrinsic_chess_cell_size.width;
                    point3D.y = j * m_intrinsic_chess_cell_size.height;
                    point3D.z = 0;
                    objectPoints.push_back(point3D);
                }
            }
            seqObjectPoints.push_back(objectPoints);
        }
        double reprojectionError = cv::calibrateCamera(
                    seqObjectPoints
                    , seqImageCorners
                    , image_size
                    , cameraMatrix
                    , distCoeffs
                    , rvecsMat
                    , tvecsMat
                    , CV_CALIB_FIX_K4);
        std::cout<<"mean reprojection error : "<<reprojectionError<<" pixels"<<std::endl;
        std::cout << "estimate errors in each image: "<<std::endl;
        double err = 0.0;
        std::vector<cv::Point2f> image_points_pro;

        for(int i=0;i<graphSuccess.size();i++)
        {
            cv::projectPoints(seqObjectPoints[i],rvecsMat[i], tvecsMat[i], cameraMatrix, distCoeffs, image_points_pro);
            err = cv::norm(cv::Mat(seqImageCorners[i]), cv::Mat(image_points_pro), cv::NORM_L2);
            err /= sqrt(seqObjectPoints[i].size());
            std::cout << "reproject error ：" << err << "pixel in "<< graphSuccess[i] << std::endl;
        }

        m_intrinsic_parameter.Init(cameraMatrix,distCoeffs);

        this->SaveIntrinsicParameters();
        std::cout << "Calibration done!" << std::endl;

//        if(show_rlt)
//        {
//            std::cout << "show undistored image : " << std::endl;

//            for (int i = 0; i < graphSuccess.size(); i++)
//            {
//                cv::Mat imageSource = cv::imread(graphSuccess[i]);
//                cv::Mat newimage = imageSource.clone();
//                cv::undistort(imageSource, newimage, cameraMatrix, distCoeffs);
//                cv::imshow("undistored image",newimage);
//                cv::waitKey(-1);
//            }
//        }
    }

    return true;
}

bool OfflineCalibrater::Alphanum_less(std::string left, std::string right)
{
    return Alphanum_comp(left, right) < 0;
}

bool OfflineCalibrater::Alphanum_isdigit(const char c)
{
    return c >= '0' && c <= '9';
}

int OfflineCalibrater::Alphanum_comp(std::string left, std::string right)
{
    const char *l = left.c_str();
    const char *r = right.c_str();

    enum mode_t { STRING, NUMBER } mode = STRING;

    while (*l && *r)
    {
        if (mode == STRING)
        {
            char l_char, r_char;
            while ((l_char = *l) && (r_char = *r))
            {
                // check if this are digit characters
                const bool l_digit = Alphanum_isdigit(l_char), r_digit = Alphanum_isdigit(r_char);
                // if both characters are digits, we continue in NUMBER mode
                if (l_digit && r_digit)
                {
                    mode = NUMBER;
                    break;
                }
                // if only the left character is a digit, we have a result
                if (l_digit) return -1;
                // if only the right character is a digit, we have a result
                if (r_digit) return +1;
                // compute the difference of both characters
                const int diff = l_char - r_char;
                // if they differ we have a result
                if (diff != 0) return diff;
                // otherwise process the next characters
                ++l;
                ++r;
            }
        }
        else // mode==NUMBER
        {
#ifdef ALPHANUM_LOCALE
            // get the left number
            char *end;
            unsigned long l_int = strtoul(l, &end, 0);
            l = end;

            // get the right number
            unsigned long r_int = strtoul(r, &end, 0);
            r = end;
#else
            // get the left number
            unsigned long l_int = 0;
            while (*l && Alphanum_isdigit(*l))
            {
                // TODO: this can overflow
                l_int = l_int * 10 + *l - '0';
                ++l;
            }

            // get the right number
            unsigned long r_int = 0;
            while (*r && Alphanum_isdigit(*r))
            {
                // TODO: this can overflow
                r_int = r_int * 10 + *r - '0';
                ++r;
            }
#endif

            // if the difference is not equal to zero, we have a comparison result
            const long diff = l_int - r_int;
            if (diff != 0)
                return diff;

            // otherwise we process the next substring in STRING mode
            mode = STRING;
        }
    }

    if (*r) return -1;
    if (*l) return +1;
    return 0;
}

bool OfflineCalibrater::ExtrinsicCalibration(const std::string &setting_file)
{
    bool success=true;
    success = this->LoadExtrinsicSettingsFile(setting_file);
    if(!success)
    {
        std::cout<<"Extrinsic calibration failed."<<std::endl;
        return false;
    }

    std::cout<<"Start extrinsic calibration based on video..."<<std::endl;
    this->LoadIntrinsicParametersFile(m_extrinsic_based_intrinsic_file);
//    success = this->CalibrationByVideo(m_show_processing);
    success = this->CalibrationByImages(m_show_processing);
    if(!success)
    {
        std::cout<<"Extrinsic calibration failed."<<std::endl;
        return false;
    }
    this->SaveExtrinsicParameters();
    std::cout<<"Extrinsic calibration based on video is done."<<std::endl;

    return true;
}

bool OfflineCalibrater::RangeTesting(const std::string &camera_param_file)
{
    LoadCameraParametersFile(camera_param_file);

    std::cout<<"\t image for range testing files : "<<std::endl;
    std::string int_dir;
    GetDerectoryName(int_dir, camera_param_file);
    GetFiles(m_range_test_image_files,int_dir);
    std::sort(m_range_test_image_files.begin(), m_range_test_image_files.end(), Alphanum_less);

    for(int i=0;i<m_range_test_image_files.size();i++)
    {
        std::cout<<"\t\t"<<m_range_test_image_files[i]<<std::endl;
    }

    cv::Mat image_frame;
    cv::Mat gray_img;

    std::vector<cv::Point2f> ground_points;
    for(int i=1;i<11;i++)
    {
        ground_points.push_back(cv::Point2f(0,10 * i));
    }
    ground_points.push_back(cv::Point2f(0,1000));

    cv::Point2f camera_center = cv::Point2f(m_intrinsic_parameter.cx,m_intrinsic_parameter.cy);
    cv::Point2f vanish_point = cv::Point2f(0,10000);
    this->GroundCoordinateToPixelPoint(vanish_point,vanish_point);

    std::vector<cv::Point2f> grid_points;
    int grid_cols = 5;
    int grid_rows = 4;
    for(int i=1;i<=grid_rows;i++)
    {
        for(int j=0;j<grid_cols;j++)
        {
            grid_points.push_back(cv::Point2f((j+0.5) * 1280 / grid_cols,vanish_point.y +  i * (720 - vanish_point.y) / grid_rows));
        }
    }

    for ( int i=0; i<m_range_test_image_files.size(); i++ )
    {
        std::string ext = m_range_test_image_files[i].substr(m_range_test_image_files[i].rfind('.') + 1,m_range_test_image_files[i].size());
        transform(ext.begin(), ext.end(), ext.begin(), ::tolower);
        if(ext.compare("420")==0)
        {
            readYUV420File(image_frame,m_range_test_image_files[i]);
        }
        else
        {
            image_frame = cv::imread(m_range_test_image_files[i]);
        }
        if ( image_frame.channels() == 3 )
            cv::cvtColor( image_frame, gray_img, CV_BGR2GRAY );
        else
            image_frame.copyTo( gray_img );

        if(image_frame.empty())
        {
            continue;
        }

        cv::circle(image_frame,vanish_point,4,cv::Scalar(0,255,0),-1);
        cv::circle(image_frame,camera_center,4,cv::Scalar(255,0,0),-1);

        cv::Point2f line_center,line_left,line_right;
        for(int i=0;i<ground_points.size();i++)
        {
            this->GroundCoordinateToPixelPoint(line_left,ground_points[i] - cv::Point2f(8,0));
            this->GroundCoordinateToPixelPoint(line_right,ground_points[i] + cv::Point2f(8,0));

//            line_left = line_center;
//            line_left.x = 0;
//            line_right = line_center;
//            line_right.x = 1279;
            cv::line(image_frame,line_left,line_right,cv::Scalar(0,0,255));

            std::stringstream sstream;
            sstream << ground_points[i].y;
            std::string dis = sstream.str();
            cv::putText(image_frame,dis + "m",line_left,1,1,cv::Scalar(0,255,0));
        }

        cv::Point2f grid_temp;
        for(int i=0;i<grid_points.size();i++)
        {
            cv::circle(image_frame,grid_points[i],3,cv::Scalar(0,255,255),-1);
            this->PixelPointToGroundCoordinate(grid_temp,grid_points[i]);
            std::stringstream sstream;
            sstream << grid_temp;
            std::string loc = sstream.str();
            cv::putText(image_frame,loc,grid_points[i] - cv::Point2f(100,0),1,1,cv::Scalar(0,255,0));
        }
        cv::imshow("frame",image_frame);
        cv::imwrite(m_range_test_image_files[i] + ".png",image_frame);
        cv::waitKey(-1);
    }
    return true;
}

void OfflineCalibrater::PixelPointToGroundCoordinate(cv::Point2f &ground_point, cv::Point2f pixel_point) const
{
    cv::Point2f uv_point;
    m_intrinsic_parameter.PixelPoint2UVPlanePoint(uv_point, pixel_point);
    m_extinsic_parameter.UVPoint2GroundCoordinate(ground_point, uv_point);
}

void OfflineCalibrater::GroundCoordinateToPixelPoint(cv::Point2f &pixel_point, cv::Point2f ground_point) const
{
    cv::Point2f uv_point;
    m_extinsic_parameter.GroundCoordinate2UVPoint(uv_point, ground_point);
    m_intrinsic_parameter.UVPlanePoint2PixelPoint(pixel_point, uv_point);
}

bool OfflineCalibrater::EstimateVanishPoint(cv::Point2f &predict_vanish_point, const cv::Mat& frame, cv::Point2f last_vanish_point)
{
    cv::Rect lane_area;
    cv::Point2f base_point = last_vanish_point;
    lane_area = cv::Rect(0,frame.rows / 2 + 50,frame.cols , frame.rows/2 - 50);

    cv::Mat lane_image =frame(lane_area).clone();
    cv::Canny(lane_image,lane_image,50,90);

    cv::imshow("lane_image",lane_image);

    std::vector<cv::Vec4f> lane_lines;
    cv::HoughLinesP(lane_image,lane_lines,1,CV_PI / 180.f,30,30,10);

    cv::Mat test = cv::Mat::zeros(720,1280,CV_8UC3);

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
            c_pt = c_pt+cv::Point2f(0,lane_area.tl().y);
            cross_points.push_back(c_pt);
            cv::circle(test,c_pt,1,cv::Scalar(255,0,0));
        }
    }
    cv::imshow("test",test);

    cv::Point2f weighted_point_sum = cv::Point2f(0,0);
    float weighted_sum = 0;
    float delta = 30;
    for(int i=0;i<cross_points.size();i++)
    {
        float dis = cv::norm(cross_points[i]-last_vanish_point);
        float weight = std::exp(-dis * dis / delta /delta);
        weighted_sum+=weight;
        weighted_point_sum+=cross_points[i] * weight;
    }

    if(weighted_sum == 0)
    {
        predict_vanish_point = last_vanish_point;
    }
    else
    {
        predict_vanish_point = weighted_point_sum / weighted_sum;
    }

    return true;
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

bool OfflineCalibrater::CalibrationByImages(bool show_rlt)
{
    cv::Mat image_frame;
    cv::Mat gray_img;

    bool EndProcessing = false;
    cv::Point2f camera_center = cv::Point2f(m_intrinsic_parameter.cx,m_intrinsic_parameter.cy);
    cv::Point2f vanish_point = cv::Point2f(0,0);
    cv::Point2f vanish_point_sum=cv::Point2f(0,0);
    cv::Point2f mean_vanish_point;
    float v_count = 0;
    for ( int i=0; i<m_extrinsic_image_files.size() && !EndProcessing; i++ )
    {
        std::string ext = m_extrinsic_image_files[i].substr(m_extrinsic_image_files[i].rfind('.') + 1,m_extrinsic_image_files[i].size());
        transform(ext.begin(), ext.end(), ext.begin(), ::tolower);
        if(ext.compare("420")==0)
        {
            readYUV420File(image_frame,m_extrinsic_image_files[i]);
        }
        else
        {
            image_frame = cv::imread(m_extrinsic_image_files[i]);
        }

        if ( image_frame.channels() == 3 )
            cv::cvtColor( image_frame, gray_img, CV_BGR2GRAY );
        else
            image_frame.copyTo( gray_img );

        if(image_frame.empty())
        {
            continue;
        }

        if(vanish_point==cv::Point2f(0,0))
        {
            vanish_point = cv::Point2f(image_frame.cols/2.f,image_frame.rows/2.f);
        }
        cv::cvtColor(image_frame,gray_img,CV_BGR2GRAY);

        cv::Point2f next_vanish;
        this->EstimateVanishPoint(next_vanish, gray_img, vanish_point);
        vanish_point = next_vanish;
        vanish_point_sum += vanish_point;
        v_count++;
        if(v_count!=0)
        {
            mean_vanish_point = vanish_point_sum / v_count;
        }

        if(show_rlt)
        {
            cv::circle(image_frame,vanish_point,4,cv::Scalar(0,255,0),-1);
            cv::circle(image_frame,mean_vanish_point,4,cv::Scalar(0,0,255),-1);
            cv::circle(image_frame,camera_center,4,cv::Scalar(255,0,0),-1);
            cv::imshow("frame",image_frame);

            char character_press = cv::waitKey(1);
            switch (character_press)
            {
            case 'q':
                EndProcessing = true;
                break;
            case 'c':
                cv::waitKey(-1);
                break;
            default:
                break;
            }
        }
    }

    m_extrinsic_vanish_point = mean_vanish_point;
    cv::Point2f vanish_uv;
    m_intrinsic_parameter.PixelPoint2UVPlanePoint(vanish_uv, mean_vanish_point);
    m_extinsic_parameter.camera_pitch_angle = atan(vanish_uv.y) / CV_PI * 180.f;
    m_extinsic_parameter.camera_yaw_angle = -atan(vanish_uv.x) / CV_PI * 180.f;
    return true;
}

bool OfflineCalibrater::CalibrationByVideo(bool show_rlt)
{
    cv::VideoCapture m_inputVideoHandler;
    bool success_open = m_inputVideoHandler.open(m_extrinsic_video_file);
    if(!success_open)
    {
        std::cout << "Failed to open video"<<std::endl;
        return false;
    }

    cv::Mat image_frame;
    cv::Mat gray_img;
    bool EndProcessing = false;
    cv::Point2f vanish_point = cv::Point2f(0,0);
    cv::Point2f vanish_point_sum=cv::Point2f(0,0);
    cv::Point2f mean_vanish_point;
    float v_count = 0;
    while (!EndProcessing)
    {
        m_inputVideoHandler >> image_frame;
        if(image_frame.empty())
        {
            EndProcessing = true;
            break;
        }

        if(vanish_point==cv::Point2f(0,0))
        {
            vanish_point = cv::Point2f(image_frame.cols/2.f,image_frame.rows/2.f);
        }
        cv::cvtColor(image_frame,gray_img,CV_BGR2GRAY);

        cv::Point2f next_vanish;
        this->EstimateVanishPoint(next_vanish, gray_img, vanish_point);
        vanish_point = next_vanish;
        vanish_point_sum += vanish_point;
        v_count++;
        if(v_count!=0)
        {
            mean_vanish_point = vanish_point_sum / v_count;
        }


        if(show_rlt)
        {
            cv::circle(image_frame,vanish_point,4,cv::Scalar(0,255,0),-1);
            cv::circle(image_frame,mean_vanish_point,4,cv::Scalar(0,0,255),-1);
            cv::imshow("frame",image_frame);

            char character_press = cv::waitKey(1);
            switch (character_press)
            {
            case 'q':
                EndProcessing = true;
                break;
            case 'c':
                cv::waitKey(-1);
                break;
            default:
                break;
            }
        }
    }
    m_extrinsic_vanish_point = mean_vanish_point;
    cv::Point2f vanish_uv;
    m_intrinsic_parameter.PixelPoint2UVPlanePoint(vanish_uv, mean_vanish_point);
    m_extinsic_parameter.camera_pitch_angle = atan(vanish_uv.y) / CV_PI * 180.f;
    m_extinsic_parameter.camera_yaw_angle = atan(vanish_uv.x) / CV_PI * 180.f;
    return true;
}

bool OfflineCalibrater::SaveExtrinsicParameters()
{
    std::cout << "save extrinsic paramters to file : "<<m_extrinsic_save_path<<std::endl;

    m_intrinsic_parameter.IntrinsicMatrix = (cv::Mat_<float>(3, 3) <<
        m_intrinsic_parameter.fx, 0, m_intrinsic_parameter.cx
        , 0, m_intrinsic_parameter.fy, m_intrinsic_parameter.cy
        , 0, 0, 1.0);

    m_intrinsic_parameter.distCoeffs = (cv::Mat_<float>(1, 5) <<
        m_intrinsic_parameter.k1, m_intrinsic_parameter.k2
        , m_intrinsic_parameter.p1, m_intrinsic_parameter.p2,
        m_intrinsic_parameter.k3);


    const char* xmlPath = m_extrinsic_save_path.c_str();

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
    internal_item->SetAttribute("Param","DisCoef_p2");
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
    if(m_extinsic_parameter.camera_pitch_angle>=0)
    {
        external_item->SetAttribute("value",m_extinsic_parameter.camera_pitch_angle);
        external_item->InsertFirstChild(doc.NewComment("extinsic_parameter.camera_pitch_angle"));
        external_item->SetAttribute("remark","up");
    }
    else {
        external_item->SetAttribute("value",-m_extinsic_parameter.camera_pitch_angle);
        external_item->InsertFirstChild(doc.NewComment("extinsic_parameter.camera_pitch_angle"));
        external_item->SetAttribute("remark","down");
    }
    external_items->InsertEndChild(external_item);

    external_item = doc.NewElement("item");
    external_item->SetAttribute("Param","yaw");
    if(m_extinsic_parameter.camera_yaw_angle >= 0)
    {
        external_item->SetAttribute("value",m_extinsic_parameter.camera_yaw_angle);
        external_item->InsertFirstChild(doc.NewComment("extinsic_parameter.camera_yaw_angle"));
        external_item->SetAttribute("remark","right");
    }
    else
    {
        external_item->SetAttribute("value",-m_extinsic_parameter.camera_yaw_angle);
        external_item->InsertFirstChild(doc.NewComment("extinsic_parameter.camera_yaw_angle"));
        external_item->SetAttribute("remark","left");
    }
    external_items->InsertEndChild(external_item);

    external_item = doc.NewElement("item");
    external_item->SetAttribute("Param","roll");
    if(m_extinsic_parameter.camera_roll_angle >= 0)
    {
        external_item->SetAttribute("value",m_extinsic_parameter.camera_roll_angle);
        external_item->InsertFirstChild(doc.NewComment("extinsic_parameter.camera_roll_angle"));
        external_item->SetAttribute("remark","clockwise");
    }
    else {
        external_item->SetAttribute("value",-m_extinsic_parameter.camera_roll_angle);
        external_item->InsertFirstChild(doc.NewComment("extinsic_parameter.camera_roll_angle"));
        external_item->SetAttribute("remark","counterclockwise");
    }

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
    if(m_workshop_parameter.Dc >= 0)
    {
        external_item->SetAttribute("value",m_workshop_parameter.Dc);
        external_item->InsertFirstChild(doc.NewComment("相机与车辆中心线的偏离"));
        external_item->SetAttribute("unit","mm");
        external_item->SetAttribute("remark","left");
    }
    else {
        external_item->SetAttribute("value",-m_workshop_parameter.Dc);
        external_item->InsertFirstChild(doc.NewComment("相机与车辆中心线的偏离"));
        external_item->SetAttribute("unit","mm");
        external_item->SetAttribute("remark","right");
    }
    external_items->InsertEndChild(external_item);

    doc.SaveFile(xmlPath);
    std::cout <<m_extrinsic_save_path << "extrinsic file is saved: "<<std::endl;
    return true;
}

bool OfflineCalibrater::SaveIntrinsicParameters()
{
    std::cout << "save intrinsic paramters to file : "<<m_intrinsic_save_path<<std::endl;

    m_intrinsic_parameter.IntrinsicMatrix = (cv::Mat_<float>(3, 3) <<
        m_intrinsic_parameter.fx, 0, m_intrinsic_parameter.cx
        , 0, m_intrinsic_parameter.fy, m_intrinsic_parameter.cy
        , 0, 0, 1.0);

    m_intrinsic_parameter.distCoeffs = (cv::Mat_<float>(1, 5) <<
        m_intrinsic_parameter.k1, m_intrinsic_parameter.k2
        , m_intrinsic_parameter.p1, m_intrinsic_parameter.p2,
        m_intrinsic_parameter.k3);


    const char* xmlPath = m_intrinsic_save_path.c_str();

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
    internal_item->SetAttribute("Param","DisCoef_p2");
    internal_item->SetAttribute("value",m_intrinsic_parameter.p2);
    internal_item->InsertFirstChild(doc.NewComment("intrinsic_parameter.p2"));
    internal_items->InsertEndChild(internal_item);

    doc.SaveFile(xmlPath);
    return true;
}

bool OfflineCalibrater::GetFiles(std::vector<std::string> &file_names, std::string path)
{
    DIR *pDir;
    struct dirent * ptr;

    if(!(pDir = opendir(path.c_str()))){
        std::cout<<"Folder doesn't Exist"<<std::endl;
    }

    while ((ptr = readdir(pDir))!=0) {
        if(strcmp(ptr->d_name,".")!=0 && strcmp(ptr->d_name,"..")!=0){
            std::string filename = ptr->d_name;
            std::string ext = filename.substr(filename.rfind('.') + 1,filename.size());
            transform(ext.begin(), ext.end(), ext.begin(), ::tolower);
            if(ext.compare("jpg")==0 || ext.compare("png")==0 || ext.compare("420")==0 || ext.compare("bmp")==0)
            {
                file_names.push_back(path + "/" + ptr->d_name);
            }
        }
    }
    return true;
}

bool OfflineCalibrater::LoadIntrinsicSettingsFile(const std::string &setting_file)
{
    XMLDocument xml_doc;
    std::cout <<"Intrinsic settings file "<< setting_file << " is loading ... "  << std::endl;
    if (xml_doc.LoadFile(setting_file.c_str()) != 0)
    {
        std::cout << "error: can't find " << setting_file << std::endl;
        return false;
    }

    //Global settings
    XMLElement *elem_root = xml_doc.RootElement();
    XMLElement *elem_param = elem_root->FirstChildElement();
    const XMLAttribute *cur_attribute = elem_param->FirstAttribute();
    cur_attribute = cur_attribute->Next();
    std::string if_show = cur_attribute->Value();
    if(if_show=="yes")
    {
        this->m_show_processing = true;
    }
    else {
        this->m_show_processing = false;
    }

    elem_param = elem_param->NextSiblingElement();
    cur_attribute = elem_param->FirstAttribute();
    cur_attribute = cur_attribute->Next();
    std::string save_name = cur_attribute->Value();

    int npos = setting_file.rfind("/");
    std::string param_dir = setting_file.substr(0,npos);

    this->m_intrinsic_save_path = param_dir + "/" + save_name;

    //Chessboard Param
    elem_root = elem_root->NextSiblingElement();
    elem_param = elem_root->FirstChildElement();
    cur_attribute = elem_param->FirstAttribute();
    cur_attribute = cur_attribute->Next();
    this->m_intrinsic_chess_cell_size.height = float(atof(cur_attribute->Value()));

    elem_param = elem_param->NextSiblingElement();
    cur_attribute = elem_param->FirstAttribute();
    cur_attribute = cur_attribute->Next();
    this->m_intrinsic_chess_cell_size.width = float(atof(cur_attribute->Value()));

    elem_param = elem_param->NextSiblingElement();
    cur_attribute = elem_param->FirstAttribute();
    cur_attribute = cur_attribute->Next();
    this->m_intrinsic_corners_size.height = float(atof(cur_attribute->Value()));

    elem_param = elem_param->NextSiblingElement();
    cur_attribute = elem_param->FirstAttribute();
    cur_attribute = cur_attribute->Next();
    this->m_intrinsic_corners_size.width = float(atof(cur_attribute->Value()));

    std::cout<<"\t intrinsic image files : "<<std::endl;
    std::string int_dir;
    GetDerectoryName(int_dir, setting_file);
    GetFiles(m_intrinsic_image_files,int_dir);

    for(int i=0;i<m_intrinsic_image_files.size();i++)
    {
        std::cout<<"\t\t"<<m_intrinsic_image_files[i]<<std::endl;
    }

    std::cout<<"\t show images : "<<if_show<<std::endl;
    std::cout<<"\t chess_cell_size.height : "<<m_intrinsic_chess_cell_size.height<<std::endl;
    std::cout<<"\t chess_cell_size.width : "<<m_intrinsic_chess_cell_size.width<<std::endl;
    std::cout<<"\t corners_size.height : "<<m_intrinsic_corners_size.height<<std::endl;
    std::cout<<"\t corners_size.width : "<<m_intrinsic_corners_size.width<<std::endl;
    std::cout<<"\t intrinsic_save_path : "<<m_intrinsic_save_path<<std::endl;



    std::cout << setting_file << " is loaded" << std::endl;

    return true;
}

bool OfflineCalibrater::LoadExtrinsicSettingsFile(const std::string &config_file)
{
    XMLDocument xml_doc;
    std::cout <<"Extrinsic settings file "<< config_file << " is loading ... "  << std::endl;
    if (xml_doc.LoadFile(config_file.c_str()) != 0)
    {
        std::cout << "error: can't find " << config_file << std::endl;
        return false;
    }

    //Global settings
    int npos = config_file.rfind("/");
    std::string param_dir = config_file.substr(0,npos);

    XMLElement *elem_root = xml_doc.RootElement();
    XMLElement *elem_param = elem_root->FirstChildElement();
    const XMLAttribute *cur_attribute = elem_param->FirstAttribute();
    cur_attribute = cur_attribute->Next();
    std::string if_show = cur_attribute->Value();
    if(if_show=="yes")
    {
        this->m_show_processing = true;
    }
    else {
        this->m_show_processing = false;
    }

    elem_param = elem_param->NextSiblingElement();
    cur_attribute = elem_param->FirstAttribute();
    cur_attribute = cur_attribute->Next();
    m_extrinsic_video_file = param_dir + "/" + cur_attribute->Value();

    elem_param = elem_param->NextSiblingElement();
    cur_attribute = elem_param->FirstAttribute();
    cur_attribute = cur_attribute->Next();
    m_extrinsic_based_intrinsic_file = param_dir + "/" + cur_attribute->Value();

    elem_param = elem_param->NextSiblingElement();
    cur_attribute = elem_param->FirstAttribute();
    cur_attribute = cur_attribute->Next();
    m_extrinsic_save_path = param_dir + "/" + cur_attribute->Value();

    //Workshop Param
    elem_root = elem_root->NextSiblingElement();
    elem_param = elem_root->FirstChildElement();
    cur_attribute = elem_param->FirstAttribute();
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


    std::cout<<"\t show images : "<<if_show<<std::endl;
    std::cout<<"\t m_extrinsic_video_filec : "<<m_extrinsic_video_file<<std::endl;
    std::cout<<"\t m_intrinsic_file_path : "<<m_extrinsic_based_intrinsic_file<<std::endl;
    std::cout<<"\t m_extrinsic_save_path : "<<m_extrinsic_save_path<<std::endl;

    std::cout<<"\t workshop_parameter.Xc : "<<m_workshop_parameter.Xc<<std::endl;
    std::cout<<"\t workshop_parameter.Dc : "<<m_workshop_parameter.Dc<<std::endl;
    std::cout<<"\t workshop_parameter.Zc : "<<m_workshop_parameter.Zc<<std::endl;

    m_extinsic_parameter.camera_world_x = -m_workshop_parameter.Dc / 1000;
    m_extinsic_parameter.camera_world_y = -m_workshop_parameter.Zc / 1000;
    m_extinsic_parameter.camera_world_z = m_workshop_parameter.Xc / 1000;
    m_extinsic_parameter.camera_pitch_angle=0;
    m_extinsic_parameter.camera_roll_angle=0;
    m_extinsic_parameter.camera_yaw_angle=0;    

    std::cout<<"\t extrinsic image files : "<<std::endl;
    std::string int_dir;
    GetDerectoryName(int_dir, config_file);
    GetFiles(m_extrinsic_image_files,int_dir);
    std::sort(m_extrinsic_image_files.begin(), m_extrinsic_image_files.end(), Alphanum_less);

    for(int i=0;i<m_extrinsic_image_files.size();i++)
    {
        std::cout<<"\t\t"<<m_extrinsic_image_files[i]<<std::endl;
    }

    std::cout << config_file << " is loaded" << std::endl << std::endl;

//    cv::Mat source = cv::Mat::ones(720,1280,CV_8UC1) * 128;
//    cv::cvtColor(source,source,CV_GRAY2BGR);
//    cv::Rect mask = cv::Rect(250,100,c_image.cols,c_image.rows);
//    c_image.copyTo(source(mask));
//    cv::imwrite("source.jpg",source);
    return true;
}

bool OfflineCalibrater::LoadIntrinsicParametersFile(const std::string &intrinsic_parameters_file)
{
    XMLDocument xml_doc;
    std::cout << intrinsic_parameters_file << " is loading... "  << std::endl;
    if (xml_doc.LoadFile(intrinsic_parameters_file.c_str()) != 0)
    {
        std::cout << "error: can't find " << intrinsic_parameters_file << std::endl;
        return false;
    }
    else
    {
        std::cout << intrinsic_parameters_file << " is loaded" << std::endl;
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

    std::cout<<"\t intrinsic_parameter.fx : "<<m_intrinsic_parameter.fx<<std::endl;
    std::cout<<"\t intrinsic_parameter.fy : "<<m_intrinsic_parameter.fy<<std::endl;
    std::cout<<"\t intrinsic_parameter.cx : "<<m_intrinsic_parameter.cx<<std::endl;
    std::cout<<"\t intrinsic_parameter.cy : "<<m_intrinsic_parameter.cy<<std::endl;
    std::cout<<"\t intrinsic_parameter.k1 : "<<m_intrinsic_parameter.k1<<std::endl;
    std::cout<<"\t intrinsic_parameter.k2 : "<<m_intrinsic_parameter.k2<<std::endl;
    std::cout<<"\t intrinsic_parameter.k3 : "<<m_intrinsic_parameter.k3<<std::endl;
    std::cout<<"\t intrinsic_parameter.p1 : "<<m_intrinsic_parameter.p1<<std::endl;
    std::cout<<"\t intrinsic_parameter.p2 : "<<m_intrinsic_parameter.p2<<std::endl;

    m_intrinsic_parameter.Init(m_intrinsic_parameter.fx
                               ,m_intrinsic_parameter.fy
                               ,m_intrinsic_parameter.cx
                               ,m_intrinsic_parameter.cy
                               ,m_intrinsic_parameter.k1
                               ,m_intrinsic_parameter.k2
                               ,m_intrinsic_parameter.k3
                               ,m_intrinsic_parameter.p1
                               ,m_intrinsic_parameter.p2);

    return true;
}

bool OfflineCalibrater::LoadCameraParametersFile(const std::string &camera_parameters_file)
{
    XMLDocument xml_doc;
    std::cout << camera_parameters_file << " is loading... "  << std::endl;
    if (xml_doc.LoadFile(camera_parameters_file.c_str()) != 0)
    {
        std::cout << "error: can't find " << camera_parameters_file << std::endl;
        return false;
    }
    else
    {
        std::cout << camera_parameters_file << " is loaded" << std::endl;
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
    m_extinsic_parameter.camera_pitch_angle = float(atof(cur_attribute->Value()));
    cur_attribute = cur_attribute->Next();
    temp_string = cur_attribute->Value();
    if (temp_string == "down")
    {
        m_extinsic_parameter.camera_pitch_angle = -m_extinsic_parameter.camera_pitch_angle;
    }

    elem_param = elem_param->NextSiblingElement();
    cur_attribute = elem_param->FirstAttribute();
    cur_attribute = cur_attribute->Next();
    m_extinsic_parameter.camera_yaw_angle = float(atof(cur_attribute->Value()));
    cur_attribute = cur_attribute->Next();
    temp_string = cur_attribute->Value();
    if (temp_string == "left")
    {
        m_extinsic_parameter.camera_yaw_angle = -m_extinsic_parameter.camera_yaw_angle;
    }

    elem_param = elem_param->NextSiblingElement();
    cur_attribute = elem_param->FirstAttribute();
    cur_attribute = cur_attribute->Next();
    m_extinsic_parameter.camera_roll_angle = float(atof(cur_attribute->Value()));
    cur_attribute = cur_attribute->Next();
    temp_string = cur_attribute->Value();
    if (temp_string == "counterclockwise")
    {
        m_extinsic_parameter.camera_roll_angle = -m_extinsic_parameter.camera_roll_angle;
    }

    elem_param = elem_param->NextSiblingElement();
    cur_attribute = elem_param->FirstAttribute();
    cur_attribute = cur_attribute->Next();
    m_extinsic_parameter.camera_height = float(atof(cur_attribute->Value()));
    m_extinsic_parameter.camera_height = std::abs(m_extinsic_parameter.camera_height) / 1000.0f;
    m_extinsic_parameter.camera_world_y = -m_extinsic_parameter.camera_height;

    elem_param = elem_param->NextSiblingElement();
    cur_attribute = elem_param->FirstAttribute();
    cur_attribute = cur_attribute->Next();
    m_extinsic_parameter.camera_world_z = float(atof(cur_attribute->Value()));
    m_extinsic_parameter.camera_world_z = -std::abs(m_extinsic_parameter.camera_world_z) / 1000.0f;

    elem_param = elem_param->NextSiblingElement();
    cur_attribute = elem_param->FirstAttribute();
    cur_attribute = cur_attribute->Next();
    m_extinsic_parameter.camera_world_x = float(atof(cur_attribute->Value()));
    m_extinsic_parameter.camera_world_x = -m_extinsic_parameter.camera_world_x / 1000.0f;

    m_intrinsic_parameter.IntrinsicMatrix = (cv::Mat_<float>(3, 3) <<
        m_intrinsic_parameter.fx, 0, m_intrinsic_parameter.cx
        , 0, m_intrinsic_parameter.fy, m_intrinsic_parameter.cy
        , 0, 0, 1.0);
    m_intrinsic_parameter.distCoeffs = (cv::Mat_<float>(1, 5) <<
        m_intrinsic_parameter.k1, m_intrinsic_parameter.k2
        , m_intrinsic_parameter.p1, m_intrinsic_parameter.p2,
        m_intrinsic_parameter.k3);

    const float PI = 3.1415926f;
    float arc_pitch_angle = m_extinsic_parameter.camera_pitch_angle * PI/180.0f;
    float arc_yaw_angle = m_extinsic_parameter.camera_yaw_angle * PI / 180.0f;
    float arc_roll_angle = m_extinsic_parameter.camera_roll_angle * PI / 180.0f;

    cv::Mat pitchMatrix = (cv::Mat_<float>(3, 3) << 1, 0, 0
        , 0, std::cos(arc_pitch_angle), std::sin(arc_pitch_angle)
        , 0, -std::sin(arc_pitch_angle), std::cos(arc_pitch_angle));
    cv::Mat yaw_Matrix = (cv::Mat_<float>(3, 3) << std::cos(arc_yaw_angle), 0, -std::sin(arc_yaw_angle)
        , 0, 1, 0
        , std::sin(arc_yaw_angle), 0, std::cos(arc_yaw_angle));
    cv::Mat roll_Matrix = (cv::Mat_<float>(3, 3) << std::cos(arc_roll_angle), std::sin(arc_roll_angle), 0
        , -std::sin(arc_roll_angle), std::cos(arc_roll_angle), 0
        , 0, 0, 1);

    m_extinsic_parameter.rotateMatrix = roll_Matrix*yaw_Matrix*pitchMatrix;
    m_extinsic_parameter.rotateMatrix_inv = m_extinsic_parameter.rotateMatrix.inv();
    m_extinsic_parameter.cameraLocationInWorld = (cv::Mat_<float>(3, 1) <<
        m_extinsic_parameter.camera_world_x, -m_extinsic_parameter.camera_height, m_extinsic_parameter.camera_world_z);
    m_extinsic_parameter.ground_plane_world_y = 0;

    return true;
}

bool OfflineCalibrater::GetDerectoryName(std::string &strResult, std::string strPath)
{
    int nPos = strPath.rfind('/');
    strResult = strPath;
    if (nPos != -1)
    {
        strResult = strPath.substr(0, nPos);
    }
    return true;
}

OfflineCalibrater::OfflineCalibrater()
{
    m_show_processing = true;
}

bool HRYTCalibrater::RangeTesting(const std::string &camera_param_file)
{
    LoadCameraParametersFile(camera_param_file);

    std::cout<<"\t image for range testing files : "<<std::endl;
    std::string int_dir;
    GetDerectoryName(int_dir, camera_param_file);
    GetFiles(m_range_test_image_files,int_dir);
    std::sort(m_range_test_image_files.begin(), m_range_test_image_files.end(), Alphanum_less);

    for(int i=0;i<m_range_test_image_files.size();i++)
    {
        std::cout<<"\t\t"<<m_range_test_image_files[i]<<std::endl;
    }

    cv::Mat image_frame;
    cv::Mat gray_img;

    std::vector<cv::Point2f> ground_points;
    for(int i=1;i<11;i++)
    {
        ground_points.push_back(cv::Point2f(10 * i, 0));
    }
    ground_points.push_back(cv::Point2f(1000,0));

    cv::Point2f camera_center = cv::Point2f(m_intrinsic_parameter.cx,m_intrinsic_parameter.cy);
    cv::Point2f vanish_point = cv::Point2f(10000,0);
    this->HRYTGroundPointToPixelPointOffline(vanish_point,vanish_point);

    std::vector<cv::Point2f> grid_points;
    int grid_cols = 5;
    int grid_rows = 4;
    for(int i=1;i<=grid_rows;i++)
    {
        for(int j=0;j<grid_cols;j++)
        {
            grid_points.push_back(cv::Point2f((j+0.5) * 1280 / grid_cols,vanish_point.y +  i * (720 - vanish_point.y) / grid_rows));
        }
    }

    for ( int i=0; i<m_range_test_image_files.size(); i++ )
    {
        std::string ext = m_range_test_image_files[i].substr(m_range_test_image_files[i].rfind('.') + 1,m_range_test_image_files[i].size());
        transform(ext.begin(), ext.end(), ext.begin(), ::tolower);
        if(ext.compare("420")==0)
        {
            readYUV420File(image_frame,m_range_test_image_files[i]);
        }
        else
        {
            image_frame = cv::imread(m_range_test_image_files[i]);
        }
        if ( image_frame.channels() == 3 )
            cv::cvtColor( image_frame, gray_img, CV_BGR2GRAY );
        else
            image_frame.copyTo( gray_img );

        if(image_frame.empty())
        {
            continue;
        }

        cv::circle(image_frame,vanish_point,4,cv::Scalar(0,255,0),-1);
        cv::circle(image_frame,camera_center,4,cv::Scalar(255,0,0),-1);

        cv::Point2f line_center,line_left,line_right;
        for(int i=0;i<ground_points.size();i++)
        {
//            this->GroundCoordinateToPixelPoint(line_left,ground_points[i] - cv::Point2f(8,0));
//            this->GroundCoordinateToPixelPoint(line_right,ground_points[i] + cv::Point2f(8,0));
            this->HRYTGroundPointToPixelPointOffline(line_right,ground_points[i] - cv::Point2f(0,8));
            this->HRYTGroundPointToPixelPointOffline(line_left,ground_points[i] + cv::Point2f(0,8));

//            line_left = line_center;
//            line_left.x = 0;
//            line_right = line_center;
//            line_right.x = 1279;
            cv::line(image_frame,line_left,line_right,cv::Scalar(0,0,255));

            std::stringstream sstream;
            sstream << ground_points[i].x;
            std::string dis = sstream.str();
            cv::putText(image_frame,dis + "m",line_left,1,1,cv::Scalar(0,255,0));
        }

        cv::Point2f grid_temp;
        for(int i=0;i<grid_points.size();i++)
        {
            cv::circle(image_frame,grid_points[i],3,cv::Scalar(0,255,255),-1);
//            this->PixelPointToGroundCoordinate(grid_temp,grid_points[i]);
            this->PixelPointToHRYTGroundPointOffline(grid_temp,grid_points[i]);
            std::stringstream sstream;
            sstream << grid_temp;
            std::string loc = sstream.str();
            cv::putText(image_frame,loc,grid_points[i] - cv::Point2f(100,0),1,1,cv::Scalar(0,255,0));
        }
        cv::imshow("frame",image_frame);
        cv::imwrite(m_range_test_image_files[i] + ".png",image_frame);
        cv::waitKey(-1);
    }
    return true;
}

bool HRYTCalibrater::PixelPointToHRYTGroundPointOffline(cv::Point2f &hryt_ground_point, cv::Point2f pixel_point) const
{
    cv::Point2f uv_point;
    m_intrinsic_parameter.PixelPoint2UVPlanePoint(uv_point, pixel_point);
    cv::Point3f world_point;
    m_extinsic_parameter.UVPoint2WorldCoordinate(world_point,uv_point,0);
    cv::Point3f hryt_3d_point;
    WorldCoordinate2HRYTCoordinate(hryt_3d_point,world_point);
    hryt_ground_point.x = hryt_3d_point.x;
    hryt_ground_point.y = hryt_3d_point.y;
    return true;
}

bool HRYTCalibrater::HRYTGroundPointToPixelPointOffline(cv::Point2f &pixel_point, cv::Point2f hryt_ground_point) const
{
    cv::Point3f hryt_3d_point;
    hryt_3d_point.x = hryt_ground_point.x;
    hryt_3d_point.y = hryt_ground_point.y;
    hryt_3d_point.z = 0;
    cv::Point3f world_point;
    HRYTCoordinate2WorldCoordinate(world_point,hryt_3d_point);
    cv::Point2f uv_point;
    m_extinsic_parameter.WorldCoordinate2UVPoint(uv_point,world_point);
    m_intrinsic_parameter.UVPlanePoint2PixelPoint(pixel_point,uv_point);
    return true;
}

bool HRYTCalibrater::PixelPointToHRYT3DPointOffline(cv::Point3f &hryt_3d_point, cv::Point2f pixel_point, float point_zloc_hryt) const
{
    cv::Point2f uv_point;
    m_intrinsic_parameter.PixelPoint2UVPlanePoint(uv_point, pixel_point);
    cv::Point3f world_point;
    m_extinsic_parameter.UVPoint2WorldCoordinate(world_point,uv_point,point_zloc_hryt);
    WorldCoordinate2HRYTCoordinate(hryt_3d_point,world_point);
    return true;
}

bool HRYTCalibrater::HRYT3DPointToPixelPointOffline(cv::Point2f &pixel_point, cv::Point3f hryt_3d_point) const
{
    cv::Point3f world_point;
    HRYTCoordinate2WorldCoordinate(world_point,hryt_3d_point);
    cv::Point2f uv_point;
    m_extinsic_parameter.WorldCoordinate2UVPoint(uv_point,world_point);
    m_intrinsic_parameter.UVPlanePoint2PixelPoint(pixel_point,uv_point);
    return true;
}

bool HRYTCalibrater::PixelPedestrainBBoxToHRYTHeadPointOffline(cv::Point3f &hryt_head_point, cv::Rect bounding_box) const
{
    cv::Point2f top_center_pixel = cv::Point2f(bounding_box.x + bounding_box.width / 2, bounding_box.y);
    cv::Point2f bottom_center_pixel = cv::Point2f(bounding_box.x + bounding_box.width / 2, bounding_box.y + bounding_box.height);
    cv::Point2f top_center_uv, bottom_center_uv;
    m_intrinsic_parameter.PixelPoint2UVPlanePoint(top_center_uv, top_center_pixel);
    m_intrinsic_parameter.PixelPoint2UVPlanePoint(bottom_center_uv, bottom_center_pixel);
    cv::Point3f head_center_world;
    m_extinsic_parameter.UVPoint2WorldCoordinate(head_center_world,bottom_center_uv,0);
    float scale = head_center_world.z - m_extinsic_parameter.camera_world_z;
    float height = std::abs(top_center_uv.y - bottom_center_uv.y) * scale;
    head_center_world.y = 0 - height;
    WorldCoordinate2HRYTCoordinate(hryt_head_point,head_center_world);
    return true;
}

void HRYTCalibrater::HRYTCoordinate2WorldCoordinate(cv::Point3f &world_coordinate, cv::Point3f HRYT_coordinate)
{
    world_coordinate.x = -HRYT_coordinate.y;
    world_coordinate.y = -HRYT_coordinate.z;
    world_coordinate.z = HRYT_coordinate.x;
}

void HRYTCalibrater::WorldCoordinate2HRYTCoordinate(cv::Point3f &HRYT_coordinate, cv::Point3f world_coordinate)
{
    HRYT_coordinate.x = world_coordinate.z;
    HRYT_coordinate.y = -world_coordinate.x;
    HRYT_coordinate.z = -world_coordinate.y;
}
