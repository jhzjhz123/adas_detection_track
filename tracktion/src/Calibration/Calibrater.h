#pragma once

#include<sys/types.h>
#include<sys/stat.h>
#include<dirent.h>
#include<iostream>
#include<fstream>
#include<stdlib.h>
#include<vector>
#include<algorithm>
#include<opencv2/opencv.hpp>
#include "tinyxml2.h"

using namespace tinyxml2;

/***********************************************************************
 * @class Calibrater
 *
 * @Right-handed : ↑ ↓ ←  →
 *	Pixel plane	:           O : left_up         x : →		y : ↓
 *	Image plane	:           O : left_up         u : →		v : ↓
 *	Camera plane:           O : center          Xc: →		Yc: ↓		Zc: Froward     Right-handed
 *	World coordinate(m):    O : car_head_ground Xw: →		Yw: ↓		Zw: Forward     Right-handed
 *	Ground plane:(m)        O : car_head_ground Xg: →		Yg: ↑ (Forward)
 *	Eula order:             (Xw,Yw,Zw) -> -T  -> *Rx(pitch)  -> *Ry(yaw)  -> *Rz(roll)  -> (Xc,Yc,Zc)
 *	HRYT coordinate(m):     O : car_head_ground Xh: Forward         Yh: ←		Zh: ↑           Right-handed
 * @author xiongwc
 * @date 2019/10/29
 ************************************************************************/
struct IntrinsicParameter
{
    float p1;
    float p2;
    float k1;
    float k2;
    float k3;
    float fx;
    float fy;
    float cx;
    float cy;
    cv::Mat_<float> IntrinsicMatrix;
    cv::Mat_<float> distCoeffs;
    cv::Mat_<float> mtx_empty = cv::Mat_<float>::zeros(3, 1);
    bool Init(float fx,float fy,float cx,float cy,float k1,float k2,float k3,float p1,float p2);
    bool Init(cv::Mat cameraMatrix, cv::Mat dist_coeffs);
    void UVPlanePoint2PixelPoint(cv::Point2f &pixel_point, cv::Point2f uv_point) const;
    void PixelPoint2UVPlanePoint(cv::Point2f &uv_point, cv::Point2f pixel_point) const;
};

struct ExtinsicParameter
{
    float camera_pitch_angle;
    float camera_yaw_angle;
    float camera_roll_angle;
    float camera_height;
    float camera_world_x;
    float camera_world_y;
    float camera_world_z;
    float ground_plane_world_y;

    cv::Mat_<float> rotateMatrix;// Rz*Ry*Rx
    cv::Mat_<float> rotateMatrix_inv;// Rx'*Ry'*Rz'
    cv::Mat_<float> cameraLocationInWorld;//cv::Point3f(camera_world_x,camera_world_y,camera_world_z)
    cv::Mat_<float> translateMatrix;// = cv::Mat_<float>(3, 3);
    cv::Mat_<float> pMatrix;// = cv::Mat_<float>(3, 4);

    bool InitByWorldLocation(float camera_world_x,float camera_world_y,float camera_world_z,float pitch_angle,float yaw_angle,float roll_angle);

    void UVPoint2GroundCoordinate(cv::Point2f &ground_point, cv::Point2f uv_point) const;
    void GroundCoordinate2UVPoint(cv::Point2f &uv_point, cv::Point2f ground_point) const;

    void UVPoint2WorldCoordinate(cv::Point3f &world_coordinate, cv::Point2f uv_point, float height_on_ground) const;
    void WorldCoordinate2UVPoint(cv::Point2f &uv_point, cv::Point3f world_coordinate) const;
};

struct ChessboardParameter
{
    float Ds;//单个棋盘格的尺寸
    float D_width;//左、右棋盘的宽度
    float D_height;//左、右棋盘的高度
};
struct WorkShopParameter
{
    float Dc;//摄像头中心与车辆中心线在y方向上（左右）的偏移
    float Xc;//摄像机相距车头的距离
    float Zc;//摄像头距离地面的高度
};

class Calibrater
{
public:
    Calibrater() {}
    void CrossPointOfTwoLanes(cv::Point2f &cross_point, cv::Point2f lineA1, cv::Point2f lineA2, cv::Point2f lineB1, cv::Point2f lineB2);
};

class OnlineCalibrater
        :public Calibrater
{
public:
    OnlineCalibrater();
    ExtinsicParameter m_offline_extinsic_parameter;
    ExtinsicParameter m_online_extinsic_parameter;
    IntrinsicParameter m_intrinsic_parameter;

    bool load_offline_parameters(const std::string & offline_camera_parameters_file);

    void PixelPointToGroundCoordinateOffline(cv::Point2f & ground_point, cv::Point2f pixel_point) const;
    void GroundCoordinateToPixelPointOffline(cv::Point2f & pixel_point, cv::Point2f ground_point) const;

    void PixelPointToWorldCoordinate(cv::Point3f & world_coordinate, cv::Point2f pixel_point, float height_on_ground) const;
    void WorldCoordinateToPixelPoint(cv::Point2f & pixel_point, cv::Point3f world_coordinate) const;

    void PixelPointToGroundCoordinate(cv::Point2f & ground_coordinate, cv::Point2f pixel_point) const;
    void GroundCoordinateToPixelPoint(cv::Point2f & pixel_point, cv::Point2f ground_point) const;
};

class OfflineCalibrater
        :public Calibrater
{
public:
    OfflineCalibrater();

    ExtinsicParameter m_extinsic_parameter;
    IntrinsicParameter m_intrinsic_parameter;

    bool IntrinsicCalibration(const std::string& setting_file);
    bool ExtrinsicCalibration(const std::string& setting_file);
    bool RangeTesting(const std::string& camera_param_file);

    bool LoadCameraParametersFile(const std::string &camera_parameters_file);
    void PixelPointToGroundCoordinate(cv::Point2f & ground_point, cv::Point2f pixel_point) const;
    void GroundCoordinateToPixelPoint(cv::Point2f & pixel_point, cv::Point2f ground_point) const;
protected:
    bool m_show_processing;
    cv::Size m_intrinsic_corners_size;
    cv::Size m_intrinsic_chess_cell_size;//mm
    std::vector<std::string> m_intrinsic_image_files;    
    std::vector<std::string> m_extrinsic_image_files;
    std::vector<std::string> m_range_test_image_files;

    std::string m_intrinsic_save_path;

    std::string m_extrinsic_video_file;
    std::string m_extrinsic_chessboard_image_file;
    std::string m_extrinsic_save_path;
    std::string m_extrinsic_based_intrinsic_file;
    cv::Point2f m_extrinsic_vanish_point;

    bool GetFiles(std::vector<std::string> &file_names, std::string path);
    bool LoadIntrinsicSettingsFile(const std::string &setting_file);
    bool GetDerectoryName(std::string &strResult, std::string strPath);    
    bool SaveIntrinsicParameters();
    bool LoadExtrinsicSettingsFile(const std::string &config_file);
    bool LoadIntrinsicParametersFile(const std::string &intrinsic_parameters_file);
    bool CalibrationByVideo(bool show_rlt);
    bool CalibrationByImages(bool show_rlt);
    bool SaveExtrinsicParameters();

    void readYUV420File(cv::Mat &img, const std::string file_name);

    static bool Alphanum_less(std::string left, std::string right);
    static int Alphanum_comp(std::string left, std::string right);
    static bool Alphanum_isdigit(const char c);
private:    
    ChessboardParameter m_chessboard_parameter;
    WorkShopParameter m_workshop_parameter;

    bool EstimateVanishPoint(cv::Point2f &predict_vanish_point, const cv::Mat &frame, cv::Point2f last_vanish_point);
};

class HRYTCalibrater:
        public OfflineCalibrater
{
public:

    bool RangeTesting(const std::string& camera_param_file);

    //像素坐标转华人运通地面坐标
    bool PixelPointToHRYTGroundPointOffline(cv::Point2f & hryt_ground_point,cv::Point2f pixel_point) const;
    //华人运通地面坐标转像素坐标
    bool HRYTGroundPointToPixelPointOffline(cv::Point2f & pixel_point,cv::Point2f hryt_ground_point) const;

    //像素坐标以及像素对应物理空间的Z坐标，转华人运通3D坐标
    bool PixelPointToHRYT3DPointOffline(cv::Point3f & hryt_3d_point,cv::Point2f pixel_point, float point_zloc_hryt) const;
    //华人运通3D坐标转像素坐标
    bool HRYT3DPointToPixelPointOffline(cv::Point2f& pixel_point, cv::Point3f  hryt_3d_point) const;

    //根据行人的像素Bounding box获取华人运通坐标系内行人头顶的3D坐标
    bool PixelPedestrainBBoxToHRYTHeadPointOffline(cv::Point3f & hryt_head_point, cv::Rect bounding_box) const;

    static void HRYTCoordinate2WorldCoordinate(cv::Point3f &world_coordinate,cv::Point3f HRYT_coordinate);
    static void WorldCoordinate2HRYTCoordinate(cv::Point3f &HRYT_coordinate,cv::Point3f world_coordinate);
};
