#pragma once
#include<iostream>
#include<opencv2/opencv.hpp>
#include "tinyxml2.h"
#include "../Tools/Tools.h"

using namespace tinyxml2;

/***********************************************************************
 * @class Calibrater
 *
 * @Right-handed : ↑ ↓ ←  →
 *	Pixel plane	:           O : left_up         x : →		y : ↓
 *	Image plane	:           O : left_up         u : →		v : ↓
 *	Camera plane:           O : center          Xc: →		Yc: ↓		Zc: Froward     Right-handed
 *	World coordinate(m):    O : veh_head_ground Xw: →		Yw: ↓		Zw: Forward     Right-handed
 *	HRYT coordinate(m):     O : veh_head_ground Xh: Forward	Yh: ←		Zh: ↑           Right-handed
 *	Ground plane:(m)        O : veh_head_ground Xg: →		Yg: ↑ (Forward)
 *	Eula order:             (Xw,Yw,Zw) -> -T  -> *Rx(pitch)  -> *Ry(yaw)  -> *Rz(roll)  -> (Xc,Yc,Zc)
 * @author xiongwc
 * @date 2019/10/29
 ************************************************************************/
class Calibrater
{
public:
    Calibrater();
    ~Calibrater();

    struct ChessboardParameter
    {
        float Ds;//单个棋盘格的高度
        float Db;//左右棋盘中心的横向距离
        float D_width;//左、右棋盘的宽度
        float D_height;//左、右棋盘的高度
        float D_TH;//标定板的高度
        float D_TW;//标定板的宽度
        float Dg;//标定板灰色留边宽度
    };
    struct WorkShopParameter
    {
        float Dc;//摄像头中心与车辆中心线在y方向上（左右）的偏移
        float Xc;//摄像机相距车头的距离
        float Zc;//摄像头距离地面的高度
        float Yb;//车辆中心线延长线距离左/右标定靶中心的距离
        float D_camera;//摄像头距离靶标的距离
        float Zb;//两个标定靶中心距离地面的高度        
    };

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
        bool InitByHRYTLocation(float camera_hryt_x,float camera_hryt_y,float camera_hryt_z,float pitch_angle,float yaw_angle,float roll_angle);

        void UVPoint2GroundCoordinate(cv::Point2f &ground_point, cv::Point2f uv_point) const;
        void GroundCoordinate2UVPoint(cv::Point2f &uv_point, cv::Point2f ground_point) const;

        void UVPoint2WorldCoordinate(cv::Point3f &world_coordinate, cv::Point2f uv_point, float height_on_ground) const;
        void WorldCoordinate2UVPoint(cv::Point2f &uv_point, cv::Point3f world_coordinate) const;

        static void HRYTCoordinate2WorldCoordinate(cv::Point3f &world_coordinate,cv::Point3f HRYT_coordinate);
        static void WorldCoordinate2HRYTCoordinate(cv::Point3f &HRYT_coordinate,cv::Point3f world_coordinate);
    };

    ChessboardParameter m_chessboard_parameter;
    WorkShopParameter m_workshop_parameter;
    IntrinsicParameter m_intrinsic_parameter;
    ExtinsicParameter m_offline_extinsic_parameter;
    ExtinsicParameter m_online_extinsic_parameter;
    std::string m_chessboard_image_path;
    std::string m_chessboard_corners_image_path;
    std::string m_offline_parameters_saving_path;
    std::string m_online_parameters_saving_path;

    std::vector<cv::Point3f> m_real_corners_in_HRYT_System;
    std::vector<cv::Point2f> m_observed_corners_in_image;

    cv::Point3f m_camera_in_HRYT_System;
    cv::Point3f m_camera_in_world;

    cv::Point2f m_vanish_point_of_lanes;
    const int m_vanish_pt_count = 50;

    cv::Mat_<uchar> m_online_captured_image;
    const int IMAGE_WIDTH = 1280;
    const int IMAGE_HEIGHT = 720;

    bool m_configure_loaded = false;

    bool load_HRYT_configration(const std::string & config_file);

    bool load_offline_parameters(const std::string & offline_camera_parameters);

    bool OfflineCalibrationOfHRYT(const std::string &config_file);

    bool OnlineCalibration(const uchar *ptr,int image_width,int image_height);

    int PixelPointToGroundCoordinateOffline(cv::Point2f & ground_point, cv::Point2f pixel_point) const;
    int GroundCoordinateToPixelPointOffline(cv::Point2f & pixel_point, cv::Point2f ground_point) const;

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

    static bool accend_by_x(const cv::Point2f& pt1,const cv::Point2f& pt2);
    bool VanishPointByLaneLines(cv::Point2f &vanish_point, const std::vector<cv::Vec4f> &lane_lines,const cv::Point2f offline_vanish_point);
private:
    bool ChessCornerDetection(std::vector<cv::Point2f> &corners,const cv::Mat &gray_source,int max_grid_size,float max_rotate_angle);
    void CrossPointOfTwoLanes(cv::Point2f &cross_point, cv::Point2f lineA1, cv::Point2f lineA2, cv::Point2f lineB1, cv::Point2f lineB2);
    bool SaveOfflineCameraParameters();
    cv::Mat LoadYUV420FileAsBGR(const std::string &file_path,int image_height,int image_width);
};
