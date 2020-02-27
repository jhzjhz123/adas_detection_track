#ifndef DISTANCEMEASURE_H
#define DISTANCEMEASURE_H

#include <Eigen/Dense>
#include <opencv2/imgproc.hpp>
#include <opencv2/core/eigen.hpp>
#include <cmath>
#include<iostream>

using namespace std;
using namespace cv;
using namespace Eigen;

#define PI 3.1415926

class DistanceMeasure
{
  public:
    enum TransferType{
      WidthHeight, 
      WidthLength, 
      PixelOnly
    };
    //void ConfigParam(config cfg);
    void ConfigParam(double fx, double fy , double cx, double cy, double k1, double k2, 
    double p1, double p2, double k3, double pitch, double yaw, double roll, double Cam2Chamber, double Cam2Center ,double CamHeight);

    void pixel2world(Rect2f &PixelBoundingBox, Rect2f &WorldBoudingBox, TransferType transfertype);
    Point2f estimationvp();

  public:
    void pixel2worldpixelonly(Rect2f &PixelBoundingBox, Rect2f &WorldBoudingBox);
    void pixel2worldwidthheight(Rect2f &PixelBoundingBox, Rect2f &WorldBoudingBox);
    void pixel2worldwidthlength(Rect2f &PixelBoundingBox, Rect2f &WorldBoudingBox);
    void updatepitchyaw(double pitch, double yaw);
    Point2f CurrentVPPoint;

  private:
    Mat R,cameraMatrix,distort,cameraMatrix_inv, head;
    Matrix3d _intrin, _rota;
    Vector3d _t;
    double _pitch, _yaw, _roll;
    double _fx, _fy, _cx, _cy;
    double _k1,_k2,_p1,_p2,_k3;

    Mat eulerAnglesToRotationMatrix(Vec3f &theta);
    Vector2d pixelonly(double i,double j);
    Vector2d pixel2worldzplane(double i,double j, double z);
    void vpadjust(Point2f pointvpadjust);

};



#endif //DISTANCEMEASURE_H
