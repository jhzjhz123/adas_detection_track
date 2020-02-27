#include "DistanceMeasure.h"

void DistanceMeasure::updatepitchyaw(double pitch, double yaw){
  _yaw = yaw;
  _pitch = pitch;
  Vec3f theta(_pitch, _yaw, _roll);
  R = eulerAnglesToRotationMatrix(theta);
  head = R.inv()*cameraMatrix.inv();
}

// void DistanceMeasure::ConfigParam(config cfg){
//   _fx = cfg.instrinsics[0];
//   _fy = cfg.instrinsics[1];
//   _cx = cfg.instrinsics[2];
//   _cy = cfg.instrinsics[3];
//   _pitch = cfg.extrinsics[0];
//   _yaw = cfg.extrinsics[1];
//   _roll = cfg.extrinsics[2];
  
//   cameraMatrix = (Mat_<double>(3,3) << _fx,0,_cx,
// 		0,_fy,_cy,
// 		0,0,1);
//   cameraMatrix_inv = cameraMatrix.inv();

//   Vec3f theta(_pitch, _yaw, _roll);
//   R = eulerAnglesToRotationMatrix(theta);
//   head = R.inv()*cameraMatrix.inv();
//   //rotamatrix(_rota);
//   //instrinmatrix(_intrin);
//   _t << cfg.transform[0], cfg.transform[1], cfg.transform[2];
//   _k1 = cfg.distortion[0];
//   _k2 = cfg.distortion[1];
//   _p1 = cfg.distortion[2];
//   _p2 = cfg.distortion[3];
//   _k3 = cfg.distortion[4];
//   distort = (Mat_<double>(5,1)<<_k1,_k2,_p1,_p2,_k3);
//   //memcpy(_distort, cfg.distortion, sizeof(double)*5);
// }

void DistanceMeasure::ConfigParam(double fx, double fy , double cx, double cy, double k1, double k2, 
    double p1, double p2, double k3, double pitch, double yaw, double roll, double Cam2Chamber, double Cam2Center ,double CamHeight){
  _fx = fx;
  _fy = fy;
  _cx = cx;
  _cy = cy;
  _pitch = pitch;
  _yaw = yaw;
  _roll = roll;
  
  cameraMatrix = (Mat_<double>(3,3) << _fx,0,_cx,
		0,_fy,_cy,
		0,0,1);
  cameraMatrix_inv = cameraMatrix.inv();

  Vec3f theta(_pitch, _yaw, _roll);
  R = eulerAnglesToRotationMatrix(theta);
  head = R.inv()*cameraMatrix.inv();
  //rotamatrix(_rota);
  //instrinmatrix(_intrin);
  _t <<Cam2Chamber, Cam2Center, CamHeight;
  _k1 = k1;
  _k2 = k2;
  _p1 = p1;
  _p2 = p2;
  _k3 = k3;
  distort = (Mat_<double>(5,1)<<_k1,_k2,_p1,_p2,_k3);
  //memcpy(_distort, cfg.distortion, sizeof(double)*5);
}

Vector2d DistanceMeasure::pixelonly(double i,double j){
    vector<Point2f> px{Point2f(i,j)};
    Vector2d coord;
    //undistortPoints(px,px,cameraMatrix,distort);
    //px[0].x=cameraMatrix.at<double>(0,0)*px[0].x+cameraMatrix.at<double>(0,1)*px[0].y+cameraMatrix.at<double>(0,2);
    //px[0].y=cameraMatrix.at<double>(1,0)*px[0].x+cameraMatrix.at<double>(1,1)*px[0].y+cameraMatrix.at<double>(1,2);
    Point3d p;
    p.x = px[0].x;
    p.y = px[0].y;
    p.z = 1;
    double x = p.x*head.at<double>(0,0)+p.y*head.at<double>(0,1)+p.z*head.at<double>(0,2);
    double y = p.x*head.at<double>(1,0)+p.y*head.at<double>(1,1)+p.z*head.at<double>(1,2);
    double z = p.x*head.at<double>(2,0)+p.y*head.at<double>(2,1)+p.z*head.at<double>(2,2);

    double zc = _t(2)/y;
    double zx = zc*x;
    double zz = zc*z;
    coord(0) = zx-_t(1);//left negtive right positive
    coord(1) = zz+_t(0);
    return coord;
}

void DistanceMeasure::pixel2worldpixelonly(Rect2f &PixelBoundingBox, Rect2f &WorldBoudingBox){
  double i = PixelBoundingBox.x;double j = PixelBoundingBox.y;
  Vector2d world = this->pixelonly(i,j);
  WorldBoudingBox.x = world(0);WorldBoudingBox.y = world(1);
}

void DistanceMeasure::pixel2worldwidthheight(Rect2f &PixelBoundingBox, Rect2f &WorldBoudingBox){
  double i = PixelBoundingBox.x;double j = PixelBoundingBox.y;
  double w = PixelBoundingBox.width;double h = PixelBoundingBox.height;
  double midi = i+w/2.;double midj = j+h;
  double lefti = i;double leftj = j+h;
  double righti = i+w; double rightj = j+h;
  double topi = i+w/2;double topj = j;
  Vector2d mid = this->pixelonly(midi, midj);
  Vector2d left = this->pixelonly(lefti,leftj);

  Vector2d right = this->pixelonly(righti, rightj);
  double distance = mid(1);
  Vector2d top = this->pixel2worldzplane(topi,topj,distance);

  WorldBoudingBox.x = mid(0);WorldBoudingBox.y = mid(1);
  WorldBoudingBox.width = abs(left(0)-right(0));
  WorldBoudingBox.height = abs(top(1));
  //cout << "world:  " <<WorldBoudingBox << endl;
}

void DistanceMeasure::pixel2worldwidthlength(Rect2f &PixelBoundingBox, Rect2f &WorldBoudingBox){
  double i = PixelBoundingBox.x;double j = PixelBoundingBox.y;
  double w = PixelBoundingBox.width;double h = PixelBoundingBox.height;
  double midi = i+w/2.;double midj = j+h;
  double lefti = i;double leftj = j+h;
  double righti = i+w; double rightj = j+h;
  double topi = i+w/2;double topj = j;
  Vector2d mid = this->pixelonly(midi, midj);
  Vector2d left = this->pixelonly(lefti,leftj);
  Vector2d right = this->pixelonly(righti, rightj);
  Vector2d top = this->pixelonly(topi, topj);

  WorldBoudingBox.x = mid(0);WorldBoudingBox.y = mid(1);
  WorldBoudingBox.width = abs(left(0)-right(0));
  WorldBoudingBox.height = abs(top(1) - mid(1));
}
void DistanceMeasure::pixel2world(Rect2f &PixelBoundingBox, Rect2f &WorldBoudingBox, TransferType transfertype){

   switch (transfertype)
   {
    case PixelOnly:
      this->pixel2worldpixelonly(PixelBoundingBox,WorldBoudingBox);
      break;
    case WidthHeight:
      this->pixel2worldwidthheight(PixelBoundingBox,WorldBoudingBox);
      break;
    case WidthLength:
      this->pixel2worldwidthlength(PixelBoundingBox,WorldBoudingBox);
      break;  
   default:
     break;
   }
}

Vector2d DistanceMeasure::pixel2worldzplane(double i,double j, double z){
    vector<Point2f> px{Point2f(i,j)};
    undistortPoints(px,px,cameraMatrix,distort);
    px[0].x=cameraMatrix.at<double>(0,0)*px[0].x+cameraMatrix.at<double>(0,1)*px[0].y+cameraMatrix.at<double>(0,2);
    px[0].y=cameraMatrix.at<double>(1,0)*px[0].x+cameraMatrix.at<double>(1,1)*px[0].y+cameraMatrix.at<double>(1,2);
    Point3d p;
    p.x = px[0].x;
    p.y = px[0].y;
    p.z = 1;
    double zc = (z+_t(0))/(p.x*head.at<double>(2,0)+p.y*head.at<double>(2,1)+p.z*head.at<double>(2,2));

    double x = p.x*head.at<double>(0,0)+p.y*head.at<double>(0,1)+p.z*head.at<double>(0,2);
    double y = p.x*head.at<double>(1,0)+p.y*head.at<double>(1,1)+p.z*head.at<double>(1,2);

    double zx = zc*x;
    double zy = zc*y;
    //cout << i << "  "<< j << "  " << z << "  "<< zy << endl;
    //cout << p << endl;
    Vector2d coord;
    coord(0) = zx+_t(1);
    coord(1) = _t(2)-zy;
    return coord;
}


Mat DistanceMeasure::eulerAnglesToRotationMatrix(Vec3f &theta)
{
    theta = theta/180*3.1415926;
    Mat R_x = (Mat_<double>(3,3) <<
               1,       0,              0,
               0,       cos(theta[0]),   -sin(theta[0]),
               0,       sin(theta[0]),   cos(theta[0])
               );
 
    Mat R_y = (Mat_<double>(3,3) <<
               cos(theta[1]),    0,      sin(theta[1]),
               0,               1,      0,
               -sin(theta[1]),   0,      cos(theta[1])
               );
 

    Mat R_z = (Mat_<double>(3,3) <<
               cos(theta[2]),    -sin(theta[2]),      0,
               sin(theta[2]),    cos(theta[2]),       0,
               0,               0,                  1);
 

    Mat R = R_z * R_y * R_x;
 
    return R;
}

Point2f DistanceMeasure::estimationvp(){
    Mat re = cameraMatrix*R;
    vector<Point2f> pleft,pright;
    for(int i=10000;i<200000;i=i+10000){
        Point3f p1;
        p1.x = 4000;
        p1.y = _t(2);
        p1.z = i;
        Point2f p1map;
        p1map.x = p1.x*re.at<double>(0,0)+p1.y*re.at<double>(0,1)+p1.z*re.at<double>(0,2);
        p1map.y = p1.x*re.at<double>(1,0)+p1.y*re.at<double>(1,1)+p1.z*re.at<double>(1,2);
        double z = p1.x*re.at<double>(2,0)+p1.y*re.at<double>(2,1)+p1.z*re.at<double>(2,2);
        p1map.x = p1map.x/z;
        p1map.y = p1map.y/z;
        //cout << p1map << endl;
        pleft.push_back(p1map);
    }

    for(int i=10000;i<200000;i=i+10000){
        Point3f p1;
        p1.x = -4000;
        p1.y = _t(2);
        p1.z = i;
        Point2f p1map;
        p1map.x = p1.x*re.at<double>(0,0)+p1.y*re.at<double>(0,1)+p1.z*re.at<double>(0,2);
        p1map.y = p1.x*re.at<double>(1,0)+p1.y*re.at<double>(1,1)+p1.z*re.at<double>(1,2);
        double z = p1.x*re.at<double>(2,0)+p1.y*re.at<double>(2,1)+p1.z*re.at<double>(2,2);
        p1map.x = p1map.x/z;
        p1map.y = p1map.y/z;
        pright.push_back(p1map);
    }


    double x11 = pleft.begin()->x;double x12 = (pleft.end()-1)->x;
    double x21 = pright.begin()->x;double x22 = (pright.end()-1)->x;
    double x0 = (x11*x22 - x21*x12)/(x22-x12 + x11-x21);
    double y11 = pleft.begin()->y;double y12 = (pleft.end()-2)->y;
    double y21 = pright.begin()->y;double y22 = (pright.end()-2)->y;
    double y0 = (y11*y22 - y21*y12)/(y22-y12 + y11-y21);

    //cout << (y22-y12 + y11-y21) << endl;

    Point2f p(x0,y0);

    //std::cout << "Vanished Points:" << p << endl;
    CurrentVPPoint = p;
    return p;
}

void DistanceMeasure::vpadjust(Point2f pointvpadjust){
    for (int i = 0; i < 4; i++)
    {
        // cout << CurrentVPPoint << endl;
        // cout << pointvpadjust << endl;
        Point2f ad1 = CurrentVPPoint-pointvpadjust;
        double alfa1 = tan(_pitch/180*3.1415926);
        double belta1 = ad1.y/_fy;
        double eta1 = alfa1+belta1;
        // cout << "pitch: " << atan(eta1)*180/3.1415926 << endl;
        _pitch = atan(eta1)*180/3.1415926;

        double alfa2 = tan(_yaw/180*3.1415296);
        double belta2 = ad1.x / _fx;
        double eta2 = alfa2-belta2;

        // cout << "yaw: " << atan(eta2)*180/3.1415926 << endl;
        _yaw = atan(eta2)*180/3.1415926;

        Vec3f theta1(_pitch, _yaw, _roll);
        R = eulerAnglesToRotationMatrix(theta1);
        head = R.inv()*cameraMatrix.inv();

        CurrentVPPoint = estimationvp();
    }
}