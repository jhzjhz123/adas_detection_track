#ifndef KALMAN_FILTER_TOOLBOX_H
#define KALMAN_FILTER_TOOLBOX_H

#include <opencv2/video/tracking.hpp>
#include "dataType.h"

using namespace std;
using namespace cv;

class KalmanFilterToolBox
{
public:
    KalmanFilterToolBox(int ndim, double dt, const DETECTBOX &measurement);
    ~KalmanFilterToolBox();
    static const double chi2inv95[10];
    void predict(float errorCoefficient);
    void update(const DETECTBOX &measurement);
    Eigen::Matrix<float, 1, -1> mahalanobis_distance(const std::vector<DETECTBOX> &measurements, bool only_position);
    cv::Mat state;
private:
    float _std_weight_position;
    float _std_weight_velocity;
    cv::Mat _std;
    cv::KalmanFilter *kfilter;

};

#endif
