#include "KalmanFilterToolBox.h"
#include <Eigen/Cholesky>
#include<Eigen/Dense>
#include<opencv2/core/eigen.hpp>
#include "../Tools/Tools.h"

// 具有N个自由度的卡方分布的0.95分位数的表（包含N=1, ..., 9的值）。 取自 MATLAB/Octave 的 chi2inv 函数并用作马氏距离阈值
// 若n个相互独立的随机变量ξ₁，ξ₂，...,ξn ，均服从标准正态分布（也称独立同分布于标准正态分布），则这n个服从标准正态分布的随机变量的平方和构成一新的随机变量，其分布规律称为卡方分布（chi-square distribution）
const double KalmanFilterToolBox::chi2inv95[10] = {
    0,
    3.8415,
    5.9915,
    7.8147,
    9.4877,
    11.070,
    12.592,
    14.067,
    15.507,
    16.919
};

KalmanFilterToolBox::KalmanFilterToolBox(int ndim, double dt, const DETECTBOX &measurement)
{
    // Motion and observation uncertainty are chosen relative to the current state estimate. 
    // These weights control the amount of uncertainty in the model. This is a bit hacky.
    this->_std_weight_position = 1./20;
    this->_std_weight_velocity = 1./160;

    // Create Kalman filter model matrices.
    this->kfilter = new KalmanFilter(8,4,0); // 创建卡尔曼滤波器对象(状态矩阵X维度，测量矩阵Y维度，控制矩阵B维度) 

    setIdentity(this->kfilter->transitionMatrix); // 状态转移矩阵A，根据X=AX+Bu+w (系统误差w~N(0,Q)), 状态矩阵的维数决定了状态转移矩阵A的维数，因为状态矩阵是8×1的(x,y,a,h,vx,vy,va,vh)，则状态转移矩阵A是8×8的。
    for (size_t i = 0; i < 4; i++)
        this->kfilter->transitionMatrix.at<float>(i,i+4) = 0.03;

    setIdentity(this->kfilter->measurementMatrix); // 测量矩阵H，根据Y=HX+v (测量误差v~N(0,R)), 结合状态矩阵，可以确定测量矩阵H的维数, 因为测量值是8x1的，则H是4×8的。

    _std = (Mat_<float>(8,1) << 2 * this->_std_weight_position * measurement[3],
                                2 * this->_std_weight_position * measurement[3],
                                1e-2,
                                2 * this->_std_weight_position * measurement[3],
                                10 * this->_std_weight_velocity * measurement[3],
                                10 * this->_std_weight_velocity * measurement[3],
                                1e-5,
                                10 * this->_std_weight_velocity * measurement[3]);

    this->kfilter->errorCovPre = Mat::diag(_std.mul(_std)); // 状态的协方差矩阵P P'(k)=A*P(k-1)*At + Q) 8*8

    this->kfilter->statePost = (Mat_<float>(8,1)<< measurement[0],measurement[1], measurement[2],measurement[3],0,0,0,0); // 修正后的状态矩阵X，X=X'+K*(z-H*X') 8*1

    this->state = this->kfilter->statePost;
}

KalmanFilterToolBox::~KalmanFilterToolBox(){
    delete this->kfilter;
}

void KalmanFilterToolBox::predict(float errorCoefficient)
{
    this->kfilter->processNoiseCov = Mat::diag(_std.mul(_std))*errorCoefficient ; // 状态转移协方差矩阵Q  8*8

    this->kfilter->predict();   
}


void KalmanFilterToolBox::update(const DETECTBOX &measurement)
{   
    Mat _std_measurement = (Mat_<float>(4,1)<< _std_weight_position*measurement[3], 
                                               _std_weight_position*measurement[3],
                                               1e-2, 
                                               _std_weight_position*measurement[3]);
    this->kfilter->measurementNoiseCov = Mat::diag(_std_measurement.mul(_std_measurement)); // 测量矩阵的协方差矩阵R

    // The 4 dimensional measurement vector (x, y, a, h), where (x, y) is the center position, a the aspect ratio, and h the height of the bounding box.
    Mat InputMeasure = (Mat_<float>(4,1) << measurement(0),measurement(1),measurement(2),measurement(3));
    this->kfilter->correct(InputMeasure);
    
    this->state = this->kfilter->statePost;
}

// 计算kf预测位置和检测结果bbox间的马氏距离
Eigen::Matrix<float, 1, -1> KalmanFilterToolBox::mahalanobis_distance(const std::vector<DETECTBOX> &measurements, bool only_position)
{
    //KAL_HDATA pa = this->project(mean, covariance);
    if(only_position) {
        errMsg::getInstance()->out(
            "KalmanFilterToolBox.cpp",
            "KalmanFilterToolBox::mahalanobis_distance",
            "only_position, not implement!"
        );
    }
    Mat mean = this->kfilter->measurementMatrix * this->kfilter->statePre;
    Mat InputStd = (Mat_<float>(1,4) << _std_weight_position *this->kfilter->statePre.at<float>(3),
                                        _std_weight_position *this->kfilter->statePre.at<float>(3),
                                        1e-1, 
                                        _std_weight_position *this->kfilter->statePre.at<float>(3));
    Mat diag = Mat::diag(InputStd.mul(InputStd));
    //this->kfilter->measurementNoiseCov = diag;

    Mat measurementMatrxiTranspose;
    cv::transpose(this->kfilter->measurementMatrix, measurementMatrxiTranspose);
    Mat covar = this->kfilter->measurementMatrix * this->kfilter->errorCovPre * measurementMatrxiTranspose+diag;

    cv::transpose(mean,mean);

    Eigen::Matrix<float, 1, 4, Eigen::RowMajor> mean1 ;
    cv::cv2eigen(mean, mean1);
    Eigen::Matrix<float, 4, 4> covariance1 ;
    cv::cv2eigen(covar, covariance1);
    Eigen::Matrix<float, 4, 4> cov = covariance1.transpose();
    // cout << "mean : " <<  mean1 << endl;
    // cout << "covar : " << cov << endl<< endl;
    // Eigen::Matrix<float, -1, 4, Eigen::RowMajor> d(size, 4);
    DETECTBOXSS d(measurements.size(), 4);
    int pos = 0;

    for(DETECTBOX box:measurements) {   
        //cout << "measure : " << box << endl;
        d.row(pos++) = box - mean1;
    }
    //cout << endl;
    Eigen::Matrix<float, -1, -1, Eigen::RowMajor> factor = cov.llt().matrixL();
    Eigen::Matrix<float, -1, -1> z = factor.triangularView<Eigen::Lower>().solve<Eigen::OnTheRight>(d).transpose();
            
    auto zz = ((z.array())*(z.array())).matrix();
    auto square_maha = zz.colwise().sum();
    //cout << "square_maha" << square_maha << endl;
    return square_maha;
}

