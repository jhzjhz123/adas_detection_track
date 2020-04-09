#include "track.h"

Track::Track(const DETECTION_ROW& detection, const int track_id, const int n_init, const int max_age)
{
    /* ID assigned to object */
    this->_track_id = track_id;
    /* counter to track*/
    this->_hits = 1;
    /* Track life duration counter*/
    this->_age = 1;
    /* update counter */
    this->_time_since_update = 0;
    this->_state = TrackState::Tentative; // 初始状态
    /* confirmed track threshold  */
    this->_n_init = n_init;
    /* delete track threshold */
    this->_max_age = max_age;
    /* feature to save */

    #ifdef USE_FEATURE
    this->_features = FEATURESS(1, feature_dims);
    FEATURE feature = detection.feature;
    this->_features.row(0) = feature;//features.rows() must = 0;
    #endif

    /* errorCoeffcient to track the object at pixel level */
    this->errorCoefficient = 1.0;

    this->dt = 1.0 / FRAMEFREQUENCY_W;

    DETECTBOX measurement = detection.to_xyah();
    this->kf = new KalmanFilterToolBox(4,1,measurement);
    this->kfw = new cv::KalmanFilter(6,2,0);

    this->WorldFilterInit(this->kfw, measurement);

    this->cls_update(detection.cls);
}

Track::~Track(){
    // delete this->kf;
    // delete this->kfw;
}

void Track::WorldFilterInit(cv::KalmanFilter *kalman, const DETECTBOX &measurement){
    kalman->transitionMatrix = (cv::Mat_<float>(6, 6)<< 1, 0, dt, 0,  (dt*dt)/2, 0, 
                                                        0, 1, 0,  dt, 0,         (dt*dt)/2,
                                                        0, 0, 1,  0,  dt/2,      0,
                                                        0, 0, 0,  1,  0,         dt/2,
                                                        0, 0, 0,  0,  1,         0,
                                                        0, 0, 0,  0,  0,         1);
    setIdentity(kalman->measurementMatrix);
    setIdentity(kalman->processNoiseCov, cv::Scalar::all(1E-3));
    setIdentity(kalman->measurementNoiseCov, Scalar::all(1E-6));
    setIdentity(kalman->errorCovPost, Scalar::all(0.1));
    kalman->statePost = (cv::Mat_<float>(6,1) << measurement(0), measurement(1), 0,0,0,0);
    this->_std_weight_position = 0.1;
    this->_std_weight_velocity = 0.1;
    this->_std_weight_accelarate = 0.01;
}

float Track::UpdateErrorCoefficient(const KalmanFilterToolBox *kalman){
    float x_direction_pix_vel = kalman->state.at<float>(4);
    float y_direction_pix_vel =kalman->state.at<float>(5);
    float pix_vel = sqrt(x_direction_pix_vel*x_direction_pix_vel + \
    y_direction_pix_vel*y_direction_pix_vel);

    float error = PIXEL_ERROR_CHANGERATE_WITH_SPEED*pix_vel;
    if(error > MAXIMUM_PIXEL_ERROR) error = MAXIMUM_PIXEL_ERROR;
    if(error < PIXEL_ERROR_CHANGERATE_WITH_SPEED) error = PIXEL_ERROR_CHANGERATE_WITH_SPEED;
    return error;
}

void Track::UpdateObject(const KalmanFilterToolBox *kalman){
    float x_direction_pix_vel = kalman->state.at<float>(4);
    float y_direction_pix_vel = kalman->state.at<float>(5);
    float w_direction_pix_vel = kalman->state.at<float>(6);
    this->vel_pix.x = x_direction_pix_vel;
    this->vel_pix.y = y_direction_pix_vel;
    
    DETECTBOX detbox = this->to_tlwh();
    this->lti = detbox(0);
    this->ltj = detbox(1);
    this->w = detbox(2);
    this->h = detbox(3);
    this->mbi = this->lti + this->w/2.;
    this->mbj = this->ltj + this->h;
    this->mti = this->lti + this->w/2.;
    this->mtj = this->ltj;
    this->lbi = this->lti;
    this->lbj = this->ltj + this->h;
    this->rbi = this->lti + this->w;
    this->rbj = this->ltj + this->h;
}

void Track::PredictTrack(){
    // 使用kf对mean(x)和P(k)均方误差进行预测
    this->kf->predict(errorCoefficient);
    this->_age += 1;
    this->_time_since_update += 1; // 预测完成以后，需要对每一个tracker的_time_since_update加1,。
}

cv::Rect2f Track::PredictDistance(const object::TransferType &w, const HRYTCalibrater &cali){
    cv::Rect2f worldcoordinate;
    switch (w)
    {
    case object::TransferType::WidthHeight:
        worldcoordinate = this->pixel2worldwidthheight(cali);
        break;
    case object::TransferType::WidthLength:
        worldcoordinate = this->pixel2worldwidthlength(cali);
        break;
    case object::TransferType::PixelOnly:
        worldcoordinate = this->pixel2worldpixelonly(cali);
        break;            
    }
    return worldcoordinate;
}


float Track::WorldCoordinateErrorCoefficient(cv::Mat &measurement){
    float error = (MAXIMUM_DISTANCE_ESTIMATION - measurement.at<float>(0))/MAXIMUM_DISTANCE_ESTIMATION*WORLD_ERROR_CHANGERATE_WITH_SPEED;
    if(error < MINIMUM_WORLD_ERROR) error = MINIMUM_WORLD_ERROR;
    if(error > MAXIMUM_WORLD_ERROR) error = MAXIMUM_WORLD_ERROR;
    return error;
}

void Track::WorldCoordinatePredict(cv::KalmanFilter *kalman, const Mat &measurement, const float error){

    if(!this->kf_ini_flag){
        kalman->statePost = (cv::Mat_<float>(6,1) << measurement.at<float>(0), 
        measurement.at<float>(1), 0,0,0,0);
        this->kf_ini_flag = true;
    }
    Mat InputState = (Mat_<float>(6,1) << 
        error*_std_weight_position * measurement.at<float>(0),
        error*_std_weight_position * measurement.at<float>(1),
        error*_std_weight_velocity * measurement.at<float>(0),
        error*_std_weight_velocity * measurement.at<float>(1),
        error*_std_weight_accelarate * measurement.at<float>(0),
        error*_std_weight_accelarate * measurement.at<float>(1)
        ); 

    kalman->processNoiseCov = Mat::diag(InputState.mul(InputState)); // 4*4

    kalman->predict();
}

void Track::WorldCoordinateCorrect(cv::KalmanFilter *kalman, const Mat &measurement){
    Mat InputSMeas = (Mat_<float>(2,1) << 
        _std_weight_position * measurement.at<float>(0),
        _std_weight_position * measurement.at<float>(1));
    kalman->measurementNoiseCov = Mat::diag(InputSMeas.mul(InputSMeas)); // 2*2
    kalman->correct(measurement);
}


void Track::UpdateTrack(const DETECTION_ROW& detection, const HRYTCalibrater cali, const object::TransferType &worldtype){
    /*
    更新包括以下几个参数：
    a. 更新kalman滤波的一系列运动变量、命中次数以及重置_time_since_update;
    b. det的深度特征保存到trk的特征集中；
    c. 如果已经连续命中3帧，将trk的状态改为confirmed;
    */
    /*pixel filter correct*/
    this->kf->update(detection.to_xyah());
    this->_hits += 1;
    this->_time_since_update = 0;
    if(this->_state == TrackState::Tentative && this->_hits >= this->_n_init) { // 如果命中帧数大于预设帧数，状态改为confirmed
        this->_state = TrackState::Confirmed;
    }

    this->ObjectClassify(kf);

    /*require error from the pixel filter*/
    this->errorCoefficient = this->UpdateErrorCoefficient(kf);
    /*copy the filter data to object class*/
    this->UpdateObject(kf);

    /*estimate distance*/
    this->world = this->PredictDistance(worldtype, cali);

    Mat measurement = Mat::zeros(2,1, CV_32F);
    measurement.at<float>(0) = this->world.x;
    measurement.at<float>(1) = this->world.y;

    /*require error from the world filter*/
    float error = this->WorldCoordinateErrorCoefficient(measurement);

    /*world filter predict*/
    this->WorldCoordinatePredict(kfw, measurement, error);

    /*world filter correct*/
    this->WorldCoordinateCorrect(kfw, measurement);
    //boundaryset(this->kf->statePost);

    this->world.x = this->kfw->statePost.at<float>(0);
    this->world.y = this->kfw->statePost.at<float>(1);
    this->vel.x = this->kfw->statePost.at<float>(2);
    this->vel.y = this->kfw->statePost.at<float>(3);   

    this->cls_update(detection.cls);
}

void Track::mark_missed()
{
    if(this->_state == TrackState::Tentative) {
        this->_state = TrackState::Deleted;
    } else if(this->_time_since_update > this->_max_age) { // 已经连续max_age帧没能匹配检测结果了，热认为trk无效了，需要从trk列表中删除
        this->_state = TrackState::Deleted;
    }
}



DETECTBOX Track::to_tlwh()
{
    DETECTBOX ret;//
    //ret = mean.leftCols(4);
    //ret(2) *= ret(3);
    //ret.leftCols(2) -= (ret.rightCols(2)/2);
    ret(0) = this->kf->state.at<float>(0);
    ret(1) = this->kf->state.at<float>(1);
    ret(2) = this->kf->state.at<float>(2);
    ret(3) = this->kf->state.at<float>(3);
    
    ret(2) *= ret(3);
    ret(0) -= ret(2)/2;
    ret(1) -= ret(3)/2;
    return ret;
}

void Track::ObjectClassify(const KalmanFilterToolBox *kalman){

    this->vel_pix.x = kalman->state.at<float>(4);
    this->vel_pix.y = kalman->state.at<float>(5);

    float x_direction_pix_vel = this->vel_pix.y;
    float y_direction_pix_vel = this->vel_pix.x;
    if( abs(x_direction_pix_vel) < PIXEL_THRESHOLD && abs(y_direction_pix_vel) < PIXEL_THRESHOLD){
        if(this->objstate == object::ObjectState::NotMoving){
            this->consistant_state_id++;
        }else{
            this->objstate = object::ObjectState::NotMoving;
            this->consistant_state_id = 0;
        }
    }else if( x_direction_pix_vel < -PIXEL_THRESHOLD){
        if(this->objstate == object::ObjectState::MovingFarAway){
            this->consistant_state_id++;
        }else{
            this->objstate = object::ObjectState::MovingFarAway;
            this->consistant_state_id = 0;
        }
    }else{
        if(this->objstate == object::ObjectState::Closing){
            this->consistant_state_id++;
        }else{
            this->objstate = object::ObjectState::Closing;
            this->consistant_state_id = 0;
        }
    }
}