#ifndef TRACK_H
#define TRACK_H

#include "dataType.h"

#include "KalmanFilterToolBox.h"
#include "Object.h"


/*Changable*/
#define PIXEL_ERROR_CHANGERATE_WITH_SPEED 0.1 /*0.01~0.1*/
#define MAXIMUM_PIXEL_ERROR 2 /*less than 1*/
#define WORLD_ERROR_CHANGERATE_WITH_SPEED 1 /*1~3*/
/***********/

#define MAXIMUM_WORLD_ERROR WORLD_ERROR_CHANGERATE_WITH_SPEED 
#define MINIMUM_WORLD_ERROR 0.1 /*less than 1*/
#define PIXEL_THRESHOLD 5
#define MAXIMUM_DISTANCE_ESTIMATION 200


class Track:public object
{
    friend class linear_assignment;
    /*"""
    A single target track with state space `(x, y, a, h)` and associated
    velocities, where `(x, y)` is the center of the bounding box, `a` is the
    aspect ratio and `h` is the height.

    Parameters
    ----------
    mean : ndarray
        Mean vector of the initial state distribution.
    covariance : ndarray
        Covariance matrix of the initial state distribution.
    track_id : int
        A unique track identifier.
    n_init : int
        Number of consecutive detections before the track is confirmed. The
        track state is set to `Deleted` if a miss occurs within the first
        `n_init` frames.
    max_age : int
        The maximum number of consecutive misses before the track state is
        set to `Deleted`.
    feature : Optional[ndarray]
        Feature vector of the detection this track originates from. If not None,
        this feature is added to the `features` cache.

    Attributes
    ----------
    mean : ndarray
        Mean vector of the initial state distribution.
    covariance : ndarray
        Covariance matrix of the initial state distribution.
    track_id : int
        A unique track identifier.
    hits : int
        Total number of measurement updates.
    _age : int
        Total number of frames since first occurance.
    time_since_update : int
        Total number of frames since last measurement update.
    state : TrackState
        The current track state.
    features : List[ndarray]
        A cache of features. On each measurement update, the associated feature
        vector is added to this list.

    """*/
    enum TrackState {Tentative = 1, Confirmed, Deleted};

public:
    Track(const DETECTION_ROW& detection, const int track_id, const int n_init, const int max_age);

    ~Track();

    void PredictTrack();
    
    void UpdateTrack(const DETECTION_ROW& detection, const Calibrater cali, const object::TransferType &worldtype);

    void mark_missed();

    /*
    当某帧出现新的检测结果的时候（即，出现了与当前跟踪结果无法match的检测结果），我们认为可能出现了新的目标，会为其创建新的跟踪器trk。
    不过我们需要观察一段时间，如果连续3帧中潜在的新跟踪器对目标位置的预测结果（标记为tentative）都能够与检测结果match，那么则确认是出现了新的目标轨迹（标记为confirmed）；
    如果不能达到该要求，则认为是出现了“虚警”，需要从跟踪器列表trks中删除该trk（标记为deleted）。
    */

    bool is_confirmed()const {return this->_state == TrackState::Confirmed;}

    bool is_deleted()const {return this->_state == TrackState::Deleted;}

    bool is_tentative()const {return this->_state == TrackState::Tentative;}

    int track_id()const{return this->_track_id;}

    int time_since_update() const{return this->_time_since_update;}

    DETECTBOX to_tlwh();

#ifdef USE_FEATURE
    FEATURESS features()const{return this->_features;}
private:
    FEATURESS _features;
#endif

private:
    float _std_weight_position;
    float _std_weight_velocity;
    float _std_weight_accelarate;

    int _max_age; // 轨迹最大允许丢失匹配的帧数
    int _hits;
    int _age;
    int _n_init;
    int _track_id;
    int _time_since_update;
    float errorCoefficient ;

    float dt;

    cv::KalmanFilter *kfw;
    KalmanFilterToolBox *kf;

    TrackState _state;


    /**
     * @brief pixel coordinate errorcoefficient
     * @param kalman - pixel kalman filter
     * @return errorcoefficient
    */
    float UpdateErrorCoefficient(const KalmanFilterToolBox *kalman);

    /**
     * @brief require the object from filter
     * @param kalman - pixel kalman filter
     * @return none
    */
    void UpdateObject(const KalmanFilterToolBox *kalman);

    /**
     * @brief estimation of world coordinate
     * @param w - choose which method to estimate distance
     * @param cali - calibrator
     * @return world coordinate with x,y,w,h
    */
    cv::Rect2f PredictDistance(const object::TransferType &w, const Calibrater &cali);

    /**
     * @brief world coordinate initiation
     * @param kalman - world coordinate kalmanfilter
     * @param measurement - initial measurement
     * @return none
    */
    void WorldFilterInit(cv::KalmanFilter *kalman, const DETECTBOX &measurement);

    /**
     * @brief world coordinate errorcoefficient
     * @param measurement - initial measurement
     * @return errorcoefficient
    */
    float WorldCoordinateErrorCoefficient(cv::Mat &measurement);

    /**
     * @brief world coordinate kalmanfilter predict
     * @param kalman - world coordinate kalmanfilter
     * @param measurement - measurement
     * @param camvelocity - camvelocity
     * @param error - error to update process noise
     * @return none
    */
    void WorldCoordinatePredict(cv::KalmanFilter *kalman, const Mat &measurement,const float error);

    /**
     * @brief world coordinate kalmanfilter correct
     * @param kalman - world coordinate kalmanfilter
     * @param measurement - measurement
     * @return none
    */
    void WorldCoordinateCorrect(cv::KalmanFilter *kalman, const Mat &measurement);

    void ObjectClassify(const KalmanFilterToolBox *kalman);

};

#endif // TRACK_H
