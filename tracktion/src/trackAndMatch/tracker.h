#ifndef TRACKER_H
#define TRACKER_H
#include <vector>

#include "KalmanFilterToolBox.h"
#include "track.h"

class NearNeighborDisMetric;
class IDDistributor;

class tracker{
    /*
    This is the multi-target tracker.
    Parameters
    ----------
    metric : nn_matching.NearestNeighborDistanceMetric
        A distance metric for measurement-to-track association.
    max_age : int
        Maximum number of missed misses before a track is deleted.
    n_init : int
        Number of consecutive detections before the track is confirmed. The
        track state is set to `Deleted` if a miss occurs within the first
        `n_init` frames.
    */
public:
    NearNeighborDisMetric* nnDisMetr_ptr;
    float max_iou_distance;
    int max_age;
    int n_init;

    int _next_idx;
public:
    std::vector<Track> tracks;
    tracker(/*NearNeighborDisMetric* metric,*/ float max_cosine_distance, int nn_budget, float max_iou_distance = 0.9, int max_age = 10, int n_init=3);

    ~tracker();

    // Propagate track state distributions one time step forward. This function should be called once every time step, before `update`.
    void predict();
    // Perform measurement update and track management
    void update(const DETECTIONS& detections, IDDistributor* IDdis, int index, const Calibrater cali, const object::TransferType worldtype);

    // https://www.cnblogs.com/zhubaohua-bupt/p/7182803.html
    typedef DYNAMICM (tracker::* GATED_METRIC_FUNC)(std::vector<Track>& tracks, const DETECTIONS& dets, const std::vector<int>& track_indices, const std::vector<int>& detection_indices);
    
    DYNAMICM gated_matric(std::vector<Track>& tracks, const DETECTIONS& dets, const std::vector<int>& track_indices, const std::vector<int>& detection_indices);
    DYNAMICM iou_cost(std::vector<Track>& tracks, const DETECTIONS& dets, const std::vector<int>& track_indices, const std::vector<int>& detection_indices);
    Eigen::VectorXf iou(DETECTBOX& bbox, DETECTBOXSS &candidates);
private:   
    std::vector<int> targets;
    void _match(const DETECTIONS& detections, TRACHER_MATCHD& res);
    void _initiate_track(const DETECTION_ROW& detection, IDDistributor* IDdis, int index);
};

#endif // TRACKER_H
