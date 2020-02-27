#ifndef LINEAR_ASSIGNMENT_H
#define LINEAR_ASSIGNMENT_H

#include <iomanip>
#include "dataType.h"
#include "tracker.h"
#include "KalmanFilterToolBox.h"
#include "hungarian.h"

#define INFTY_COST 1e5
class tracker;
//for matching;
class linear_assignment
{
    linear_assignment();
    linear_assignment(const linear_assignment& );
    ~linear_assignment();
    static linear_assignment* instance;
private:
    Hungarian hungarian;
public:
    static linear_assignment* getInstance();
    TRACHER_MATCHD matching_cascade(tracker* tracker_ptr,
            tracker::GATED_METRIC_FUNC distance_metric_func,
            float max_distance,
            int cascade_depth,
            std::vector<Track>& tracks,
            const DETECTIONS& detections,
            std::vector<int> &track_indices,
            std::vector<int> detection_indices = std::vector<int>());
    TRACHER_MATCHD min_cost_matching(tracker* tracker_ptr,
            tracker::GATED_METRIC_FUNC distance_metric_func,
            float max_distance,
            std::vector<Track>& tracks,
            const DETECTIONS& detections,
            std::vector<int>& track_indices,
            std::vector<int>& detection_indices);
    void gate_cost_matrix(
            DYNAMICM& cost_matrix,
            std::vector<Track>& tracks,
            const DETECTIONS& detections,
            const std::vector<int>& track_indices,
            const std::vector<int>& detection_indices,
            float gated_cost = INFTY_COST,
            bool only_position = false);
};

#endif // LINEAR_ASSIGNMENT_H
