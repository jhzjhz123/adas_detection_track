#ifndef NN_MATCHING_H
#define NN_MATCHING_H

#include "dataType.h"
#include <map>

//A tool to calculate distance;
class NearNeighborDisMetric{
    /*
    A nearest neighbor distance metric that, for each target, returns
    the closest distance to any sample that has been observed so far.
    Parameters
    ----------
    metric : str
        Either "euclidean" or "cosine".
    matching_threshold: float
        The matching threshold. Samples with larger distance are considered an
        invalid match.
    budget : Optional[int]
        If not None, fix samples per class to at most this number. Removes
        the oldest samples when the budget is reached.
    */

public:
    enum METRIC_TYPE{euclidean=1, cosine};
    NearNeighborDisMetric(METRIC_TYPE metric_type, float matching_threshold, int budget);
    float mating_threshold;

#ifdef USE_FEATURE
    void partial_fit(std::vector<TRACKER_DATA>& tid_feats, std::vector<int>& active_targets);
    DYNAMICM distance(const FEATURESS& features, const std::vector<int> &targets);
private:
    std::map<int, FEATURESS > samples;
#endif

private:
    typedef Eigen::VectorXf (NearNeighborDisMetric::*PTRFUN)(const FEATURESS&, const FEATURESS&);
    Eigen::VectorXf _nneuclidean_distance(const FEATURESS& x, const FEATURESS& y);
    Eigen::MatrixXf _pdist(const FEATURESS& x, const FEATURESS& y);

    Eigen::VectorXf _nncosine_distance(const FEATURESS& x, const FEATURESS& y);
    Eigen::MatrixXf _cosine_distance(const FEATURESS & a, const FEATURESS& b, bool data_is_normalized = false);
private:
    PTRFUN func_ptr;
    int budget;

};

#endif // NN_MATCHING_H
