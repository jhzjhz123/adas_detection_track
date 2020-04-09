#include "tracker.h"
#include "nn_matching.h"
#include "linear_assignment.h"

using namespace std;

//#define MY_inner_DEBUG
#ifdef MY_inner_DEBUG
#include <string>
#include <iostream>
#endif

tracker::tracker(/*NearNeighborDisMetric *metric,*/ float max_cosine_distance, int nn_budget, float max_iou_distance, int max_age, int n_init)
{
    // 使用nn_match的NearNeighborDisMetric（包含partial_fit/distance）
    // 在match阶段，将distance封装成gated_metric，进行外观信息（深度特征）与运动信息（马氏距离）的计算
    this->nnDisMetr_ptr = new NearNeighborDisMetric(NearNeighborDisMetric::METRIC_TYPE::cosine, max_cosine_distance, nn_budget);
    this->max_iou_distance = max_iou_distance;
    this->max_age = max_age;
    this->n_init = n_init;

    this->tracks.clear();
    this->_next_idx = 0;
}

tracker::~tracker()
{
    delete this->nnDisMetr_ptr;
}

void tracker::predict(){    
    // 依次对trk中所有的子trk遍历，逐个预测
    for(Track& track:tracks) 
        track.PredictTrack();
}

void tracker::update(const DETECTIONS &detections, IDDistributor* IDdis, int index, const HRYTCalibrater cali, const object::TransferType worldtype)
{
    // 一、进行检测结果和跟踪结果的匹配
    TRACHER_MATCHD res;
    _match(detections, res);
    
    // 二、根据匹配情况进行后续的追踪器相应模块的更新操作
    // （1）针对matched，要用检测结果去更新相应的tracker参数
    vector<MATCH_DATA>& matches = res.matches;
    for(MATCH_DATA& data:matches) { 
        int track_idx = data.first;
        int detection_idx = data.second;
        tracks[track_idx].UpdateTrack(detections[detection_idx], cali, worldtype);
    }

    // （2）针对unmatched_tracks，如果这个trk是还没有经过确认的，直接从trk列表中删除；
    // 如果这个是之前经过确认的，但是已经连续max_age帧（程序中设置为10）没能匹配检测结果了，也认为这个trk无效了，需要从trk列表中删除。
    vector<int>& unmatched_tracks = res.unmatched_tracks;
    for(int& track_idx:unmatched_tracks) { 
        this->tracks[track_idx].mark_missed();
    }

    // （3）针对unmatched_detections，要为其创建新的tracker。
    vector<int>& unmatched_detections = res.unmatched_detections;
    for(int& detection_idx:unmatched_detections) { 
        this->_initiate_track(detections[detection_idx], IDdis, index);
    }

    vector<Track>::iterator it;
    vector<int> eraseID;
    for(it = tracks.begin(); it != tracks.end();) {
        if((*it).is_deleted()) {
            eraseID.push_back(it->track_id());
            // need to push back first //
            it = tracks.erase(it);
        }
        else{
            if(it == tracks.end()) break;
            ++it;
        }
    }
    IDdis->erase(index, eraseID);

    #ifdef USE_FEATURE
    // 三、更新已经确认的trk的特征集模块
    // 使用active_targets保存confirmed的track的id
    // active_targets对应的feature（之前每帧只要能匹配上，都会把与之匹配的det的feature保存下来），用于更新kalman滤波的distance metric
    vector<int> active_targets;
    vector<TRACKER_DATA> tid_features;
    for (Track& track:tracks) {
        if(track.is_confirmed() == false) continue;
        active_targets.push_back(track.track_id());
        tid_features.push_back(std::make_pair(track.track_id(), track.features()));
    }
    // 并且使用partial_fit更新feature集，只保留budget数量的feature，且根据active_targets中的id去掉更改状态的feature集
    this->nnDisMetr_ptr->partial_fit(tid_features, active_targets);
    #endif
}

// 进行检测结果和跟踪结果的匹配
// 确认trk的状态tentative、confirmed、deleted，级联匹配、iou匹配
void tracker::_match(const DETECTIONS &detections, TRACHER_MATCHD &res)
{
    // 将已经存在tracks分为confirmed_tracks和unconfirmed_tracks
    vector<int> confirmed_tracks;
    vector<int> unconfirmed_tracks;
    int idx = 0;
    for(Track& t:tracks) { 
        if(t.is_confirmed()) confirmed_tracks.push_back(idx);
        else unconfirmed_tracks.push_back(idx);
        idx++;
    }
    
    // 针对之前confirmed_tracks，将它们与当前检测结果detections进行级联匹配
    TRACHER_MATCHD matcha = linear_assignment::getInstance()->matching_cascade(
                this, &tracker::gated_matric,
                this->nnDisMetr_ptr->mating_threshold,
                this->max_age,
                this->tracks,
                detections,
                confirmed_tracks);

    // unconfirmed_tracks和unmatched_tracks一起组成iou_track_candidates
    vector<int> iou_track_candidates;
    iou_track_candidates.assign(unconfirmed_tracks.begin(), unconfirmed_tracks.end());
    vector<int>::iterator it;		
    for(it = matcha.unmatched_tracks.begin(); it != matcha.unmatched_tracks.end();) {
        int idx = *it;
        if(tracks[idx].time_since_update() == 1) { //push into unconfirmed
            iou_track_candidates.push_back(idx);
            it = matcha.unmatched_tracks.erase(it);
            continue;
        }
        ++it;
    }
    
    // iou_track_candidates与unmatched_detections进行iou匹配
    TRACHER_MATCHD matchb = linear_assignment::getInstance()->min_cost_matching(
                this, &tracker::iou_cost,
                this->max_iou_distance,
                this->tracks,
                detections,
                iou_track_candidates,
                matcha.unmatched_detections);
    
    // 合并上述两步的匹配结果，get result:
    res.matches.assign(matcha.matches.begin(), matcha.matches.end());
    res.matches.insert(res.matches.end(), matchb.matches.begin(), matchb.matches.end());
    //unmatched_tracks;
    res.unmatched_tracks.assign(
                matcha.unmatched_tracks.begin(),
                matcha.unmatched_tracks.end());
    res.unmatched_tracks.insert(
                res.unmatched_tracks.end(),
                matchb.unmatched_tracks.begin(),
                matchb.unmatched_tracks.end());
    res.unmatched_detections.assign(
                matchb.unmatched_detections.begin(),
                matchb.unmatched_detections.end());
}

void tracker::_initiate_track(const DETECTION_ROW &detection, IDDistributor* IDdis, int index)
{
    this->_next_idx = IDdis->get(index);
    this->tracks.push_back(Track(detection, this->_next_idx, this->n_init, this->max_age)); // 初始化一个新的tracker
}


DYNAMICM tracker::gated_matric(std::vector<Track> &tracks, const DETECTIONS &dets, const std::vector<int>& track_indices, const std::vector<int>& detection_indices)
{
    #ifdef USE_FEATURE
    // 外观信息约束——计算当前每个新检测结果的深度特征features与这一level中每个trk已经保存的特征集targets之间的余弦距离矩阵
    FEATURESS features(detection_indices.size(), feature_dims);
    int pos = 0;
    for(int i:detection_indices) {
        features.row(pos++) = dets[i].feature;
    }
    targets.clear();
    for(int i:track_indices) {
        targets.push_back(tracks[i].track_id());
    }

    DYNAMICM cost_matrix = this->nnDisMetr_ptr->distance(features, targets); // 真正开始计算余弦距离
    #else
    DYNAMICM cost_matrix = Eigen::MatrixXf::Zero(track_indices.size(), detection_indices.size());
    #endif

    // 运动信息约束——Mahalanobis distance距离
    linear_assignment::getInstance()->gate_cost_matrix(cost_matrix, tracks, dets, track_indices, detection_indices);
    
    return cost_matrix; // 此时的cost_matrix包含了以上两个信息
}

DYNAMICM tracker::iou_cost(std::vector<Track> &tracks, const DETECTIONS &dets, const std::vector<int>& track_indices, const std::vector<int>& detection_indices){
    // 计算iou_track_candidates和unmatched_detections里的框两两之间的iou,经由1-iou的到cost_matric
    //!!!python diff: track_indices && detection_indices will never be None.
    //    if(track_indices.empty() == true) {
    //        for(size_t i = 0; i < tracks.size(); i++) {
    //            track_indices.push_back(i);
    //        }
    //    }
    //    if(detection_indices.empty() == true) {
    //        for(size_t i = 0; i < dets.size(); i++) {
    //            detection_indices.push_back(i);
    //        }
    //    }
    int rows = track_indices.size();
    int cols = detection_indices.size();
    DYNAMICM cost_matrix = Eigen::MatrixXf::Zero(rows, cols);
    for(int i = 0; i < rows; i++) {
        int track_idx = track_indices[i];
        if(tracks[track_idx].time_since_update() > 1) {
            cost_matrix.row(i) = Eigen::RowVectorXf::Constant(cols, INFTY_COST);
            continue;
        }
        DETECTBOX bbox = tracks[track_idx].to_tlwh();
        int csize = detection_indices.size();
        DETECTBOXSS candidates(csize, 4);
        for(int k = 0; k < csize; k++) candidates.row(k) = dets[detection_indices[k]].tlwh;
        
        // 经由1-iou得到cost_matric(阈值处理)
        Eigen::RowVectorXf rowV = (1. - iou(bbox, candidates).array()).matrix().transpose(); 
        cost_matrix.row(i) = rowV;
    }
    return cost_matrix;
}

Eigen::VectorXf tracker::iou(DETECTBOX& bbox, DETECTBOXSS& candidates)
{
    float bbox_tl_1 = bbox[0];
    float bbox_tl_2 = bbox[1];
    float bbox_br_1 = bbox[0] + bbox[2];
    float bbox_br_2 = bbox[1] + bbox[3];
    float area_bbox = bbox[2] * bbox[3];

    Eigen::Matrix<float, -1, 2> candidates_tl;
    Eigen::Matrix<float, -1, 2> candidates_br;
    candidates_tl = candidates.leftCols(2) ;
    candidates_br = candidates.rightCols(2) + candidates_tl;

    int size = int(candidates.rows());
    //    Eigen::VectorXf area_intersection(size);
    //    Eigen::VectorXf area_candidates(size);
    Eigen::VectorXf res(size);
    for(int i = 0; i < size; i++) {
        float tl_1 = std::max(bbox_tl_1, candidates_tl(i, 0));
        float tl_2 = std::max(bbox_tl_2, candidates_tl(i, 1));
        float br_1 = std::min(bbox_br_1, candidates_br(i, 0));
        float br_2 = std::min(bbox_br_2, candidates_br(i, 1));

        float w = br_1 - tl_1; w = (w < 0? 0: w);
        float h = br_2 - tl_2; h = (h < 0? 0: h);
        float area_intersection = w * h;
        float area_candidates = candidates(i, 2) * candidates(i, 3);
        res[i] = area_intersection/(area_bbox + area_candidates - area_intersection);
    }
    //#ifdef MY_inner_DEBUG
    //        std::cout << res << std::endl;
    //#endif
    return res;
}

