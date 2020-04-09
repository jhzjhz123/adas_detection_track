#include "linear_assignment.h"
#include <map>

linear_assignment *linear_assignment::instance = NULL;
linear_assignment::linear_assignment()
{
}

linear_assignment::~linear_assignment()
{
    if(instance != NULL)
        delete instance;
}

linear_assignment *linear_assignment::getInstance()
{
    if(instance == NULL) 
        instance = new linear_assignment();
    return instance;
}

// 级联匹配需要从刚刚匹配成功的trk循环遍历到最多已经有10次（即cascade_depth或max_age）没有匹配的trk，以对更加频繁出现的目标赋予优先权
// 所以这里的状态为confirmed但却已经好多帧没有匹配到检测结果的trk
TRACHER_MATCHD linear_assignment::matching_cascade(
        tracker *tracker_ptr,
        tracker::GATED_METRIC_FUNC distance_metric_func,
        float max_distance,
        int cascade_depth,
        std::vector<Track> &tracks,
        const DETECTIONS &detections,
        std::vector<int>& track_indices,
        std::vector<int> detection_indices)
{
    // 首先分配track-indices和detection-indices
    TRACHER_MATCHD res;
    for(size_t i = 0; i < detections.size(); i++) {
        detection_indices.push_back(int(i));
    }
    std::vector<int> unmatched_detections;
    unmatched_detections.assign(detection_indices.begin(), detection_indices.end());
    res.matches.clear();
    std::vector<int> track_indices_l;

    // 遍历（合并匹配结果）
    // (1). 计算当前帧每个检测结果的深度特征与这一level中每个trk中已保留的特征集之间的余弦距离矩阵cost_matric，取最小值作为该trk与检测结果之间的计算值；
    // (2). 在cost_matric中，进行运动信息约束。对于每个trk，计算其kf预测结果和检测结果之间的马氏距离，并存放于cost_matric中，当trk的马氏距离大于阈值（gating_threshold）的值设为无穷大；
    // (3). 将经由max_distance处理之后的cost_matric作为匈牙利算法的输入，得到线性匹配结果，并去除差距较大的匹配
    std::map<int, int> matches_trackid;
    for(int level = 0; level < cascade_depth; level++) {
        if(unmatched_detections.size() == 0) break; //No detections left;

        track_indices_l.clear();
        for(int k:track_indices) {
            if(tracks[k].time_since_update() == 1+level)
                track_indices_l.push_back(k);
        }
        if(track_indices_l.size() == 0) continue; //Nothing to match at this level.

        // 计算cost_matric
        TRACHER_MATCHD tmp = min_cost_matching(
                    tracker_ptr, distance_metric_func,
                    max_distance, tracks, detections, track_indices_l,
                    unmatched_detections);
        unmatched_detections.assign(tmp.unmatched_detections.begin(), tmp.unmatched_detections.end());
        for(size_t i = 0; i < tmp.matches.size(); i++) {
            MATCH_DATA pa = tmp.matches[i];
            res.matches.push_back(pa);
            matches_trackid.insert(pa);
        }
    }
    res.unmatched_detections.assign(unmatched_detections.begin(), unmatched_detections.end());
    for(size_t i = 0; i < track_indices.size(); i++) {
        int tid = track_indices[i];
        if(matches_trackid.find(tid) == matches_trackid.end())
            res.unmatched_tracks.push_back(tid);
    }
    return res; // 经过上述处理之后，得到当前最终级联匹配结果
}

TRACHER_MATCHD linear_assignment::min_cost_matching(tracker *tracker_ptr,
        tracker::GATED_METRIC_FUNC distance_metric_func,
        float max_distance,
        std::vector<Track> &tracks,
        const DETECTIONS &detections,
        std::vector<int> &track_indices,
        std::vector<int> &detection_indices)
{
    TRACHER_MATCHD res;

    if((detection_indices.size() == 0) || (track_indices.size() == 0)) {
        res.matches.clear();
        res.unmatched_tracks.assign(track_indices.begin(), track_indices.end());
        res.unmatched_detections.assign(detection_indices.begin(), detection_indices.end());
        return res;
    }

    // 执行级联匹配遍历步骤中的（1）和（2）
    // https://blog.csdn.net/qq_33154343/article/details/84141832#%5B1%5D%C2%A0%20%E6%88%90%E5%91%98%E6%8C%87%E9%92%88%E8%AE%BF%E9%97%AE%E8%BF%90%E7%AE%97%E7%AC%A6%20.*
    DYNAMICM cost_matrix = (tracker_ptr->*(distance_metric_func))(tracks, detections, track_indices, detection_indices);

    // 执行级联匹配遍历步骤中的（3）
    // a. 把经过马氏距离处理的矩阵cost_matric继续经由max_distance（max_cosine_distance=0.9, max_iou_distance = 0.9）处理, 把cost大于阈值的，都设置成阈值+0.00001.
    for(int i = 0; i < cost_matrix.rows(); i++) {
        for(int j = 0; j < cost_matrix.cols(); j++) {
            float tmp = cost_matrix(i,j);
            if(tmp > max_distance) cost_matrix(i,j) = max_distance + 1e-5;
        }
    }
    //Eigen::Matrix<float, -1, 2, Eigen::RowMajor> indices = HungarianOper::Solve(cost_matrix);
    // cout << "detection size : " << detections.size() << endl;
    // for (int i = 0; i < detections.size(); i++)
    // {
    //     cout << "detections : " << detections[i].tlwh << endl;
    // }
    
    //cout << "tracks size : " << tracks.size() << endl;
    // for (int i = 0; i < tracks.size(); i++)
    // {
    //     cout << "tracks : " << tracks[i].track_id << " position : "<< tracks[i].to_tlwh() << endl;
    // }
    //cout << "cost_matrix : " << cost_matrix << endl;

    // b. 把经由max_distance处理之后的cost_matric作为匈牙利算法的输入，得到线性匹配结果
    Eigen::Matrix<float, -1, 2, Eigen::RowMajor> indices = hungarian.solve(cost_matrix);
    //cout << "indices : " << indices << endl ;

    // c. 对于匹配结果进行筛选，删去两者差别(即cosine distance)太大的，得到当前level的匹配结果
    res.matches.clear();
    res.unmatched_tracks.clear();
    res.unmatched_detections.clear();
    for(size_t col = 0; col < detection_indices.size(); col++) {
        bool flag = false;
        for(int i = 0; i < indices.rows(); i++)
            if(indices(i, 1) == col) { flag = true; break;}
        if(flag == false)res.unmatched_detections.push_back(detection_indices[col]);
    }
    for(size_t row = 0; row < track_indices.size(); row++) {
        bool flag = false;
        for(int i = 0; i < indices.rows(); i++)
            if(indices(i, 0) == row) { flag = true; break; }
        if(flag == false) res.unmatched_tracks.push_back(track_indices[row]);
    }
    for(int i = 0; i < indices.rows(); i++) {
        int row = indices(i, 0);
        int col = indices(i, 1);

        int track_idx = track_indices[row];
        int detection_idx = detection_indices[col];
        // cout << "row : " << row << " col : " << col <<  "track id : " << track_idx << " detection id : " << detection_idx << 
        // " cost_matrix : " << cost_matrix(row, col) <<endl;

        if(cost_matrix(row, col) > max_distance) { // 如果某个组合的cost值大于阈值， 这样的组合还是认为不match的，相应的，还要把组合中的检测框和跟踪框都踢到各自的unmatched列表中。
            res.unmatched_tracks.push_back(track_idx);
            res.unmatched_detections.push_back(detection_idx);
        } else {
            res.matches.push_back(std::make_pair(track_idx, detection_idx));
        }
    }
    return res; // 经过上述处理之后，得到当前初步matches、unmatche-tracks、unmat-detections
}

// DYNAMICM
void linear_assignment::gate_cost_matrix(
    DYNAMICM &cost_matrix,
    std::vector<Track> &tracks,
    const DETECTIONS &detections,
    const std::vector<int> &track_indices,
    const std::vector<int> &detection_indices,
    float gated_cost, bool only_position)
{
    // 运动信息约束
    int gating_dim = (only_position == true?2:4);
    double gating_threshold = KalmanFilterToolBox::chi2inv95[gating_dim];
    std::vector<DETECTBOX> measurements;
    
    // 将各检测结果由[x,y,w,h]转换为[center_x,center_y,aspect_ratio,height]
    for(int i:detection_indices) { 
        DETECTION_ROW t = detections[i];
        measurements.push_back(t.to_xyah());
    }

    // 对于每个trk,计算其kf预测位置和检测结果bbox间的马氏距离，并放在cost_matric中，针对马氏距离大于阈值（gating_threshold）的值设置为100000（gated_cost，相当于无穷大）
    for(size_t i  = 0; i < track_indices.size(); i++) {
        Track& track = tracks[track_indices[i]];
        Eigen::Matrix<float, 1, -1> gating_distance = track.kf->mahalanobis_distance(measurements, only_position);
        for (int j = 0; j < gating_distance.cols(); j++) {
            if (gating_distance(0, j) > gating_threshold)  
                cost_matrix(i, j) = gated_cost;
        }
    }
    // return cost_matrix;
}