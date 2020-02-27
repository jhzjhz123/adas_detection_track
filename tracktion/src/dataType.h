#pragma once
#ifndef DATATYPE_H
#define DATATYPEH

#include <iostream>
#include <cstddef>
#include <vector>
#include <map>
#include <Eigen/Core>
#include <Eigen/Dense>

#define FRAMEFREQUENCY 15
#define FRAMEFREQUENCY_W 30

//#define USE_FEATURE
const unsigned int feature_dims = 13; // num_class + 4 if hog 128

typedef Eigen::Matrix<float, 1, 4, Eigen::RowMajor> DETECTBOX;
typedef Eigen::Matrix<float, -1, 4, Eigen::RowMajor> DETECTBOXSS;
typedef Eigen::Matrix<float, 1, feature_dims, Eigen::RowMajor> FEATURE;
typedef Eigen::Matrix<float, Eigen::Dynamic, feature_dims, Eigen::RowMajor> FEATURESS;

//main
using RESULT_DATA = std::pair<int, DETECTBOX>;

//tracker:
using TRACKER_DATA = std::pair<int, FEATURESS>;
using MATCH_DATA = std::pair<int, int>;

typedef struct t{
    std::vector<MATCH_DATA> matches;
    std::vector<int> unmatched_tracks;
    std::vector<int> unmatched_detections;
}TRACHER_MATCHD;

//linear_assignment:
typedef Eigen::Matrix<float, -1, -1, Eigen::RowMajor> DYNAMICM;

// * Each rect's data structure.
// * tlwh: topleft point & (w,h)
// * confidence: detection confidence.
// * feature: the rect's 128d feature.
// */
class DETECTION_ROW
{
    const float kRatio=0.5;
    enum DETECTBOX_IDX {IDX_X = 0, IDX_Y, IDX_W, IDX_H };

public:
    unsigned int cls;
    DETECTBOX tlwh;
    float confidence;
    FEATURE feature;
    DETECTBOX to_xyah() const{//(centerx, centery, ration, h)
        DETECTBOX ret = tlwh;
        ret(0,IDX_X) += (ret(0, IDX_W)*kRatio);
        ret(0, IDX_Y) += (ret(0, IDX_H)*kRatio);
        ret(0, IDX_W) /= ret(0, IDX_H);
        return ret;
    }

    DETECTBOX to_tlbr() const{//(x,y,xx,yy)
        DETECTBOX ret = tlwh;
        ret(0, IDX_X) += ret(0, IDX_W);
        ret(0, IDX_Y) += ret(0, IDX_H);
        return ret;
    }

    DETECTION_ROW& operator=(const DETECTION_ROW& tmpRow){
        this->cls = tmpRow.cls;
        this->tlwh = tmpRow.tlwh;
        this->confidence = tmpRow.confidence;
        this->feature = tmpRow.feature;
        this->to_xyah() = tmpRow.to_xyah();
        this->to_tlbr() = tmpRow.to_tlbr();
        return *this;
    }
};

typedef std::vector<DETECTION_ROW> DETECTIONS;

class IDDistributor
{
private:
    std::map<int, std::vector<int>> IDcontainer;
    int _max_id;
    int _group_num;
    std::vector<int> IDlist;
public:
    IDDistributor(const int maxID,const int groups){
        _max_id = maxID;
        _group_num = groups;
        for(int i = 0; i < maxID; i++)
            IDlist.push_back(i);
        for (int i = 0; i < _group_num; i++)
        {
            std::vector<int> targets;
            IDcontainer.insert(std::pair<int,std::vector<int>>(i,targets));
        }
    }
    ~IDDistributor(){
        IDcontainer.clear();
    }

    int get(const int index){
        std::vector<int> targets = this->IDcontainer[index];
        std::vector<int> idlst = IDlist;

        for (auto it = targets.begin(); it != targets.end(); it++)
        {
            auto item = find(idlst.begin(), idlst.end(), *it);
            if(item == IDlist.end()) continue;
            idlst.erase(item);
        }
        auto min_item = min_element(idlst.begin(), idlst.end());
        this->IDcontainer[index].push_back(*min_item);
        //cout << "index: " << index << "  min id: " <<*min_item << endl;
        return *min_item;
    }

    void erase(const int index, const std::vector<int> IDErase){
        for (auto item = IDErase.begin(); item != IDErase.end(); item++)
        {
            auto it_erase = find(this->IDcontainer[index].begin(), this->IDcontainer[index].end(), *item);
            //cout << "index: " << index << "  erase id: " << *it_erase << endl;
            this->IDcontainer[index].erase(it_erase);
        }
    }

    void clear(const int index){
        this->IDcontainer[index].clear();
    }
};

#endif // DATATYPE_H
