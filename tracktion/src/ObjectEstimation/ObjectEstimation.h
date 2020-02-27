#ifndef OBJECTESTIMATION_H_
#define OBJECTESTIMATION_H_

#include <map>
#include <vector>
#include <iomanip>
#include "tracker.h"
#include <Eigen/Core>
#include <opencv2/core/eigen.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv2/highgui.hpp>
#include "Calibration/Calibrater.h"
#include "dataType.h"
#include "DistanceMeasure.h"
#include "Object.h"
#include "trajectory.h"
#include "TTC.h"

#ifdef AISDK_INTERFACE
#else
#include"darknet.h"
#endif

using namespace std;

#define SAVEOBJECTS 30


class Track;
class tracker;
class IDDistributor;
class TTC;
class Calibrater;

class ObjectEstimation: public object
{
public:

#ifdef DISTANCEMEASURE
    ObjectEstimation(double fx, double fy ,double cx, double cy, double k1, \
        double k2, double p1, double p2, double k3, double pitch, double yaw, double roll, \
        double Cam2Chamber, double Cam2Center ,double CamHeight);    
#else
    ObjectEstimation(const std::string &config_file);
#endif

    ~ObjectEstimation();
    
#ifdef AISDK_INTERFACE
    std::map<int,DETECTIONS> getdetections(const std::vector<std::vector<float>>& dets, const cv::Mat& frame, const int& frame_idx);
#else
    std::map<int,DETECTIONS> getdetections(const detection_with_class* dets,  const int& num, const cv::Mat& frame, const int& frame_idx);
#endif

    objects UpdateAll(map<int, DETECTIONS> d, const std::map<int, Point2f>& W, const cv::Point2f CamVelocity);

    int get_frameid() const {return frameID;};
    void set_moveThreshold(const float& threshold1, const float& threshold2) {a_threshold = threshold1, r_threshold = threshold2;};
    std::pair<float, float> get_moveThreshold() const {return std::pair<float, float>(a_threshold, r_threshold);};
    
    void SaveDetections(const int& frame_idx, const DETECTIONS& dets);
    void SaveObjects(objects& objs, const std::map<int, Point2f> &W);    
    inline void EstimateMoveState(objects& objs_);

    bool getFrameDetections(int frame_idx, DETECTIONS& res);
    bool getFrameTracks(int frame_idx, objects& objs);


    void drawdetect(Mat &image, map<int, DETECTIONS> d,const std::map<int ,cv::Point2f> W, const objects objs);
    void draw_image(const objects &objs, Mat& image);
    void draw_ground(const objects &objs, Mat& image);
    void draw_ego(const std::map<int, Point2f>& W, cv::Mat &imgego);
    void draw_Calibrater(cv::Mat &img, const Calibrater &cali);
    template<typename Tobj>
    void draw_image_per_class(Mat &image, const Tobj obj, const std::string name);
    template<typename Tobj>
    void draw_ground_per_class(Mat& image, const Tobj obj,  cv::Scalar scalar,const std::string name);

private:
    template<typename Tobj> 
    map<int, Tobj> UpdateOneClass(const DETECTIONS detections, tracker* tr, map<int, Tobj> obj_old, int index, object::TransferType worldtype);

    vector<string> clname;
    vector<vector<int>> clid;
    const int nn_budget;
    const float max_cosine_distance;
    const int groups;
    const int maxid;
    int frameID;
    int frame_num_saved;

    DistanceMeasure dism;
    Calibrater cali;
    
    tracker **trackers;
    IDDistributor *IDDistribute;
    TTC ttc;
    cv::Point2f camvelocity;

    std::vector<objects> QueueObject;
    objects objects_main;
    std::map<int, DETECTIONS> dets_with_frameID;
    float a_threshold, r_threshold;
};

#endif