#ifndef TTC_H_
#define TTC_H_

#include <iostream>
#include <math.h>
#include <map>
#include <deque>
#include <Eigen/Core>
#include <Eigen/Dense>
#include"Object.h"
// #include "opencv2/imgproc/imgproc.hpp"
// #include "opencv2/highgui/highgui.hpp"

#include<iomanip>
#define TTCTRACKINGFRAMES 11
#define VEHICLEWIDTH 1.8 //单位米
// #define ALPHATIME 33.333 //单位ms，两张图片间隔时间
#define CAMERALEFT 0.9 //据左侧的距离 单位米


// 采用像素变化进行ttc计算
class objects;
class TTC
{
typedef std::deque<PEDOBJ> PedQueue;
// typedef std::deque<RIDEROBJ> RiderQueue;
typedef std::deque<VEHOBJ> VehQueue;
typedef std::deque<ROADOBJ> RoadQueue;

private:
    double alpha;
    /* objects queue list */
    PedQueue _ped_queue;
    // RiderQueue _rider_queue;
    VehQueue _veh_queue;
    RoadQueue _road_queue;


    /**
     * @brief 输入某类目标连续多帧的数据，通过计算更新该类 每个目标的ttc。
     * @param Tqueue - queuelist
     * @param Tobj - 当前帧的某类目标，比如所有的人，所有的车等。
     * @return none
    */
    template<typename Tqueue, typename Tobj>
    void FetchDataFromQueue(Tqueue &queue,Tobj &tobj);

    /**
     * @brief 输入某类目标连续多帧的数据，检测每个小目标物是否处于靠近状态，如果是，计算更新ttc。
     * @param Tqueue - queuelist
     * @return none
    */
    template<typename Tqueue>
    void FindObjectToUpdate(Tqueue &queue);

    /**
     * @brief 输入某类目标连续多帧的数据，以及需要计算ttc的id号。计算更新ttc。
     * @param Tqueue - queuelist
     * @Tint - 某类目标的某个id号。遍历最新一帧的所有id号
     * @return 返回ttc计算结果：是否存在以及ttc时间
    */
    template<typename Tqueue, typename Tint>
    ObjectTTC update(Tqueue &queue, Tint &frameid);


public:

    TTC():alpha(1e-8){ }

    /**
    *@breif 外部函数调用ttc的开关，计算所有目标的ttc
    *@param Tobj - 某帧检测到的所有类别的所有小目标
    *@return - none 
    */
    void TTCCalculate(objects &objs);
}; 

#endif