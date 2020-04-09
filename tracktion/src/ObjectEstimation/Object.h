#ifndef OBJECT_H_
#define OBJECT_H_
#include"opencv2/video/tracking.hpp"

#include"DistanceMeasure.h"
#include"Calibration/Calibrater.h"

#include"dataType.h"
#include<map>


typedef struct objectttc
{
    bool ttc_existd = 0;
    double ttc_time = 109.9;

}ObjectTTC;


class object{
    
protected:
    enum ObjectState{NotMoving = 0, Closing, MovingFarAway, LateralCrossing, Undefined};
    enum MoveState{NotMove = 0, MoveClosing, MoveFarAway, Unknown};

    float lti,ltj,w,h;
    float mbi,mbj;
    float mti,mtj;
    float lbi,lbj;
    float rbi,rbj;
    float x,y,width,height;
    float conf;
    int id;
    bool kf_ini_flag;
    unsigned int  cls;
    unsigned int frame_id;

    float _std_weight_position = 0.1;
    float _std_weight_velocity = 0.1;
    float _std_weight_accelarate = 0.1;

    cv::Point2f vel;
    cv::Rect2f world;

    double head;
    double screen;

#ifdef DISTANCEMEASURE
    cv::Rect2f pixel2worldpixelonly(DistanceMeasure distancemeasure);
    cv::Rect2f pixel2worldwidthheight(DistanceMeasure distancemeasure);
    cv::Rect2f pixel2worldwidthlength(DistanceMeasure distancemeasure);
#else
    cv::Rect2f pixel2worldpixelonly(const HRYTCalibrater cali);
    cv::Rect2f pixel2worldwidthheight(const HRYTCalibrater cali);
    cv::Rect2f pixel2worldwidthlength(const HRYTCalibrater cali);
#endif

    //void filter_world(cv::Rect2f &world);

public:
    enum TransferType{WidthHeight, WidthLength, PixelOnly};
    ObjectState objstate = ObjectState::Undefined;
    MoveState movestate = MoveState::Unknown;

    object():lti(0), ltj(0), w(0), h(0), mbi(0), mbj(0), mti(0), mtj(0),
    lbi(0), lbj(0), rbi(0), rbj(0), x(0), y(0), width(0), height(0),
    id(0), kf_ini_flag(false), frame_id(0), head(0), screen(0){}

    std::map<int, float> Dislist;

#ifdef DISTANCEMEASURE
    void update(const RESULT_DATA det, DistanceMeasure dism, DistanceMeasure::TransferType worldtype, int frameindex);
#else
    //void update(const RESULT_DATA det, HRYTCalibrater cali, DistanceMeasure::TransferType worldtype, int frame_id);
#endif

    void ttc_update(const ObjectTTC obj_ttc){
        this->objttc = obj_ttc;}

    void cls_update(const unsigned int cl){
        this->cls = cl;}

    void frame_index_update(unsigned int frameindex){this->frame_id = frameindex;}

    void conf_update(float confi){this->conf = confi;}

    ObjectTTC objttc;

    ObjectTTC ttc() const{ return this->objttc; }

    unsigned int frame_index()const{return this->frame_id;}
    
    cv::Point2f midbottom() const{return cv::Point2f(mbi, mbj); }

    cv::Point2f midtop() const{return cv::Point2f(mti, mtj); }

    cv::Point2f leftbottom() const{return cv::Point2f(lbi, lbj);}

    cv::Point2f rightbottom() const{return cv::Point2f(rbi, rbj);}

    cv::Rect2f box() const{return cv::Rect2f(this->topleft(),this->size());}

    cv::Point2f topleft() const{return cv::Point2f(lti, ltj);}

    cv::Size size() const{ return cv::Size(w, h);}

    unsigned int objectclass() const{return this->cls;}

    cv::Rect2f worldposition()const{return this->world;}

    cv::Point2f velocity()const{return this->vel;}

    float confidence()const {return this->conf;}

    cv::Point2f vel_pix;
    unsigned int consistant_state_id = 0;

    double headposition() const{return this->worldposition().height*0.9;}
    double screenposition() const{return this->worldposition().height*0.4;}

    double get_distance() const {return this->worldposition().x * 1000;};
    MoveState get_movestate() const {return this->movestate;};
    void set_movestate(const MoveState& state) {this->movestate = state;};
};

typedef std::map<int, object> PEDOBJ;
// typedef std::map<int, object> RIDEROBJ;
typedef std::map<int, object> VEHOBJ;
typedef std::map<int, object> ROADOBJ;

class objects 
{
    public:
        PEDOBJ PedObj;
        // RIDEROBJ RiderObj;
        VEHOBJ VehObj;
        ROADOBJ RoadObj;
        unsigned int size() const{
            // return (PedObj.size()+RiderObj.size()+VehObj.size()+RoadObj.size());
            return (PedObj.size()+VehObj.size()+RoadObj.size());
        }
};


#endif