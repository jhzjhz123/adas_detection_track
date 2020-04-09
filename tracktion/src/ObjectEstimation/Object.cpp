
#include"Object.h"

#ifdef DISTANCEMEASURE
void object::update(const RESULT_DATA det, DistanceMeasure dism,
                        DistanceMeasure::TransferType worldtype, int frame_id){
    id = det.first;
    DETECTBOX detbox = det.second;
    lti = detbox(0);
    ltj = detbox(1);
    w = detbox(2);
    h = detbox(3);
    mbi = lti + w/2.;
    mbj = ltj + h;
    mti = lti + w/2.;
    mtj = ltj;
    lbi = lti;
    lbj = ltj + h;
    rbi = lti + w;
    rbj = ltj + h;
    switch (worldtype)
    {
    case DistanceMeasure::TransferType::WidthHeight:
        world = this->pixel2worldwidthheight(dism);
        break;
    case DistanceMeasure::TransferType::WidthLength:
        world = this->pixel2worldwidthlength(dism);
        break;
    case DistanceMeasure::TransferType::PixelOnly:
        world = this->pixel2worldpixelonly(dism);
        break;            
    }
    this->frame_index_update(frame_id);
    this->filter_world(world);
}

cv::Rect2f object::pixel2worldpixelonly(DistanceMeasure distancemeasure){
    cv::Point2f midbottom = this->midbottom();
    cv::Size size = cv::Size(0,0);
    cv::Rect2f rect(midbottom, size);
    cv::Rect2f w;
    distancemeasure.pixel2worldpixelonly(rect, w);
    return w;
}

cv::Rect2f object::pixel2worldwidthheight(DistanceMeasure distancemeasure){
    cv::Point2f topleft = this->topleft();
    cv::Size size = this->size();
    cv::Rect2f rect(topleft, size);
    cv::Rect2f w;
    distancemeasure.pixel2worldwidthheight(rect, w);
    return w;            
}

cv::Rect2f object::pixel2worldwidthlength(DistanceMeasure distancemeasure){
    cv::Point2f topleft = this->topleft();
    cv::Size size = this->size();
    cv::Rect2f rect(topleft, size);
    cv::Rect2f w;
    distancemeasure.pixel2worldwidthlength(rect, w);
    return w;            
}

void object::filter_world(cv::Rect2f &world){
    Mat measurement = Mat::zeros(2,1, CV_32F);
    measurement.at<float>(0) = world.x;
    measurement.at<float>(1) = world.y;

    if(!this->kf_ini_flag){
        this->kf->statePost = (cv::Mat_<float>(4,1) << measurement.at<float>(0), measurement.at<float>(1), 0,0);
        this->kf_ini_flag = true;
    }
    Mat InputState = (Mat_<float>(4,1) << 
        2 * _std_weight_position * measurement.at<float>(1),
        2 * _std_weight_position * measurement.at<float>(1),
        10 * _std_weight_velocity * measurement.at<float>(1),
        10 * _std_weight_velocity * measurement.at<float>(1));

    Mat InputSMeas = (Mat_<float>(2,1) << 
        2 * _std_weight_position * measurement.at<float>(1),
        2 * _std_weight_position * measurement.at<float>(1));
    // cout << InputState << endl;
    this->kf->processNoiseCov = Mat::diag(InputState.mul(InputState)); // 4*4
    this->kf->measurementNoiseCov = Mat::diag(InputSMeas.mul(InputSMeas)); // 2*2

    Mat prediction = this->kf->predict();
    this->kf->correct(measurement);
    this->world.x = this->kf->statePost.at<float>(0);
    this->world.y = this->kf->statePost.at<float>(1);
    this->vel.x = this->kf->statePost.at<float>(2);
    this->vel.y = this->kf->statePost.at<float>(3);   

}
#else

// void object::update(const RESULT_DATA det, HRYTCalibrater cali,
//                         DistanceMeasure::TransferType worldtype, int frame_id){
//     id = det.first;
//     DETECTBOX detbox = det.second;
//     lti = detbox(0);
//     ltj = detbox(1);
//     w = detbox(2);
//     h = detbox(3);
//     mbi = lti + w/2.;
//     mbj = ltj + h;
//     mti = lti + w/2.;
//     mtj = ltj;
//     lbi = lti;
//     lbj = ltj + h;
//     rbi = lti + w;
//     rbj = ltj + h;
//     switch (worldtype)
//     {
//     case DistanceMeasure::TransferType::WidthHeight:
//         world = this->pixel2worldwidthheight(cali);
//         break;
//     case DistanceMeasure::TransferType::WidthLength:
//         world = this->pixel2worldwidthlength(cali);
//         break;
//     case DistanceMeasure::TransferType::PixelOnly:
//         world = this->pixel2worldpixelonly(cali);
//         break;            
//     }
//     this->frame_index_update(frame_id);
//     this->filter_world(world);
// }

cv::Rect2f object::pixel2worldpixelonly(const HRYTCalibrater cali){
    cv::Point2f midbottom = this->midbottom();
    cv::Point2f w;
    cali.PixelPointToHRYTGroundPointOffline(midbottom,w);


    cv::Rect2f WorldBox;
    WorldBox.x = w.x;WorldBox.y = w.y;
    return WorldBox;
}

cv::Rect2f object::pixel2worldwidthheight(const HRYTCalibrater cali){
    cv::Point2f left = this->leftbottom();
    cv::Point2f right = this->rightbottom();
    cv::Point2f bottom = this->midbottom();
    cv::Point2f leftw,rightw,bottomw;
    cv::Point3f topw;

    cali.PixelPedestrainBBoxToHRYTHeadPointOffline(topw,this->box());
    cali.PixelPointToHRYTGroundPointOffline(leftw,left);
    cali.PixelPointToHRYTGroundPointOffline(rightw,right);
    cali.PixelPointToHRYTGroundPointOffline(bottomw,bottom);

    cv::Rect2f w;
    w.x = bottomw.x;
    w.y = bottomw.y;
    w.width = abs(rightw.y - leftw.y);
    w.height = abs(topw.z);

    return w;            
}

cv::Rect2f object::pixel2worldwidthlength(const HRYTCalibrater cali){
    cv::Point2f midtopw, midbottomw, leftw, rightw;
    cv::Point2f midtop = this->midtop();
    cv::Point2f midbottom = this->midbottom();
    cv::Point2f left = this->leftbottom();
    cv::Point2f right = this->rightbottom();

    cali.PixelPointToHRYTGroundPointOffline(midtopw,midtop);
    cali.PixelPointToHRYTGroundPointOffline(midbottomw,midbottom);
    cali.PixelPointToHRYTGroundPointOffline(leftw,left);
    cali.PixelPointToHRYTGroundPointOffline(rightw,right);

    Rect2f w;
    w.x = midbottomw.x;
    w.y = midbottomw.y;
    w.height = midtopw.x - midbottomw.x;
    w.width = abs(leftw.y - rightw.y);

    return w;            
}

void boundaryset(cv::Mat &statePost){
    assert(statePost.size() == Size(1,6));
    float *velocity_x = &statePost.at<float>(2);
    float *velocity_y = &statePost.at<float>(3);
    float *accelarate_x  = &statePost.at<float>(4);
    float *accelarate_y = &statePost.at<float>(5);
    if(*velocity_x > 30) *velocity_x = 30;
    if(*velocity_x < -60) *velocity_x = -60;
    if(*velocity_y > 30) *velocity_y = 30;
    if(*velocity_y < -30) *velocity_y = -30;
    if(*accelarate_x > 3) *accelarate_x = 3;
    if(*accelarate_x < -6) *accelarate_x = -6;
    if(*accelarate_y > 3) *accelarate_y = 3;
    if(*accelarate_y < -3) *accelarate_y = -3;
}



#endif
