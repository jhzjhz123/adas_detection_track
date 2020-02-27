#include"trajectory.h"

DrivingPathEstimation::	DrivingPathEstimation(const float vehLength, const float vehWidth, 
    const float maxTireAngle,const float maxStirAngle){
    this->L = vehLength;
    this->K = vehWidth;	
    this->max_tire_angle = maxTireAngle;
    this->max_stir_angle = maxStirAngle;
    this->LastFrameID = 0;
}

cv::Point2f DrivingPathEstimation::PathEstimation(const float vel, const float stir_angle){
    float angle = stir_angle;
    if(stir_angle == 0) angle += 0.0000000000000001;
    this->inner_turn_angle = angle / this->max_stir_angle*this->max_tire_angle/180.*PI;	
    this->inner_turn_radius = L/(tan(this->inner_turn_angle));
    this->mid_turn_radius = this->inner_turn_radius + this->K/2.;
    this->mid_turn_angle = atan(this->L / this->mid_turn_radius);
    this->mid_turn_angle_rate = vel/this->mid_turn_radius;
    this->sx = this->mid_turn_radius * sin(this->mid_turn_angle_rate*dt);
    this->sy = this->mid_turn_radius * (1-cos(this->mid_turn_angle_rate*dt));
    return cv::Point2f(sx,sy);
}

float DrivingPathEstimation::CurrentAngleRate()const{
    return this->mid_turn_angle_rate/PI*180.;};

void DrivingPathEstimation::UpdateCurrentData(const float Velocity, 
const float StirAngle, const int UpdatedFrameID){
    this->dt = static_cast<float>((UpdatedFrameID - LastFrameID))/FRAMEFREQUENCY;

    cv::Point2f sp = this->PathEstimation(Velocity, StirAngle);
    float currentAngle = this->CurrentAngleRate();

    trajectory.currentAngle.push_back(currentAngle);
    trajectory.updatePoints.push_back(sp);

    if(trajectory.currentAngle.size() > TRAJECTORYSIZE){
        trajectory.updatePoints.erase(trajectory.updatePoints.begin());
        trajectory.currentAngle.erase(trajectory.currentAngle.begin());
        }

    this->currentcoordinate = 0;
    this->LastFrameID = UpdatedFrameID;
}

std::map<int ,cv::Point2f> DrivingPathEstimation::UpdateCurrentTrajectory(){
    cv::Point2f P;
    std::map<int, cv::Point2f> W;
    int frameid = this->LastFrameID;
    for (auto it = trajectory.currentAngle.end(); it != trajectory.currentAngle.begin(); it--)
    {
        this->currentcoordinate += *(it-1)*dt/180*PI;

        cv::Point2f updatePoints;
        unsigned int index = trajectory.currentAngle.end() - it;

        updatePoints.x = trajectory.updatePoints[index].x*cos(this->currentcoordinate) - 
        trajectory.updatePoints[index].y * sin(this->currentcoordinate);

        updatePoints.y = trajectory.updatePoints[index].x*sin(this->currentcoordinate) + 
        trajectory.updatePoints[index].y * cos(this->currentcoordinate);

        P -= updatePoints;
        W.insert(std::make_pair(frameid--, P));
    }
    //std::cout << this->dt << std::endl;
    return W;
}