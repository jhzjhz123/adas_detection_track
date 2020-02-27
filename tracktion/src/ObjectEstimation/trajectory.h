#ifndef TRAJECTORY_H_
#define TRAJECTORY_H_

#include<vector>
#include<map>
#include<opencv2/highgui.hpp>
#include"../Tools/Tools.h"
#include"../dataType.h"

#define PI 3.1415926
#define TRAJECTORYSIZE 64

struct Trajectory
{
	std::vector<float> currentAngle;
	std::vector<cv::Point2f> updatePoints;
};

class DrivingPathEstimation{
public:	
	/**
	* @biref initialization for the veh
	* @param vehlegth -length betwwen front and back tire
	* @param vehWidth -length betwwen front or back tire
	* @param maxTireAngle -maximun angle the tire can reach
	* @param maxStirAngle -maximun angle the stiring can reach
	* @return none	
	*/
	DrivingPathEstimation(const float vehLength, const float vehWidth, 
    const float maxTireAngle,const float maxStirAngle);
	/**
	* @biref path estimation for the veh
	* @param vel -veh velocity
	* @param stir_angle
	* @param t time
	* @return Position estimated
	*/
	cv::Point2f PathEstimation(const float vel, const float stir_angle);

	float CurrentAngleRate()const;

	/**
	* @biref update current data
	* @param Velocity -veh velocity
	* @param StirAngle -veh stir angle
	* @return none	
	*/
	void UpdateCurrentData(const float Velocity, const float StirAngle, const int UpdatedFrameID);

	/**
	* @biref update current trajectory
	* @return none	
	*/
	std::map<int ,cv::Point2f> UpdateCurrentTrajectory();

    cv::Point2f CamVelocity()const{return this->CurrentPoints/this->dt;};

private:
	float inner_turn_radius;
	float inner_turn_angle;
	float mid_turn_radius;
	float mid_turn_angle;
	float mid_turn_angle_rate;
	float L; // veh length
	float K; // veh width
	float sx,sy;	
	float max_tire_angle; 
	float max_stir_angle;
	Trajectory trajectory;
	float currentcoordinate;
	float definitecoordinate;
	int	LastFrameID;
	float dt;
	cv::Point2f CurrentPoints;
};


#endif