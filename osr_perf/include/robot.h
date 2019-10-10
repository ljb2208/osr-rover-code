#ifndef ROBOT_H
#define ROBOT_H

#include <ros/ros.h>
#include "../include/motorcontrollers.h"

using namespace std;
using namespace ros;

#define PI 3.1418

class robot
{
public:
    robot();
    ~robot();

    // int32_t generateCommandsint32_tv,r,encs):
	void setDistances(float d1, float d2, float d3, float d4);
	void setEncoderValues(int* enc_min_vals, int* enc_max_vals);
	void generateCommands(int64_t velocity, int64_t radius, int32_t* enc, int32_t* vel_cmds, int32_t* tick);

private:
    int32_t tick2deg(int32_t tick,int32_t e_min,int32_t e_max);
    int32_t deg2tick(int32_t deg,int32_t e_min,int32_t e_max);
    void calculateTargetDeg(int32_t  radius, int32_t* angles);
	void calculateVelocity(int32_t velocity, int32_t radius, int32_t* cmds);
	void calculateTargetTick(int32_t* target_enc, int32_t* current_enc, int32_t* tick);

	MotorControllers mc;

	float d1, d2, d3, d4;
	int enc_min[4];
	int enc_max[4];
	int mids[4];

	static float rad2deg(float rad)
	{
		return rad * 180/PI;
	}

};
#endif
