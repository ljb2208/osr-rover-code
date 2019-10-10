#include "../include/robot.h"

robot::robot()
{
	mc.connect();
}

robot::~robot()
{

}

void robot::setDistances(float d1, float d2, float d3, float d4)
{
	this->d1 = d1;
	this->d2 = d2;
	this->d3 = d3;
	this->d4 = d4;
}

void robot::setEncoderValues(int* enc_min_vals, int* enc_max_vals)
{
	for (int i=0; i < 4; i++)
	{
		enc_min[i] = enc_min_vals[i];
		enc_max[i] = enc_max_vals[i];

		mids[i] = (enc_max[i] + enc_min[i]) / 2;
	}
}

void robot::generateCommands(int64_t velocity, int64_t radius, int32_t* enc, int32_t* vel_cmds, int32_t* tick)
{
	calculateVelocity(velocity, radius, vel_cmds);
	int32_t angles[4];
	calculateTargetDeg(radius, angles);
	calculateTargetTick(angles, enc, tick);
}

void robot::calculateVelocity(int32_t velocity, int32_t radius_input, int32_t* cmds)
{
	if (velocity == 0)
	{
		for (int i=0; i < 6; i++)
			cmds[i] = 0;

		return;
	}

	if (abs(radius_input) <= 5)
	{
		for (int i=0; i < 6; i++)
			cmds[i] = velocity;

		return;
	}

	int32_t radius = 250 - (230 * abs(radius_input))/100.0;

	int32_t rmax = 1;

	if (radius < 0)
		 rmax *= -1;
	
	rmax = radius + d4;

	float a = pow(d2,2);
	float b = pow(d3,2);
	float c = pow(abs(radius) + d1,2);
	float d = pow(abs(radius) - d1,2);
	float e = abs(radius) - d4;
	float rmax_float = float(rmax);

	int32_t v1 = int(velocity*(sqrt(b + d))/rmax_float);
	int32_t v2 = int((velocity*e/rmax_float));             // Slowest wheel
	int32_t v3 = int((velocity*sqrt(a + d))/rmax_float);
	int32_t v4 = int((velocity*sqrt(a + c))/rmax_float);
	int32_t v5 = int(velocity);                            // Fastest wheel
	int32_t v6 = int((velocity*sqrt(b + c))/rmax_float);

	if (radius_input < 0)
	{
		cmds[0] = v1;
		cmds[1] = v2;
		cmds[2] = v3;
		cmds[3] = v4;
		cmds[4] = v5;
		cmds[5] = v6;
	}
	else
	{
		cmds[0] = v6;
		cmds[1] = v5;
		cmds[2] = v4;
		cmds[3] = v3;
		cmds[4] = v2;
		cmds[5] = v1;

	} 

}


int32_t robot::tick2deg(int32_t tick,int32_t e_min,int32_t e_max)
{
	return 0;
}

int32_t robot::deg2tick(int32_t deg,int32_t e_min,int32_t e_max)
{
	//param int deg  : Degrees value desired
	//param int e_min: The minimum encoder value based on physical stop
 	// param int e_max: The maximum encoder value based on physical stop

	int32_t temp = (e_max + e_min)/2 + ((e_max - e_min)/90)*deg;

	if (temp < e_min)
		temp = e_min;
	else if (temp > e_max)
		temp = e_max;

	return temp;
}

void robot::calculateTargetTick(int32_t* target_enc, int32_t* current_enc, int32_t* tick)
{
	//Takes the target angle and gets what encoder tick that value is for position control

	//param list [int] tar_enc: List of target angles in degrees for each corner
	for (int i=0; i < 4; i++)
	{
		tick[i] = deg2tick(target_enc[i], enc_min[i], enc_max[i]);

		if (abs(tick[i] -current_enc[i]) < 25)
		{
			tick[i] = -1;
		}
	}
}

void robot::calculateTargetDeg(int32_t  radius, int32_t* angles)
{
	int32_t r = 0;

	if (radius == 0)
		r = 250;
	else if (radius >= -100 && radius <= 100)
		r = 220 - abs(radius)*(250/100);
	else
		r = 250;

	if (r == 250)
	{
		angles[0] = 0;
		angles[1] = 0;
		angles[2] = 0;
		angles[3] = 0;
		return;
	}
	
	int32_t ang1 = int(rad2deg(atan(d1/(abs(r)+d3))));
	int32_t ang2 = int(rad2deg(atan(d2/(abs(r)+d3))));
	int32_t ang3 = int(rad2deg(atan(d2/(abs(r)-d3))));
	int32_t ang4 = int(rad2deg(atan(d1/(abs(r)-d3))));

	if (radius > 0)
	{
		angles[0] = ang2;
		angles[1] = -ang1;
		angles[2] = -ang4;
		angles[3] = ang3;
	}
	else
	{
		angles[0] = -ang4;
		angles[1] = ang3;
		angles[2] = ang2;
		angles[3] = -ang1;
	}
}

