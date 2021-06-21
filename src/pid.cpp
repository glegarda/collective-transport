#include "pid.h"

PID::PID() {}

PID::PID(float kp, float ki, float kd, float dt,
         const std::string& name) :
	p_kp(kp),
	p_ki(ki),
	p_kd(kd),
	p_dt(dt),
	p_integral(0.0f),
	p_prev_error(0.0f)
{
	if (!name.empty())
	{
		std::string sname = name + ".txt";
		p_log.open(sname);
		p_log << "yr,ym\n";
	}
}

PID::~PID() {}

float PID::update(float yr, float ym)
{
	float error = yr - ym;

	// Proportional
	float p = p_kp * error;

	// Integral
	p_integral += error * p_dt;
	float i = p_ki * p_integral;

	// Derivative
	float d_error = (error - p_prev_error) / p_dt;
	float d = p_kd * d_error;
	p_prev_error = error;

	// Print to log
	if(p_log.is_open())
	{
		p_log << yr << "," << ym << "\n";
	}

	return (p + i + d);
}
