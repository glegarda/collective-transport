#ifndef PID_H
#define PID_H

#include <iostream>
#include <fstream>
#include <string>
#include <cmath>

class PID
{
	public:
		PID();
		PID(float kp, float ki, float kd, float dt,
		    const std::string& name);
		~PID();

		float update(float yr, float ym);

	private:
		float p_kp;
		float p_ki;
		float p_kd;
		float p_dt;
		float p_integral;
		float p_prev_error;

		std::ofstream p_log;
};

#endif
