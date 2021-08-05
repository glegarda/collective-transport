#ifndef UTILS_H
#define UTILS_H

#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"

#include <iostream>
#include <cmath>
#include <chrono>
#include <vector>
#include <box2d/box2d.h>

#define M_EPSILON 0.001

using sclock = std::chrono::steady_clock;
using sctime = std::chrono::time_point<std::chrono::steady_clock>;

double delta(sctime t1, sctime t2);

// Chronometer
void sleep(sctime& start, float duration);

// Vector in polar coordinates
class VectorPolar {
	public:
		float r;
		float a;
		VectorPolar();
		VectorPolar(float radius, float azimuth);
		VectorPolar(const VectorPolar& v);
		VectorPolar(const b2Vec2& v);
		~VectorPolar();
		void rotate(float angle);
		b2Vec2 to_b2Vec2();
		VectorPolar operator+(const VectorPolar& v) const;
		VectorPolar operator+=(const VectorPolar& v);
		VectorPolar operator-(const VectorPolar& v) const;
		VectorPolar operator-=(const VectorPolar& v);
		VectorPolar operator*(const float& f) const;
		friend std::ostream& operator<<(std::ostream& os, const VectorPolar& v);
	private:
		void toMpiPi();
};

// Random number generators
float rndf(float fmin, float fmax);
int rndi(int imin, int imax);
float rndif(int imin, int imax);

// Vector mean and covariance (for numeric data types)
template<typename T>
float mean(const std::vector<T>& v)
{
	float cum_sum = 0.0f;
	for (const auto& elem : v)
	{
		cum_sum += static_cast<float>(elem);
	}
	return (cum_sum / v.size());
}

template<typename T1, typename T2>
float covariance(const std::vector<T1>& v1, const std::vector<T2>& v2)
{
	float e1 = mean<T1>(v1);
	float e2 = mean<T2>(v2);

	float cum_cov = 0.0f;
	std::size_t n = v1.size();
	for (auto i = 0; i < n; ++i)
	{
		float c1 = static_cast<float>(v1.at(i)) - e1;
		float c2 = static_cast<float>(v2.at(i)) - e2;
		cum_cov += (c1 * c2); 
	}

	return (cum_cov / n);
}

#endif
