#include <cmath>
#include <random>
#include "utils.h"

void sleep(sctime& start, float duration)
{
	while (true)
	{
		sctime end = sclock::now();
		std::chrono::duration<float> diff = end - start;
		if (diff.count() >= duration)
		{
			start = sclock::now();
			break;
		}
	}
}

VectorPolar::VectorPolar() : r(0.0f), a(0.0f) {}

VectorPolar::VectorPolar(float radius, float azimuth) : r(radius), a(azimuth)
{
	toMpiPi();
}

VectorPolar::VectorPolar(const VectorPolar& v) : r(v.r), a(v.a)
{
	toMpiPi();
}

VectorPolar::VectorPolar(const b2Vec2& v) :
	r(v.Length()),
	a(std::atan2(v.y, v.x)) {}

VectorPolar::~VectorPolar() {}

void VectorPolar::rotate(float angle)
{
	a += angle;
	toMpiPi();
}

b2Vec2 VectorPolar::to_b2Vec2()
{
	return b2Vec2(r * std::cos(a), r * std::sin(a));
}

VectorPolar VectorPolar::operator+(const VectorPolar& v) const
{
	float x = r * std::cos(a) + v.r * std::cos(v.a);
	float y = r * std::sin(a) + v.r * std::sin(v.a);
	VectorPolar tmp;
	tmp.r = std::sqrt(x * x + y *y);
	tmp.a = std::atan2(y, x);
	return tmp;
}

VectorPolar VectorPolar::operator+=(const VectorPolar &v)
{
	float x = r * std::cos(a) + v.r * std::cos(v.a);
	float y = r * std::sin(a) + v.r * std::sin(v.a);
	this->r = std::sqrt(x * x + y * y);
	this->a = std::atan2(y, x);
	return *this;
}

VectorPolar VectorPolar::operator-(const VectorPolar& v) const
{
	float x = r * std::cos(a) - v.r * std::cos(v.a);
	float y = r * std::sin(a) - v.r * std::sin(v.a);
	VectorPolar tmp;
	tmp.r = std::sqrt(x * x + y *y);
	tmp.a = std::atan2(y, x);
	return tmp;
}

VectorPolar VectorPolar::operator-=(const VectorPolar &v)
{
	float x = r * std::cos(a) - v.r * std::cos(v.a);
	float y = r * std::sin(a) - v.r * std::sin(v.a);
	this->r = std::sqrt(x * x + y * y);
	this->a = std::atan2(y, x);
	return *this;
}

VectorPolar VectorPolar::operator*(const float& f) const
{
	if (f < 0.0f)
	{
		return VectorPolar(-f * r, a + M_PI);
	}

	return VectorPolar(f * r, a);
}

std::ostream& operator<<(std::ostream& os, const VectorPolar& v)
{
	os << "[" << v.r << "," << v.a << "]";
	return os;
}

void VectorPolar::toMpiPi()
{
	while (a >= M_PI)
	{
		a -= (2.0f * M_PI);
	}

	while (a < -M_PI)
	{
		a += (2.0f * M_PI);
	}
}

// ** Should try to put these somewhere safer... ** //
std::random_device rd;
std::mt19937 gen(rd());

// Random float generator in the range [fmin, fmax)
float rndf(float fmin, float fmax)
{
    std::uniform_real_distribution<float> dis(fmin, fmax);
    return dis(gen);
}

// Random int generator in the range [imin, imax]
int rndi(int imin, int imax)
{
    std::uniform_int_distribution<int> dis(imin, imax);
    return dis(gen);
}

// Random int generator in the range [imin, imax]
float rndif(int imin, int imax)
{
    std::uniform_int_distribution<int> dis(imin, imax);
    return static_cast<float>(dis(gen));
}
