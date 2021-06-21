#ifndef CALLBACKS_H
#define CALLBACKS_H

#include <box2d/box2d.h>

// Called before any ray casts are done to prevent unnecessary ray casts
// True if contact with another body AABB
class QueryCallback : public b2QueryCallback
{
	public:
		QueryCallback(unsigned short id);
		bool ReportFixture(b2Fixture* fixture);

		bool m_hit;
		unsigned short m_id;
};

// Called when a robot is not carrying a load
// Loads are ignored by cast rays
class RayCastCallbackLow : public b2RayCastCallback
{
	public:
		RayCastCallbackLow();
		float ReportFixture(b2Fixture* fixture, const b2Vec2& point,
    	                    const b2Vec2& normal, float fraction);

		bool m_hit;
		float m_fraction;
		b2Vec2 m_point;
};

// Called when a robot is carrying a load
// Collisions with other loads are also avoided
class RayCastCallbackLift : public b2RayCastCallback
{
	public:
		RayCastCallbackLift();
		float ReportFixture(b2Fixture* fixture, const b2Vec2& point,
    	                    const b2Vec2& normal, float fraction);

		bool m_hit;
		float m_fraction;
		b2Vec2 m_point;
};

#endif
