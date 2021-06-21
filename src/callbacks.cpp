#include "callbacks.h"
#include "common.h"

QueryCallback::QueryCallback(unsigned short id) : m_id(id), m_hit(false) {}

// True if contact with another body AABB
bool QueryCallback::ReportFixture(b2Fixture* fixture)
{
	uintptr_t f_data = fixture->GetUserData().pointer;
	FixtureData* fd = reinterpret_cast<FixtureData*>(f_data);
	if (fd->type == BODY && fd->body_id != m_id)
	{
		m_hit = true;
	}

	return true;
}

RayCastCallbackLow::RayCastCallbackLow() : m_fraction(1.0f), m_hit(false) {}

// Cast only against the arena and other robots
float RayCastCallbackLow::ReportFixture(b2Fixture* fixture, const b2Vec2& point,
                                        const b2Vec2& normal, float fraction)
{
	uintptr_t f_data = fixture->GetUserData().pointer;
	FixtureData* fd = reinterpret_cast<FixtureData*>(f_data);
	if (fd->type == BODY)
	{
		uintptr_t b_data = fixture->GetBody()->GetUserData().pointer;
		BodyData* bd = reinterpret_cast<BodyData*>(b_data);
		if (bd->type != ROBOT && bd->type != ARENA)
		{
			return -1.0f;
		}
	}
	else
	{
		return -1.0f;
	}

	m_hit = true;
	m_fraction = fraction;
	m_point = point;

	return fraction;
}

RayCastCallbackLift::RayCastCallbackLift() : m_fraction(1.0f), m_hit(false) {}

// Cast also against other loads
float RayCastCallbackLift::ReportFixture(b2Fixture* fixture, const b2Vec2& point,
                                         const b2Vec2& normal, float fraction)
{
	uintptr_t f_data = fixture->GetUserData().pointer;
	FixtureData* fd = reinterpret_cast<FixtureData*>(f_data);
	if (fd->type != BODY)
	{
		return -1.0f;
	}

	m_hit = true;
	m_fraction = fraction;
	m_point = point;

	return fraction;
}
