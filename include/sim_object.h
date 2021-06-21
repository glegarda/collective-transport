#ifndef SIM_OBJECT_H
#define SIM_OBJECT_H

#include "common.h"
#include <box2d/box2d.h>

class SimObject
{
	public:
		SimObject();
		~SimObject();
		b2Body* getBody();
		BodyData getBodyData();

	protected:
		b2Body* p_body;
		BodyData p_body_data;
		std::vector<FixtureData> p_fixture_data;
};

#endif
