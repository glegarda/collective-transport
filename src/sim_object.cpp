#include "sim_object.h"

SimObject::SimObject() :
	p_body(nullptr),
	p_body_data() {}

SimObject::~SimObject()
{
	p_body = nullptr;	
}

b2Body* SimObject::getBody()
{
	return p_body;
}

BodyData SimObject::getBodyData()
{
	return p_body_data;
}
