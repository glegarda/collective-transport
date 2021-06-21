#ifndef ARENA_H
#define ARENA_H

#include "sim_object.h"

class Arena : public SimObject
{
	public:
		Arena();
		Arena(b2World* world);
		~Arena();
};

#endif
