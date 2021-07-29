#ifndef LOAD_H
#define LOAD_H

#include "sim_object.h"

class Load : public SimObject
{
	public:
		Load(b2World* world, const b2Vec2& position, unsigned short porters);
		~Load();

		void destroy(b2World* world);
		unsigned short getPorters();
		b2Vec2 getStartPosition();

		unsigned long long t_life;
		unsigned long long t_cover;
		bool lifted;
		bool lowered;

	private:
		unsigned short p_porters;
		static short p_next_id;
		b2Vec2 p_start_position;
};

#endif
