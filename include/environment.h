#ifndef ENVIRONMENT_H
#define ENVIRONMENT_H

#include <vector>
#include <memory>
#include "arena.h"
#include "robot.h"
#include "load.h"
#include "draw.h"

template<class T>
using uvector = std::vector<std::unique_ptr<T> >;

class Environment
{
	public:
		Environment();
		~Environment();

		void addArena();
		void addRobot(const b2Vec2& position, float angle);
		void addLoad(const b2Vec2& position, unsigned short porters);
		Robot* getNextRobot();
		float run(unsigned long long duration);

		bool render;

	private:
		friend class Arena;
		friend class Robot;
		friend class Load;

		void destroyLoad(Load* load);
		void resetLoad(unsigned short id);

		void sense();
		void commsOut();
		void commsIn();
		void control();
		void act();
		void renderSim();

		b2World* p_world;
		std::unique_ptr<Arena> p_arena;
		uvector<Robot> p_robots;
		uvector<Load> p_loads;
		std::size_t p_next;
		float p_f_sim;
		float p_fitness;
		GLFWwindow* p_window;
};

#endif
