#include <iostream>
#include <fstream>
#include <string>
#include "bt_custom.h"
#include "utils.h"
#include "common.h"
#include "environment.h"

// To randomise the initial conditions
std::string bt_exp = R"(
<root main_tree_to_execute="MainTree">
	<BehaviorTree ID="MainTree">
		<ReactiveFallback>
			<ReactiveSequence>
				<Ifsect arg0="{vprox}" arg1="0" arg2="127"/>
				<Mulav arg0="{vscr}" arg1="{vzero}" arg2="-1.000" arg3="{vprox}"/>
				<Movpv arg0="{vvote}" arg1="{vscr}" arg2="64"/>
			</ReactiveSequence>
			<Movcv arg0="{vvote}" arg1="0"/>
		</ReactiveFallback>
	</BehaviorTree>
</root>
)";

int main(int argc, char* argv[])
{
	// Process input arguments
	if (argc < 2 || argc > 4)
	{
		std::cout << "Incorrect call. Format is\n"
		          << "    ./sim <path_to_xml_file> [duration=3600] [render=1]"
		          << std::endl;
		return -1;
	}

	// Controller
	std::ifstream ifs(argv[1]);
	std::string bt((std::istreambuf_iterator<char>(ifs)),
	               (std::istreambuf_iterator<char>()   ));

	// Duration and render
	unsigned long long t_sim = 3600;
	bool render = true;
	if (argc > 2)
	{
		t_sim = std::atoll(argv[2]);

		if (argc > 3)
		{
			render = std::atoi(argv[3]);
		}
	}

	// Initialise environment
	Environment env;
	
	env.addArena();

/*
	env.addLoad(b2Vec2(0.0f, 0.0f), 2);
	env.addRobot(b2Vec2(0.05f, -0.500f), M_PI_2);
	env.addRobot(b2Vec2(-0.05f, 1.000f), -M_PI_2);
*/

	env.addLoad(b2Vec2(-1.5f,  1.25f), 2);
	env.addLoad(b2Vec2(-1.5f,  0.00f), 2);
	env.addLoad(b2Vec2(-1.5f, -1.25f), 2);

	env.addRobot(b2Vec2(2.225f,  0.600f), rndf(-M_PI, M_PI));
	env.addRobot(b2Vec2(2.225f,  0.200f), rndf(-M_PI, M_PI));
	env.addRobot(b2Vec2(2.225f, -0.200f), rndf(-M_PI, M_PI));
	env.addRobot(b2Vec2(2.225f, -0.600f), rndf(-M_PI, M_PI));
	env.addRobot(b2Vec2(1.825f,  0.600f), rndf(-M_PI, M_PI));
	env.addRobot(b2Vec2(1.825f,  0.200f), rndf(-M_PI, M_PI));
	env.addRobot(b2Vec2(1.825f, -0.200f), rndf(-M_PI, M_PI));
	env.addRobot(b2Vec2(1.825f, -0.600f), rndf(-M_PI, M_PI));
	env.addRobot(b2Vec2(1.425f,  0.600f), rndf(-M_PI, M_PI));
	env.addRobot(b2Vec2(1.425f,  0.200f), rndf(-M_PI, M_PI));
	env.addRobot(b2Vec2(1.425f, -0.200f), rndf(-M_PI, M_PI));
	env.addRobot(b2Vec2(1.425f, -0.600f), rndf(-M_PI, M_PI));
	env.addRobot(b2Vec2(1.025f,  0.600f), rndf(-M_PI, M_PI));
	env.addRobot(b2Vec2(1.025f,  0.200f), rndf(-M_PI, M_PI));
	env.addRobot(b2Vec2(1.025f, -0.200f), rndf(-M_PI, M_PI));
	env.addRobot(b2Vec2(1.025f, -0.600f), rndf(-M_PI, M_PI));

	Robot* robot = nullptr;
	// Initial exploration
	while (robot = env.getNextRobot())
	{
		robot->controller.xml_tree = bt_exp;
		robot->controller.updateSource();
	}

	float null_fitness = env.run(300);

	// Set controllers of all the robots in the environment
	while (robot = env.getNextRobot())
	{
		robot->controller.xml_tree = bt;
		robot->controller.updateSource();
	}

	// Run simulation and compute controller fitness
	env.render = render;
	float fitness = env.run(t_sim);

	std::cout << fitness << std::endl;

	return 0;
}
