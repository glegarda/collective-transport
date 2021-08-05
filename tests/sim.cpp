#include <iostream>
#include <fstream>
#include <string>
#include "bt_custom.h"
#include "utils.h"
#include "common.h"
#include "environment.h"

// The following BT is executed
std::string bt = R"(
<root main_tree_to_execute="MainTree">
	<BehaviorTree ID="MainTree">
		<AlwaysSuccess/>
	</BehaviorTree>
</root>
)";

// To randomise the initial conditions
std::string bt_exp = R"(
<root main_tree_to_execute="MainTree">
	<BehaviorTree ID="MainTree">
		<!-- EXPLORATION -->
		<ReactiveFallback>
			<ReactiveSequence>
				<Ifsect arg0="{vprox}" arg1="0" arg2="127"/>
				<Mulav arg0="{vscr}" arg1="{vzero}" arg2="-1.000" arg3="{vprox}"/>
				<Movpv arg0="{vvote}" arg1="{vscr}" arg2="64"/>
			</ReactiveSequence>
			<Movcv arg0="{vvote}" arg1="0"/>
		</ReactiveFallback>
		<!----------------->
	</BehaviorTree>
</root>
)";

int main()
{
	// Initialise environment
	Environment env;
	
	env.addArena();

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
	while ((robot = env.getNextRobot()))
	{
		robot->controller.xml_tree = bt_exp;
		robot->controller.updateSource();
	}

	// Run simulation and compute controller fitness
	env.render = false;
    env.new_prox = true;
	float fitness = env.run(3600);

	std::cout << fitness << std::endl;

	return 0;
}
