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


CommandLineParams g_cmd;

int main(int argc, char* argv[])
{
	// Process input arguments
    process_args(argc, argv, g_cmd);

	// Set up controller
    std::string bt;
    if (g_cmd.treefile.size())
    {
        std::ifstream ifs(g_cmd.treefile);
        std::string _bt((std::istreambuf_iterator<char>(ifs)),
                        (std::istreambuf_iterator<char>()   ));
        bt = _bt;
    }
    else
        bt = bt_exp;

    
    // Initialise environment
    Environment env(g_cmd);
    
	// Duration
	unsigned long long t_sim = g_cmd.simtime * env.p_f_sim;
	
	env.addArena();

	env.addLoad(b2Vec2(-1.5f,  1.25f), 2, 0.0f);
	env.addLoad(b2Vec2(-1.5f,  0.00f), 2, 0.0f);
	env.addLoad(b2Vec2(-1.5f, -1.25f), 2, 0.0f);
    
    if (g_cmd.rand_position)
    {
        env.addRandomRobots();
    }
    else
    {
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
    }
    
	Robot* robot = nullptr;
	while ((robot = env.getNextRobot()))
	{
		robot->controller.xml_tree = bt;
		robot->controller.updateSource();
	}

    // Run for 
	float fitness = env.run(t_sim);

	std::cout << fitness << std::endl;

	return 0;
}
