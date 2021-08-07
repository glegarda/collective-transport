// Gene -> single behaviour tree
// Chromosome -> gene and costs
// Generation -> set of chromosomes

#include <map>
#include <string>
#include <fstream>
#include "utils.h"
#include "openGA.hpp"
#include "bt_custom.h"
#include "environment.h"
#include "robot.h"
#include <behaviortree_cpp_v3/blackboard.h>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/loggers/bt_cout_logger.h>


CommandLineParams g_cmd;


struct EvoParams
{
	unsigned long long t_sim = 3600; // 2 minutes
	unsigned long long t_exp = 300; // 10 seconds
	unsigned int population = 20; // 20
	unsigned short min_max_depth = 1;
	unsigned short max_max_depth = 5;
	unsigned short n_evals = 4;
	// Make sure mutation rate is set to 1.0 in main()
	float p_inner = 0.9;
	float p_mut_param = 0.05;
	float p_mut_point = 0.05;
	float p_mut_subtree = 0.1;
} EvoParams;

struct ExecutionXML {
	// Behaviours
	std::string exploration = R"(
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
	)";

	std::string attraction = R"(
	<!-- ATTRACTION -->
	<ReactiveSequence>
		<ReactiveFallback>
			<ReactiveSequence>
				<Ifprob arg0="{sn}" arg1="15.000" arg2="0.500"/>
				<Mulav arg0="{vscr}" arg1="{vzero}" arg2="%s" arg3="{vattr}"/>
			</ReactiveSequence>
			<Movcv arg0="{vscr}" arg1="0"/>
		</ReactiveFallback>
		<Mulav arg0="{vvote}" arg1="{vscr}" arg2="-5.000" arg3="{vprox}"/>
	</ReactiveSequence>
	<!---------------->
	)";

	std::string recruitment = R"(
	<!-- RECRUITMENT -->
	<ReactiveSequence>
		<ReactiveFallback>
			<ReactiveSequence>
				<Ifprob arg0="{sr}" arg1="15.000" arg2="0.500"/>
				<Mulav arg0="{vscr}" arg1="{vzero}" arg2="%s" arg3="{vrecr}"/>
			</ReactiveSequence>
			<Movcv arg0="{vscr}" arg1="0"/>
		</ReactiveFallback>
		<Mulav arg0="{vvote}" arg1="{vscr}" arg2="-5.000" arg3="{vprox}"/>
	</ReactiveSequence>
	<!----------------->
	)";

	std::string home = R"(
	<!-- HOME -->
	<Mulav arg0="{vvote}" arg1="{vhome}" arg2="-5.000" arg3="{vprox}"/>
	<!---------->
	)";
	
	std::string position = R"(
	<!-- POSITION -->
	<Mulav arg0="{vvote}" arg1="{vlift}" arg2="-5.000" arg3="{vprox}"/>
	<!---------->
	)";
	
	// Obstacle avoidance as fixed behaviour
	std::string avoidance = R"(
	<!-- AVOIDANCE -->
	<ReactiveSequence>
		<Mulav arg0="{vscr}" arg1="{vzero}" arg2="0.500" arg3="{vprox}"/>
		<Ifsect arg0="{vscr}" arg1="0" arg2="127"/>
		<Mulav arg0="{vvote}" arg1="{vzero}" arg2="-5.000" arg3="{vprox}"/>
	</ReactiveSequence>
	<!--------------->
	)";
	
	// Conditions
	std::string neighbour = R"(
	<!-- NEIGHBOUR -->
	<Ifprob arg0="{sn}" arg1="%s" arg2="%s"/>
	<!--------------->
	)";

	std::string recruiter = R"(
	<!-- RECRUITER -->
	<Ifprob arg0="{sr}" arg1="%s" arg2="%s"/>
	<!--------------->
	)";
	
	std::string porter = R"(
	<!-- PORTER -->
	<ReactiveSequence>
		<Ifprob arg0="{sp}" arg1="15.000" arg2="0.500"/>
		<Ifprob arg0="{szero}" arg1="-0.250" arg2="%s"/>
	</ReactiveSequence>
	<!------------>
	)";

	std::string nest = R"(
	<!-- NEST -->
	<ReactiveSequence>
		<Ifsect arg0="{vhome}" arg1="0" arg2="0"/>
		<Ifprob arg0="{szero}" arg1="-0.250" arg2="%s"/>
	</ReactiveSequence>
	<!---------->
	)";

	std::string liftpoint = R"(
	<!-- LIFTPOINT -->
	<ReactiveSequence>
		<Ifsect arg0="{vlift}" arg1="0" arg2="0"/>
		<Ifprob arg0="{szero}" arg1="-0.250" arg2="%s"/>
	</ReactiveSequence>
	<!--------------->
	)";

	std::string item = R"(
	<!-- ITEM -->
	<ReactiveSequence>
		<Mulav arg0="{vscr}" arg1="{vzero}" arg2="0.100" arg3="{vlift}"/>
		<Ifsect arg0="{vscr}" arg1="0" arg2="0"/>
		<Ifprob arg0="{szero}" arg1="-0.250" arg2="%s"/>
	</ReactiveSequence>
	<!---------->
	)";
		
	std::string fixedprob = R"(
	<!-- FIXEDPROB -->
	<Ifprob arg0="{szero}" arg1="-0.250" arg2="%s"/>
	<!--------------->
	)";
} ExecutionXML; // end struct

// GeneType definition as behaviour trees
// Blackboard (R/W access by main BT)
const std::vector<std::string> bb_rsreg = {"pvote", "szero", "sn", "sr", "sp",
                                           "sscr"};
const std::vector<std::string> bb_wsreg = {"pvote", "sscr"};
const std::vector<std::string> bb_rvreg = {"vzero", "vvote", "vprox", "vattr",
                                           "vrecr", "vhome", "vlift", "vscr"};
const std::vector<std::string> bb_wvreg = {"vvote", "vscr"};

// Parameters **************************
enum ParamType
{
	CHAR,    // signed 8-bit [-128, 127]
	UINTR,   // ranged unsigned integer [1, 100]
	FLOAT5,  // ranged floating point [-5.0, 5.0]
	FLOAT32, // ranged floating point [-32.0, 32.0]
	FP53,    // fixed point 5.3 [-128, 127]/8
	RSREG,   // read scalar register
	RVREG,   // read vector register
	WSREG,   // read/write scalar register
	WVREG    // read/write vector register
};

class GPParam
{
	public:
		enum ParamType type;
		float value;
		std::string toString();
		void init();
};

// From https://stackoverflow.com/a/26221725
template<typename ... Args>
std::string string_format(const std::string& format, Args ... args)
{
    int size_s = std::snprintf(nullptr, 0, format.c_str(), args ...) + 1;
    if(size_s <= 0)
	{
		throw std::runtime_error("Error during formatting.");
	}

    auto size = static_cast<size_t>(size_s);
    auto buf = std::make_unique<char[]>(size);
    std::snprintf(buf.get(), size, format.c_str(), args ...);
    return std::string(buf.get(), buf.get() + size - 1);
}

std::string GPParam::toString()
{
	std::string param;
	std::string reg;
	switch (type)
	{
		case CHAR:
		case UINTR:
			param = string_format("%.0f", value);
			break;

		case FLOAT5:
		case FLOAT32:
		case FP53:
			param = string_format("%.3f", value);
			break;

		case RSREG:
			reg = bb_rsreg.at(static_cast<int>(value));
			param = "{" + reg + "}";
			break;

		case RVREG:
			reg = bb_rvreg.at(static_cast<int>(value));
			param = "{" + reg + "}";
			break;

		case WSREG:
			reg = bb_wsreg.at(static_cast<int>(value));
			param = "{" + reg + "}";
			break;

		case WVREG:
			reg = bb_wvreg.at(static_cast<int>(value));
			param = "{" + reg + "}";
			break;
	}
	return param;
};

void GPParam::init()
{
	switch (type)
	{
		case CHAR:
			value = rndif(-128, 127);
			break;

		case UINTR:
			value = rndif(1, 100);
			break;

		case FLOAT5:
			value = rndf(-5.0f, 5.0f);
			break;

		case FLOAT32:
			value = rndf(-32.0f, 32.0f);
			break;

		case FP53:
			value = rndif(-128, 127) / 8.0f;
			break;

		case RSREG:
			value = rndif(0, bb_rsreg.size() - 1);
			break;

		case RVREG:
			value = rndif(0, bb_rvreg.size() - 1);
			break;

		case WSREG:
			value = rndif(0, bb_wsreg.size() - 1);
			break;

		case WVREG:
			value = rndif(0, bb_wvreg.size() - 1);
			break;
	}
};

// Nodes *******************************
enum NodeID
{
	// Control-flow
	SEQ2 = 0,
	SEQ3,
	SEQ4,
	SEL2,
	SEL3,
	SEL4,
	SEQM2,
	SEQM3,
	SEQM4,
	SELM2,
	SELM3,
	SELM4,
	INVERT,
	SUCCESSD,
	FAILURED,
	REPEATI,
	// Execution (actions)
	MOVCS = 20,
	MOVCV,
	MULAS,
	MULAV,
	ROTAV,
	MOVPV,
	SUCCESSL,
	FAILUREL,
	IFPROB,
	IFSECT,
	// Execution (conditions)
	NEIGHBOUR,
	RECRUITER,
	PORTER,
	NEST,
	LIFTPOINT,
	ITEM,
	FIXEDPROB,
	// Execution (behaviours)
	EXPLORE,
	ATTRACT,
	RECRUIT,
	POSITION,
	HOME
};
typedef enum NodeID NodeID; 

class GPNode
{
	public:
		std::string name;
		unsigned short depth;
		unsigned short arity;
		std::vector<GPParam> params;
		std::string macro;
		void initParams();
};

void GPNode::initParams()
{
	for (auto& param : params)
	{
		param.init();
	}
}

std::map<NodeID, GPNode> control_flow_set
{
	{SEQ2,     {"ReactiveSequence", 0, 2, {},              ""}},
	{SEQ3,     {"ReactiveSequence", 0, 3, {},              ""}},
	{SEQ4,     {"ReactiveSequence", 0, 4, {},              ""}},
	{SEL2,     {"ReactiveFallback", 0, 2, {},              ""}},
	{SEL3,     {"ReactiveFallback", 0, 3, {},              ""}},
	{SEL4,     {"ReactiveFallback", 0, 4, {},              ""}},
	{SEQM2,    {"Sequence",         0, 2, {},              ""}},
	{SEQM3,    {"Sequence",         0, 3, {},              ""}},
	{SEQM4,    {"Sequence",         0, 4, {},              ""}},
	{SELM2,    {"Fallback",         0, 2, {},              ""}},
	{SELM3,    {"Fallback",         0, 3, {},              ""}},
	{SELM4,    {"Fallback",         0, 4, {},              ""}},
	{INVERT,   {"Inverter",         0, 1, {},              ""}},
	{SUCCESSD, {"ForceSuccess",     0, 1, {},              ""}},
	{FAILURED, {"ForceFailure",     0, 1, {},              ""}},
	{REPEATI,  {"Repeati",          0, 1, {{UINTR, 0.0f}}, ""}}
};

std::map<NodeID, GPNode> execution_set
{
	// Actions
	{MOVCS,    {"Movcs",         0, 0, {{WSREG, 0.0f}, {CHAR, 0.0f}},                                  ""}},
	{MOVCV,    {"Movcv",         0, 0, {{WVREG, 0.0f}, {CHAR, 0.0f}},                                  ""}},
	{MULAS,    {"Mulas",         0, 0, {{WSREG, 0.0f}, {RSREG, 0.0f}, {FLOAT32, 0.0f}, {RSREG, 0.0f}}, ""}},
	{MULAV,    {"Mulav",         0, 0, {{WVREG, 0.0f}, {RVREG, 0.0f}, {FLOAT32, 0.0f}, {RVREG, 0.0f}}, ""}},
	{ROTAV,    {"Rotav",         0, 0, {{WVREG, 0.0f}, {RVREG, 0.0f}, {CHAR, 0.0f}, {RVREG, 0.0f}},    ""}},
	{MOVPV,    {"Movpv",         0, 0, {{WVREG, 0.0f}, {RVREG, 0.0f}, {CHAR, 0.0f}},                   ""}},
	{SUCCESSL, {"AlwaysSuccess", 0, 0, {},                                                             ""}},
	{FAILUREL, {"AlwaysFailure", 0, 0, {},                                                             ""}},
	{IFPROB,   {"Ifprob",        0, 0, {{RSREG, 0.0f}, {FP53, 0.0f}, {FP53, 0.0f}},                    ""}},
	{IFSECT,   {"Ifsect",        0, 0, {{RVREG, 0.0f}, {CHAR, 0.0f}, {CHAR, 0.0f}},                    ""}},
	// Conditions
	{NEIGHBOUR, {"Neighbour", 0, 0, {{FP53, 0.0f}, {FP53, 0.0f}}, ExecutionXML.neighbour}},
	{RECRUITER, {"Recruiter", 0, 0, {{FP53, 0.0f}, {FP53, 0.0f}}, ExecutionXML.recruiter}},
	{PORTER,    {"Porter",    0, 0, {{FP53, 0.0f}},               ExecutionXML.porter   }},
	{NEST,      {"Nest",      0, 0, {{FP53, 0.0f}},               ExecutionXML.nest     }},
	{LIFTPOINT, {"Liftpoint", 0, 0, {{FP53, 0.0f}},               ExecutionXML.liftpoint}},
	{ITEM,      {"Item",      0, 0, {{FP53, 0.0f}},               ExecutionXML.item     }},
	{FIXEDPROB, {"Fixedprob", 0, 0, {{FP53, 0.0f}},               ExecutionXML.fixedprob}},
	// Behaviours
	{EXPLORE,  {"Explore",  0, 0, {},               ExecutionXML.exploration}},
	{ATTRACT,  {"Attract",  0, 0, {{FLOAT5, 0.0f}}, ExecutionXML.attraction }},
	{RECRUIT,  {"Recruit",  0, 0, {{FLOAT5, 0.0f}}, ExecutionXML.recruitment}},
	{POSITION, {"Position", 0, 0, {},               ExecutionXML.position   }},
	{HOME,     {"Home",     0, 0, {},               ExecutionXML.home       }}
};

std::map<NodeID, GPNode>::size_type cfs_size = control_flow_set.size();
std::map<NodeID, GPNode>::size_type es_size = execution_set.size();
double ratio = static_cast<double>(es_size) / static_cast<double>(cfs_size+es_size);

// Tree ********************************
class GPTree
{
	public:
		std::vector<GPNode> tree;
		bool grow;
		unsigned short max_depth;
};

// Returns a string -> arg0="..." arg1="..." ... argn="..."
std::string printParams(std::vector<GPParam> params)
{
	std::string param_string;
	unsigned short it = 0;
	for (auto& param : params)
	{
		param_string += " arg" + std::to_string(it) + "=\"" + param.toString() + "\"";
		it++;
	}
	return param_string;
}

// Returns action macro with corresponding parameter values
// (Ugly, but it works)
std::string printMacro(const std::string& macro, 
                       std::vector<GPParam> params,
                       unsigned short depth)
{
	std::string macro_out;
	switch (params.size())
	{
		case 1:
			macro_out = string_format(macro,
			                          params.at(0).toString().c_str());
			break;

		case 2:
			macro_out = string_format(macro,
			                          params.at(0).toString().c_str(),
			                          params.at(1).toString().c_str());
			break;

		default:
			macro_out = macro;
	}
	return macro_out;
}

std::string printTree(const GPTree& gpt, std::string tree_id)
{
	// Header
	std::string xml = "<root main_tree_to_execute=\"" + tree_id + "\">\n";
	xml += "<BehaviorTree ID=\"" + tree_id + "\">\n";

	// Tree
	std::map<unsigned short, std::string> close_pending;
	for (const auto& node : gpt.tree)
	{
		// Check for closing nodes
		while (!close_pending.empty())
		{
			auto pending = std::prev(close_pending.end());
			if (pending->first >= node.depth)
			{
				xml += pending->second;
				close_pending.erase(pending);
			}
			else
			{
				break;
			}
		}

		// Write node
		if (node.arity > 0) // control-flow
		{ 
			xml += "<" + node.name;
			if (!node.params.empty()) // decorators
			{
				xml += printParams(node.params);
			}

			xml += ">\n";
			close_pending[node.depth] = "</" + node.name + ">\n";
		}
		else // execution
		{
			if (node.macro.empty())
			{
				xml += "<" + node.name;
				xml += printParams(node.params);
				xml += "/>\n";
			}
			else
			{
				xml += printMacro(node.macro, node.params, node.depth);
			}
		}
	}

	// Close remaining control-flow nodes
	if (!close_pending.empty())
	{
		for (auto it = close_pending.rbegin(); it != close_pending.rend(); it++)
		{
			xml += it->second;
		}
	}

	// Foot
	xml += "</BehaviorTree>\n</root>";
	return xml;
}

//=============================================================================

// Define MiddleCostType -> negative fitness (openGA minimises cost)
struct GPMiddleCost
{
	float fitness;
	std::size_t tree_size;
};

typedef EA::Genetic<GPTree, GPMiddleCost> GA_Type;
typedef EA::GenerationType<GPTree, GPMiddleCost> Generation_Type;

void calculate_parsimony_coefficient(Generation_Type& generation)
{
	std::vector<std::size_t> l;
	std::vector<float> f;
	for (const auto& ch : generation.chromosomes)
	{
		l.push_back(ch.middle_costs.tree_size);
		f.push_back(ch.middle_costs.fitness);
	}

	float cov = covariance<std::size_t, float>(l, f);
	float var = covariance<std::size_t, std::size_t>(l, l);

	generation.parsimony_coefficient = var ? cov / var : 0.0f;
}

//=============================================================================

// GeneType initialisation
void grow_tree(GPTree& gpt,
               const std::function<double(void)> &rnd01,
               unsigned short depth)
{
	if (gpt.max_depth == depth || (gpt.grow && rnd01() < ratio))
	{
		NodeID index = static_cast<NodeID>(20 + rndi(0, es_size-1));
		GPNode e_node = execution_set.at(index);
		e_node.initParams();
		e_node.depth = depth;
		gpt.tree.push_back(e_node);
	}
	else
	{
		NodeID index = static_cast<NodeID>(rndi(0, cfs_size-1));
		GPNode cf_node = control_flow_set.at(index);
		cf_node.initParams();
		cf_node.depth = depth;

		GPTree subtree;
		subtree.tree.push_back(cf_node);
		subtree.grow = gpt.grow;
		subtree.max_depth = gpt.max_depth;
		for (unsigned short i = 0; i < cf_node.arity; ++i)
		{
			grow_tree(subtree, rnd01, depth + 1);
		}

		gpt.tree.insert(gpt.tree.end(), subtree.tree.begin(), subtree.tree.end());
	}
}

void init_genes(GPTree& gpt,
                const std::function<double(void)>& rnd01)
{
	// Ramped half-and-half
	static unsigned int tree_counter = 0;
	unsigned short range = EvoParams.max_max_depth - EvoParams.min_max_depth + 1;
	static unsigned short max_depth = EvoParams.min_max_depth + 1;

	if (tree_counter % (EvoParams.population / range) == 0 && tree_counter != 0)
	{
		max_depth++;
	}

	gpt.max_depth = max_depth;
	gpt.grow = (tree_counter % 2) ? true : false;

	// Introduce fixed elements (emergency obstacle avoidance)
	GPNode root = control_flow_set.at(SEL2);
	root.depth = 0;
	gpt.tree.push_back(root);

	GPNode avoidance{"Avoidance", 1, 0, {}, ExecutionXML.avoidance};
	gpt.tree.push_back(avoidance);

	// Grow behaviour tree
	grow_tree(gpt, rnd01, 1);
	tree_counter++;
}

//=============================================================================

// Evaluate solution -> simulation required
bool eval_solution(const GPTree& gpt, GPMiddleCost& gpmc)
{
    // Get a unique thread id
    std::thread::id this_id = std::this_thread::get_id();
    auto tid = std::hash<std::thread::id>()(this_id);
    
    
	gpmc.tree_size = gpt.tree.size();
	gpmc.fitness = 0.0f;
	std::string bt = printTree(gpt, "MainTree");

	for (auto i = 0; i < EvoParams.n_evals; ++i)
	{
		// Initialise environment
        //printf("%016lx %d start\n", tid, i);
		Environment env(g_cmd);
		
		env.addArena();

		env.addLoad(b2Vec2(-1.5f,  1.25f), 2);
		env.addLoad(b2Vec2(-1.5f,  0.00f), 2);
		env.addLoad(b2Vec2(-1.5f, -1.25f), 2);
        
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
		
		// Set controllers of all the robots in the environment
		while ((robot = env.getNextRobot()))
		{
			robot->controller.xml_tree = bt;
			robot->controller.updateSource();
		}

		// Run simulation and compute controller fitness
		gpmc.fitness += env.run(EvoParams.t_sim);
        //printf("%016lx %d done eval\n", tid, i);

	}

	// Mean fitness
	gpmc.fitness /= EvoParams.n_evals;
    printf("%016lx   final %f\n", tid, gpmc.fitness);

	return true;
}

//=============================================================================

// Define mutation process (avoid mutating fixed structure)
int selectNode(const GPTree& gpt, 
               const std::function<double(void)>& rnd01)
{
	bool cf = rnd01() < EvoParams.p_inner;
	std::vector<int> candidates;
	for (int i = 2; i < gpt.tree.size(); ++i)
	{
		if (cf && gpt.tree.at(i).arity)
		{
			candidates.push_back(i);
		}
		else if (!cf && !gpt.tree.at(i).arity)
		{
			candidates.push_back(i);
		}
	}

	// All trees have at least 1 execution node
	if (candidates.size() == 0)
	{
		return 2;
	}

	return candidates.at(rndi(0, candidates.size()-1));
}

GPTree mutate(const GPTree& gpt,
              const std::function<double(void)> &rnd01,
              double shrink_scale)
{
	GPTree gpt_new = gpt;

	// Parameter mutation
	for (auto& node : gpt_new.tree)
	{
		if (node.depth == 0 || node.name == "Avoidance")
		{
			continue;
		}

		if (rnd01() < EvoParams.p_mut_param)
		{
			node.initParams();
		}
	}

	// Point mutation
	for (auto& node : gpt_new.tree)
	{
		if (node.depth == 0 || node.name == "Avoidance")
		{
			continue;
		}

		if (rnd01() < EvoParams.p_mut_point)
		{
			GPNode new_node = node;

			if (node.arity > 0) // control-flow
			{
				std::vector<NodeID> candidates;
				for (const auto& it : control_flow_set)
				{
					if (it.second.arity == node.arity &&
					    it.second.name != node.name)
					{
						candidates.push_back(it.first);
					}
				}

				if (candidates.size() > 0)
				{
					int index = rndi(0, candidates.size() - 1);
					new_node = control_flow_set.at(candidates.at(index));
				}
			}
			else // execution
			{
				std::vector<NodeID> candidates;
				for (const auto& it : execution_set)
				{
					if (it.second.name != node.name)
					{
						candidates.push_back(it.first);
					}
				}

				if (candidates.size() > 0)
				{
					int index = rndi(0, candidates.size() - 1);
					new_node = execution_set.at(candidates.at(index));
				}
			}

			new_node.depth = node.depth;
			new_node.initParams();
			node = new_node;
		}
	}

	// Subtree mutation
	if (rnd01() < EvoParams.p_mut_subtree)
	{
		// Generate random subtree
		GPTree subtree;
		subtree.grow = false;
		subtree.max_depth = rndi(EvoParams.min_max_depth,
		                         EvoParams.max_max_depth);
		grow_tree(subtree, rnd01, 0);

		// Replace subtree
		int mut_begin = selectNode(gpt_new, rnd01);
		for (auto& node : subtree.tree)
		{
			node.depth += gpt_new.tree.at(mut_begin).depth;
		}

		int mut_end = mut_begin + 1;
		for (auto node = gpt_new.tree.begin() + mut_begin + 1;
		     node != gpt_new.tree.end();
		     ++node)
		{
			if (node->depth > gpt_new.tree.at(mut_begin).depth)
			{
				++mut_end;
			}
			else
			{
				break;
			}
		}

		gpt_new.tree.erase(gpt_new.tree.begin() + mut_begin,
		                   gpt_new.tree.begin() + mut_end);
		gpt_new.tree.insert(gpt_new.tree.begin() + mut_begin,
		                    subtree.tree.begin(),
		                    subtree.tree.end());
	}

	return gpt_new;
}
//=============================================================================

// Define GeneType crossover
GPTree crossover(const GPTree& gpt1,
                 const GPTree& gpt2,
                 const std::function<double(void)> &rnd01)
{
	GPTree gpt_new = gpt1;

	// Select crossover point in parent 1
	int xover1_begin = selectNode(gpt1, rnd01);
	unsigned short subtree1_depth = gpt1.tree.at(xover1_begin).depth;

	// Delete subtree from copy of parent 1
	int xover1_end = xover1_begin + 1;
	for (auto node = gpt_new.tree.begin() + xover1_begin + 1;
	     node != gpt_new.tree.end();
	     ++node)
	{
		if (node->depth > gpt_new.tree.at(xover1_begin).depth)
		{
			++xover1_end;
		}
		else
		{
			break;
		}
	}

	gpt_new.tree.erase(gpt_new.tree.begin() + xover1_begin,
	                   gpt_new.tree.begin() + xover1_end);

	// Select crossover point in parent 2
	int xover2_begin = selectNode(gpt2, rnd01);
	unsigned short subtree2_depth = gpt2.tree.at(xover2_begin).depth;

	// Extract subtree from parent 2 (only need nodes)
	std::vector<GPNode> subtree2 {gpt2.tree.at(xover2_begin)};
	for (int i = xover2_begin + 1; i < gpt2.tree.size(); ++i)
	{
		if (gpt2.tree.at(i).depth > subtree2_depth)
		{
			subtree2.push_back(gpt2.tree.at(i));
		}
		else
		{
			break;
		}
	}

	// Insert subtree from parent 2 into copy of parent 1 at crossover point
	for (auto& node : subtree2)
	{
		node.depth += (subtree1_depth - subtree2_depth);
	}

	gpt_new.tree.insert(gpt_new.tree.begin() + xover1_begin,
	                    subtree2.begin(),
	                    subtree2.end());
	return gpt_new;
}

//=============================================================================

// Calculate final cost (with covariant parsimony pressure)
double calculate_SO_total_fitness(const GA_Type::thisChromosomeType& X,
                                  float parsimony_coefficient)
{
	float f = X.middle_costs.fitness;
	float l = X.middle_costs.tree_size;
    //float cost = -(f - parsimony_coefficient * l);
    float cost = -f;
	return cost;
}

//=============================================================================

// Print data of interest to a file and to cout
std::ofstream output_file;

void SO_report_generation(
	int generation_number,
	const Generation_Type& last_generation,
	const GPTree& best_genes)
{
	std::cout
		<< "Generation [" << generation_number << "], "
		<< "Best = " << last_generation.best_total_cost << ", "
		<< "Average = " << last_generation.average_cost << ", "
		<< "Exe time = " << last_generation.exe_time
		<< std::endl;

	output_file
		<< "==================================================\n"
		<< "Generation [" << generation_number << "]\n"
		<< "Best = " << last_generation.best_total_cost << "\n"
		<< "Average = " << last_generation.average_cost << "\n"
		<< "Exe time = " << last_generation.exe_time << "\n"
		<< "Best size = " << best_genes.tree.size() << "\n"
		<< "--------------------------------------------------\n"
		<< printTree(best_genes, "MainTree") << "\n";
    
    int i = 0;
    for(auto &c : last_generation.chromosomes)
    {
        output_file
        << "==================================================\n"
        << "Chromosone [" << i++ << "] cost [" << c.total_cost << "]\n"
        << "--------------------------------------------------\n"
        << printTree(c.genes, "MainTree") << "\n";
        
    }
}

int main(int argc, char* argv[])
{
    // Process command line args
    process_args(argc, argv, g_cmd);
    
    g_cmd.gui = false;
    
    // Create and open output file
	output_file.open("report.txt");

	// Process optional input
//	if (argc > 1)
//	{
		EvoParams.t_sim = 1000;
//	}

	// Start timer
	EA::Chronometer timer;
	timer.tic();

	// Set evolution paramters
	GA_Type ga_obj;
	ga_obj.problem_mode = EA::GA_MODE::SOGA;
	ga_obj.multi_threading = true;
    ga_obj.N_threads = 16;
	ga_obj.idle_delay_us = 1; // switch between threads quickly
	ga_obj.verbose = g_cmd.verbose;
	ga_obj.population = EvoParams.population;
	ga_obj.generation_max = 200;
	ga_obj.calculate_parsimony_coefficient = calculate_parsimony_coefficient;
	ga_obj.calculate_SO_total_fitness = calculate_SO_total_fitness;
	ga_obj.init_genes = init_genes;
	ga_obj.eval_solution = eval_solution;
	ga_obj.mutate = mutate;
	ga_obj.crossover = crossover;
	ga_obj.SO_report_generation = SO_report_generation;
	ga_obj.average_stall_max = ga_obj.generation_max + 1;
	ga_obj.best_stall_max = ga_obj.generation_max + 1;
	ga_obj.elite_count = 3;
	ga_obj.crossover_fraction = 0.7;
	ga_obj.mutation_rate = 1.0;

	// Report header
	auto current = std::chrono::system_clock::now();
	std::time_t current_time = std::chrono::system_clock::to_time_t(current);
	output_file << "**************************************************\n"
	            << std::ctime(&current_time) << "\n"
	            << "Number of robots: " << "16\n"
	            << "Number of loads: " << "3, 2-porters\n"
	            << "Population: " << EvoParams.population << "\n"
	            << "Generations: " << ga_obj.generation_max << "\n"
	            << "Simulation time: " << (float)EvoParams.t_sim/30.0f << "s\n"
	            << "Evaluations: " << EvoParams.n_evals << "\n"
	            << "Elite count: " << ga_obj.elite_count << "\n"
	            << "Crossover fraction: " << ga_obj.crossover_fraction << "\n"
	            << "Prob. of inner node selection: " << EvoParams.p_inner << "\n"
	            << "Prob. of parameter mutation: " << EvoParams.p_mut_param << "\n"
	            << "Prob. of point mutation: " << EvoParams.p_mut_point << "\n"
	            << "Prob. of subtree mutation: " << EvoParams.p_mut_subtree << "\n"
	            << "Min. tree depth " << EvoParams.min_max_depth << "\n"
	            << "Max. tree depth " << EvoParams.max_max_depth << "\n"
	            << "**************************************************"
	            << std::endl;

	// Evolve
	ga_obj.solve();
	
	std::cout << "The problem is optimized in " << timer.toc() << " seconds."
	          << std::endl;

	return 0;
}
