#include "controller.h"
#include "bt_custom.h"

Controller::Controller() : voted(std::make_shared<bool>(false)), p_logger(nullptr)
{
	// Default tree always returns success
	xml_tree = R"(
	<root main_tree_to_execute="MainTree">
		<BehaviorTree ID="MainTree">
			<AlwaysSuccess/>
		</BehaviorTree>
	</root> 
	)";

	// Create builders for actions nodes with parameters
	NodeBuilder builder_movcs =
		[this](const std::string& name, const NodeConfiguration& config)
	{
		return std::make_unique<Movcs>(name, config, this->voted);
	};

	NodeBuilder builder_movcv =
		[this](const std::string& name, const NodeConfiguration& config)
	{
		return std::make_unique<Movcv>(name, config, this->voted);
	};

	NodeBuilder builder_movpv =
		[this](const std::string& name, const NodeConfiguration& config)
	{
		return std::make_unique<Movpv>(name, config, this->voted);
	};

	NodeBuilder builder_mulas =
		[this](const std::string& name, const NodeConfiguration& config)
	{
		return std::make_unique<Mulas>(name, config, this->voted);
	};

	NodeBuilder builder_mulav =
		[this](const std::string& name, const NodeConfiguration& config)
	{
		return std::make_unique<Mulav>(name, config, this->voted);
	};

	NodeBuilder builder_rotav =
		[this](const std::string& name, const NodeConfiguration& config)
	{
		return std::make_unique<Rotav>(name, config, this->voted);
	};

	// Register all custom nodes
	p_factory.registerBuilder<Movcs>("Movcs", builder_movcs);
	p_factory.registerBuilder<Movcv>("Movcv", builder_movcv);
	p_factory.registerBuilder<Movpv>("Movpv", builder_movpv);
	p_factory.registerBuilder<Mulas>("Mulas", builder_mulas);
	p_factory.registerBuilder<Mulav>("Mulav", builder_mulav);
	p_factory.registerBuilder<Rotav>("Rotav", builder_rotav);
	p_factory.registerNodeType<Ifprob>("Ifprob");
	p_factory.registerNodeType<Ifsect>("Ifsect");
	p_factory.registerNodeType<Repeati>("Repeati");

	p_tree = p_factory.createTreeFromText(xml_tree);

	// Initialise blackboard entries
	p_tree.rootBlackboard()->set("szero", 0.0f);
	p_tree.rootBlackboard()->set("vzero", VectorPolar(0.0f, 0.0f));
	p_tree.rootBlackboard()->set("sscr", 0.0f);
	p_tree.rootBlackboard()->set("vscr", VectorPolar(0.0f, 0.0f));
	p_tree.rootBlackboard()->set("pvote", 0.0f);
	p_tree.rootBlackboard()->set("vvote", VectorPolar(0.0f, 0.0f));
	p_tree.rootBlackboard()->set("vprox", VectorPolar(0.0f, 0.0f));
	p_tree.rootBlackboard()->set("vattr", VectorPolar(1.0f, 0.0f));
	p_tree.rootBlackboard()->set("vrecr", VectorPolar(1.0f, 0.0f));
	p_tree.rootBlackboard()->set("vhome", VectorPolar(1.0f, 0.0f));
	p_tree.rootBlackboard()->set("vlift", VectorPolar(1.0f, 0.0f));
	p_tree.rootBlackboard()->set("sn", 0.0f);
	p_tree.rootBlackboard()->set("sr", 0.0f);
	p_tree.rootBlackboard()->set("sp", 0.0f);
}

void Controller::updateSource()
{
	p_tree = p_factory.createTreeFromText(xml_tree);

	p_tree.rootBlackboard()->set("szero", 0.0f);
	p_tree.rootBlackboard()->set("vzero", VectorPolar(0.0f, 0.0f));
	p_tree.rootBlackboard()->set("sscr", 0.0f);
	p_tree.rootBlackboard()->set("vscr", VectorPolar(0.0f, 0.0f));
	p_tree.rootBlackboard()->set("pvote", 0.0f);
	p_tree.rootBlackboard()->set("vvote", VectorPolar(0.0f, 0.0f));
	p_tree.rootBlackboard()->set("vprox", VectorPolar(0.0f, 0.0f));
	p_tree.rootBlackboard()->set("vattr", VectorPolar(1.0f, 0.0f));
	p_tree.rootBlackboard()->set("vrecr", VectorPolar(1.0f, 0.0f));
	p_tree.rootBlackboard()->set("vhome", VectorPolar(1.0f, 0.0f));
	p_tree.rootBlackboard()->set("vlift", VectorPolar(1.0f, 0.0f));
	p_tree.rootBlackboard()->set("sn", 0.0f);
	p_tree.rootBlackboard()->set("sr", 0.0f);
	p_tree.rootBlackboard()->set("sp", 0.0f);

	//std::cout << "Tree source updated. New tree:" << std::endl;
	//printTreeRecursively(p_tree.rootNode());
	//p_logger = std::make_unique<BT::StdCoutLogger>(p_tree);
}

// Update blakboard entries using an update scene and message. Vector angles
// are given wrt heading, not orientation
void Controller::updateBB(const Scene& scene, const Message& message,
                          const std::vector<Message>& comms)
{
	const auto& bb = p_tree.rootBlackboard();

	// Reset Votes
	bb->set("vvote", VectorPolar(0.0f, 0.0f));
	bb->set("pvote", 0.0f);

	// Proximity
	VectorPolar v_prox;
	short n_sensors = sizeof(g_rc.prox_angles) / sizeof(g_rc.prox_angles[0]);
	for (short i = 0; i < n_sensors; ++i)
	{
		v_prox += VectorPolar(scene.prox[i], g_rc.prox_angles[i]);
	}

	v_prox.rotate(scene.orientation - scene.heading);
	bb->set("vprox", v_prox);

	// Attraction
	VectorPolar v_attr;
	if (scene.n)
	{
		for (const auto& neighbour : scene.neighbours)
		{
			v_attr += VectorPolar(1.0f / (1.0f + neighbour.rb.r), neighbour.rb.a);
		}

		v_attr.rotate(scene.orientation - scene.heading);
	}
	else
	{
		v_attr = VectorPolar(1.0f, 0.0f);
	}

	bb->set("sn", static_cast<float>(scene.n));
	bb->set("vattr", v_attr);

	// Recruitment and porters in group
	VectorPolar v_recr;
	short s_r = 0;
	short s_p = message.id_group ? 1 : 0;
	for (const auto& msg : comms)
	{
		// Recruitment
		if (msg.id_load && !msg.id_group)
		{
			for (const auto& neighbour : scene.neighbours)
			{
				if (neighbour.id == msg.id_messenger)
				{
					v_recr += VectorPolar(1.0f / (1.0f + neighbour.rb.r),
					                      neighbour.rb.a);
					++s_r;
				}
			}
		}

		// Porters in group
		if (message.id_group && message.id_group == msg.id_group)
		{
			++s_p;
		}
	}

	if (s_r)
	{
		v_recr.rotate(scene.orientation - scene.heading);
	}
	else
	{
		v_recr = VectorPolar(1.0f, 0.0f);
	}

	bb->set("vrecr", v_recr);
	bb->set("sr", static_cast<float>(s_r));
	bb->set("sp", static_cast<float>(s_p));

	// Home
	VectorPolar v_home;
	if (scene.nest)
	{
		float d_nest = scene.rb_nest.r - g_ac.nest_r + g_rc.radius + 0.05f;
		if (d_nest < 0.0f)
		{
			d_nest = 0.0f;
		}

		v_home = VectorPolar(d_nest,
		                     scene.rb_nest.a+scene.orientation-scene.heading);
	}
	else
	{
		v_home = VectorPolar(1.0f, -scene.heading);
	}

	bb->set("vhome", v_home);

	// Lifting point
	VectorPolar v_lift;
	if (scene.load)
	{
		v_lift = scene.rb_lift;
		v_lift.rotate(scene.orientation - scene.heading);
	}
	else
	{
		v_lift = VectorPolar(1.0f, 0.0f);
	}

	bb->set("vlift", v_lift);
}

const BT::Blackboard::Ptr Controller::getBB()
{
	return p_tree.rootBlackboard();
}

void Controller::printBB()
{
	std::cout << "===== BLACKBOARD =====" << std::endl;

	const auto& bb = p_tree.rootBlackboard();
	std::vector<BT::StringView> keys = bb->getKeys();
	for (const auto& key : keys)
	{
		std::string skey = BT::convertFromString<std::string>(key);
		const BT::Any* value_any = bb->getAny(skey);
		std::cout << skey << " = ";
		if (value_any->isNumber())
		{
			std::cout << value_any->cast<float>();
		}
		else
		{
			VectorPolar value = value_any->cast<VectorPolar>();
			std::cout << "[" << value.r << "," << value.a << "]";
		}
		std::cout << std::endl;
	}

	std::cout << "======================" << std::endl;
}

void Controller::tick()
{
	//printBB();
	*(voted.get()) = false;
	p_tree.tickRoot();
}
