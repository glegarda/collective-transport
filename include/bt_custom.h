#ifndef BT_CUSTOM_H
#define BT_CUSTOM_H

#include <iostream>
#include <cmath>
#include <random>
#include <memory>
#include "utils.h"
#include <behaviortree_cpp_v3/bt_factory.h>

using namespace BT;

// Parse string of two values separated by semicolon into VectorPolar
namespace BT
{
	template <> inline VectorPolar convertFromString(StringView str)
	{
		auto parts = splitString(str, ';');

		if (parts.size() != 2)
		{
			throw RuntimeError("invalid input");
		}
		else
		{
			VectorPolar output;
			output.r = convertFromString<float>(parts[0]);
			output.a = convertFromString<float>(parts[1]);
			return output;
		}
	}
}

//              //
// Action nodes //
//              //

class Movcs : public CoroActionNode
{
	public:
		Movcs(const std::string& name, const NodeConfiguration& config,
		      std::shared_ptr<bool> voted)
			: CoroActionNode(name, config), _voted(voted) {}

		static PortsList providedPorts()
		{
			PortsList pl;
			pl.insert(OutputPort<float>("arg0")); // entry
			pl.insert(InputPort<int>("arg1"));    // scalar
			return pl;
		}

	private:
		NodeStatus tick() override
		{
			// Check that vote has not been already cast
			const NodeConfiguration config = this->config();
			auto entry = config.output_ports.find("arg0");
			if (entry->second == "{pvote}")
			{
				if (*(_voted.get()))
				{
					setStatusRunningAndYield();
				}
				else
				{
					*(_voted.get()) = true;
				}
			}

			Optional<int> s = getInput<int>("arg1");
			if (!s)
			{
				throw RuntimeError("missing required input [arg1]: ", s.error());
			}

			// Avoid infinity and NaN
			int scalar = s.value();
			if (!std::isfinite(scalar))
			{
				scalar = 0;
			}

			setOutput("arg0", static_cast<float>(scalar));
			return NodeStatus::SUCCESS;
		}

		void halt() override
		{
			CoroActionNode::halt();
		}

		std::shared_ptr<bool> _voted;
};

// Set entry to unit vector constant
class Movcv : public CoroActionNode
{
	public:
		Movcv(const std::string& name, const NodeConfiguration& config,
		      std::shared_ptr<bool> voted)
			: CoroActionNode(name, config), _voted(voted) {}

		static PortsList providedPorts()
		{
			PortsList pl;
			pl.insert(OutputPort<VectorPolar>("arg0")); // entry
			pl.insert(InputPort<int>("arg1"));          // angle
			return pl;
		}

	private:
		NodeStatus tick() override
		{
			// Check that vote has not been already cast
			const NodeConfiguration config = this->config();
			auto entry = config.output_ports.find("arg0");
			if (entry->second == "{vvote}")
			{
				if (*(_voted.get()))
				{
					setStatusRunningAndYield();
				}
				else
				{
					*(_voted.get()) = true;
				}
			}

			Optional<int> a = getInput<int>("arg1");
			if (!a)
			{
				throw RuntimeError("missing required input [arg1]: ", a.error());
			}

			// Avoid infinity and NaN
			float angle = M_PI * static_cast<float>(a.value()) / 128.0f;
			if (!std::isfinite(angle))
			{
				angle = 0.0f;
			}
			
			setOutput("arg0", VectorPolar(1.0f, angle));
			return NodeStatus::SUCCESS;
		}

		void halt() override
		{
			CoroActionNode::halt();
		}

		std::shared_ptr<bool> _voted;
};

// Set entry to unit vector constant with random orientation within a sector
class Movpv : public CoroActionNode
{
	public:
		Movpv(const std::string& name, const NodeConfiguration& config,
		      std::shared_ptr<bool> voted)
			: CoroActionNode(name, config), _voted(voted) {}

		static PortsList providedPorts()
		{
			PortsList pl;
			pl.insert(OutputPort<VectorPolar>("arg0")); // entry
			pl.insert(InputPort<VectorPolar>("arg1"));  // s1
			pl.insert(InputPort<int>("arg2"));          // width
			return pl;
		}

	private:
		NodeStatus tick() override
		{
			// Check that vote has not been already cast
			const NodeConfiguration config = this->config();
			auto entry = config.output_ports.find("arg0");
			if (entry->second == "{vvote}")
			{
				if (*(_voted.get()))
				{
					setStatusRunningAndYield();
				}
				else
				{
					*(_voted.get()) = true;
				}
			}

			Optional<VectorPolar> s1 = getInput<VectorPolar>("arg1");
			if (!s1)
			{
				throw RuntimeError("missing required input [arg1]: ", s1.error());
			}

			Optional<int> w = getInput<int>("arg2");
			if (!w)
			{
				throw RuntimeError("missing required input [arg2]: ", w.error());
			}

			float hw = std::abs(M_PI * w.value() / 128.0f);
			float fmin = s1.value().a - hw;
			float fmax = s1.value().a + hw;

			// Avoid infinity and NaN
			float angle = 0.0f;
			if (std::isfinite(fmin) && std::isfinite(fmax))
			{
				angle = rndf(fmin, fmax);
			}

			setOutput("arg0", VectorPolar(1.0f, angle));
			return NodeStatus::SUCCESS;
		}

		void halt() override
		{
			CoroActionNode::halt();
		}

		std::shared_ptr<bool> _voted;
};

// Scalar scale and add
class Mulas : public CoroActionNode
{
	public:
		Mulas(const std::string& name, const NodeConfiguration& config,
		      std::shared_ptr<bool> voted)
			: CoroActionNode(name, config), _voted(voted) {}

		static PortsList providedPorts()
		{
			PortsList pl;
			pl.insert(OutputPort<float>("arg0")); // entry
			pl.insert(InputPort<float>("arg1"));  // s1
			pl.insert(InputPort<float>("arg2"));  // scalar
			pl.insert(InputPort<float>("arg3"));  // s2
			return pl;
		}

	private:
		NodeStatus tick() override
		{
			// Check that vote has not been already cast
			const NodeConfiguration config = this->config();
			auto entry = config.output_ports.find("arg0");
			if (entry->second == "{pvote}")
			{
				if (*(_voted.get()))
				{
					setStatusRunningAndYield();
				}
				else
				{
					*(_voted.get()) = true;
				}
			}

			Optional<float> s1 = getInput<float>("arg1");
			if (!s1)
			{
				throw RuntimeError("missing required input [arg1]: ", s1.error());
			}

			Optional<float> f = getInput<float>("arg2");
			if (!f)
			{
				throw RuntimeError("missing required input [arg2]: ", f.error());
			}

			Optional<float> s2 = getInput<float>("arg3");
			if (!s2)
			{
				throw RuntimeError("missing required input [arg3]: ", s2.error());
			}

			// Avoid infinity and NaN
			float scalar = s1.value() + f.value() * s2.value();
			if (!std::isfinite(scalar))
			{
				scalar = 0.0f;
			}

			setOutput("arg0", scalar);
			return NodeStatus::SUCCESS;
		}

		void halt() override
		{
			CoroActionNode::halt();
		}

		std::shared_ptr<bool> _voted;
};

// Vector scale and add
class Mulav : public CoroActionNode
{
	public:
		Mulav(const std::string& name, const NodeConfiguration& config,
		      std::shared_ptr<bool> voted)
			: CoroActionNode(name, config), _voted(voted) {}

		static PortsList providedPorts()
		{
			PortsList pl;
			pl.insert(OutputPort<VectorPolar>("arg0")); // entry
			pl.insert(InputPort<VectorPolar>("arg1"));  // s1
			pl.insert(InputPort<float>("arg2"));        // scalar
			pl.insert(InputPort<VectorPolar>("arg3"));  // s2
			return pl;
		}

	private:
		NodeStatus tick() override
		{
			// Check that vote has not been already cast
			const NodeConfiguration config = this->config();
			auto entry = config.output_ports.find("arg0");
			if (entry->second == "{pvote}")
			{
				if (*(_voted.get()))
				{
					setStatusRunningAndYield();
				}
				else
				{
					*(_voted.get()) = true;
				}
			}

			Optional<VectorPolar> s1 = getInput<VectorPolar>("arg1");
			if (!s1)
			{
				throw RuntimeError("missing required input [arg1]: ", s1.error());
			}

			Optional<float> f = getInput<float>("arg2");
			if (!f)
			{
				throw RuntimeError("missing required input [arg2]: ", f.error());
			}

			Optional<VectorPolar> s2 = getInput<VectorPolar>("arg3");
			if (!s2)
			{
				throw RuntimeError("missing required input [arg3]: ", s2.error());
			}

			// Avoid infinity and NaN
			VectorPolar v(s1.value() + (s2.value() * f.value()));
			if (!std::isfinite(v.r))
			{
				v.r = 0.0f;
			}

			setOutput("arg0", v);
			return NodeStatus::SUCCESS;
		}

		void halt() override
		{
			CoroActionNode::halt();
		}

		std::shared_ptr<bool> _voted;
};

// Vector rotate and add
class Rotav : public CoroActionNode
{
	public:
		Rotav(const std::string& name, const NodeConfiguration& config,
		      std::shared_ptr<bool> voted)
			: CoroActionNode(name, config), _voted(voted) {}

		static PortsList providedPorts()
		{
			PortsList pl;
			pl.insert(OutputPort<VectorPolar>("arg0")); // entry
			pl.insert(InputPort<VectorPolar>("arg1"));  // s1
			pl.insert(InputPort<int>("arg2"));          // angle
			pl.insert(InputPort<VectorPolar>("arg3"));  // s2
			return pl;
		}

	private:
		NodeStatus tick() override
		{
			// Check that vote has not been already cast
			const NodeConfiguration config = this->config();
			auto entry = config.output_ports.find("arg0");
			if (entry->second == "{vvote}")
			{
				if (*(_voted.get()))
				{
					setStatusRunningAndYield();
				}
				else
				{
					*(_voted.get()) = true;
				}
			}

			Optional<VectorPolar> s1 = getInput<VectorPolar>("arg1");
			if (!s1)
			{
				throw RuntimeError("missing required input [arg1]: ", s1.error());
			}

			Optional<int> i = getInput<int>("arg2");
			if (!i)
			{
				throw RuntimeError("missing required input [arg2]: ", i.error());
			}

			Optional<VectorPolar> s2 = getInput<VectorPolar>("arg3");
			if (!s2)
			{
				throw RuntimeError("missing required input [arg3]: ", s2.error());
			}

			// Avoid infinity and NaN
			VectorPolar s2r = s2.value();
			s2r.rotate(M_PI * static_cast<float>(i.value()) / 128.0f);
			VectorPolar v(s1.value() + s2r);
			if (!std::isfinite(v.r))
			{
				v.r = 0.0f;
			}

			setOutput("arg0", v);
			return NodeStatus::SUCCESS;
		}

		void halt() override
		{
			CoroActionNode::halt();
		}

		std::shared_ptr<bool> _voted;
};


//                 //
// Condition nodes //
//                 //

// Probabilistic success
class Ifprob : public ConditionNode
{
	public:
		Ifprob(const std::string& name, const NodeConfiguration& config)
			: ConditionNode(name, config) {}

		static PortsList providedPorts()
		{
			PortsList pl;
			pl.insert(InputPort<float>("arg0")); // s1
			pl.insert(InputPort<float>("arg1")); // k
			pl.insert(InputPort<float>("arg2")); // l
			return pl;
		}

		NodeStatus tick() override
		{
			Optional<float> s1 = getInput<float>("arg0");
			if (!s1)
			{
				throw RuntimeError("missing required input [arg0]: ", s1.error());
			}

			Optional<float> k = getInput<float>("arg1");
			if (!k)
			{
				throw RuntimeError("missing required input [arg1]: ", k.error());
			}

			Optional<float> l = getInput<float>("arg2");
			if (!l)
			{
				throw RuntimeError("missing required input [arg2]: ", l.error());
			}

			float p_success = 1 / (1 + std::exp(k.value() * (l.value() - s1.value())));
			float rand_sample = rndf(0.0f, 1.0f);

			if (rand_sample > p_success)
			{
				return NodeStatus::FAILURE;
			}

			return NodeStatus::SUCCESS;
		}
};

// Success if a vector exists within a sector
class Ifsect : public ConditionNode
{
	public:
		Ifsect(const std::string& name, const NodeConfiguration& config)
			: ConditionNode(name, config) {}

		static PortsList providedPorts()
		{
			PortsList pl;
			pl.insert(InputPort<VectorPolar>("arg0")); // s1
			pl.insert(InputPort<int>("arg1"));         // i
			pl.insert(InputPort<int>("arg2"));         // j
			return pl;
		}

		NodeStatus tick() override
		{
			Optional<VectorPolar> s1 = getInput<VectorPolar>("arg0");
			if (!s1)
			{
				throw RuntimeError("missing required input [arg0]: ", s1.error());
			}

			Optional<int> i = getInput<int>("arg1");
			if (!i)
			{
				throw RuntimeError("missing required input [arg1]: ", i.error());
			}

			Optional<int> j = getInput<int>("arg2");
			if (!j)
			{
				throw RuntimeError("missing required input [arg2]: ", j.error());
			}

			VectorPolar v = s1.value();
			float center = M_PI * static_cast<float>(i.value()) / 128.0f;
			float hw = std::abs(M_PI * j.value() / 256.0f);

			if (hw == 0)
			{
				if (v.r >= 0.05f)
				{
					return NodeStatus::FAILURE;
				}
			}
			else
			{
				VectorPolar v_diff = VectorPolar(v.r, v.a - center);
				if (v.r < 0.05f || std::abs(v_diff.a) > hw)
				{
					return NodeStatus::FAILURE;
				}
			}

			return NodeStatus::SUCCESS;
		}
};

//            //
// Decorators //
//            //

class Repeati : public DecoratorNode
{
	public:
		Repeati(const std::string& name, const NodeConfiguration& config)
			: DecoratorNode(name, config), num_cycles_(0), try_index_(0) {}

		static PortsList providedPorts()
		{
			return { InputPort<int>("arg0") }; // number of cycles
		}

		NodeStatus tick() override
		{
			if (this->status() == NodeStatus::IDLE)
			{
				Optional<int> n = getInput<int>("arg0");
				if (!n)
				{
					throw RuntimeError("missing required input [arg0]: ",
					                   n.error());
				}

				if (n.value() < 1)
				{
					throw RuntimeError("[arg0] must be a positive integer", "");
				}

				num_cycles_ = n.value();
				setStatus(NodeStatus::RUNNING);
			}

			NodeStatus child_state = child_node_->executeTick();

			switch (child_state)
			{
				case NodeStatus::SUCCESS:
					try_index_++;
					haltChild();

					if (try_index_ == num_cycles_)
					{
						try_index_ = 0;
						return NodeStatus::SUCCESS;
					}

					break;

				case NodeStatus::FAILURE:
					try_index_ = 0;
					haltChild();
					return NodeStatus::FAILURE;

				case NodeStatus::RUNNING:
					break;

				default:
					throw LogicError("A child node must never return IDLE");
			}

			return NodeStatus::RUNNING;
		}

	private:
		int num_cycles_;
		int try_index_;
};

#endif
