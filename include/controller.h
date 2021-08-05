#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <memory>
#include "common.h"
#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/loggers/bt_cout_logger.h>

class Controller
{
	public:
		Controller();

		void updateSource();
		void updateBB(const Scene& scene, const Message& message,
		              const std::vector<Message>& comms);
		const BT::Blackboard::Ptr getBB();
		void printBB();
		void guiBB(int x, int y);
		void tick();

		std::string xml_tree;
		std::shared_ptr<bool> voted;

	private:
		BT::BehaviorTreeFactory p_factory;
		BT::Tree p_tree;
		std::unique_ptr<BT::StdCoutLogger> p_logger;
};


#endif
