#ifndef ROBOT_H
#define ROBOT_H

#include <memory>
#include "sim_object.h"
#include "load.h"
#include "controller.h"
#include "pid.h"

class Robot : public SimObject
{
	public:
		Robot(b2World* world, const b2Vec2& position, float angle);
		~Robot();
	
		void setVelocity(const VectorPolar& v_local);
		b2Vec2 getLaserStart(float angle);
		b2Vec2 getLaserEnd(float angle);
		float getOrientation();
		float getHeading();
		void setMessage();
		void processComms();
		void attachLoad(b2World* world, Load* load);
		unsigned short detachLoad(b2World* world);
		Load* getLoad();
		b2Transform getPose();
	
		Controller controller;
		Message message;
		std::vector<Message> comms;
		Scene scene;
		bool platform_up;
		unsigned short wait_count;
		void aquired_entity(uintptr_t e, b2Fixture *sf, b2Fixture *df);
		void lost_entity(uintptr_t e, b2Fixture *sf, b2Fixture *df);
		std::vector<SensorData> visible_entities;
	
	private:
		b2Vec3 holonomicFK3(const b2Vec3& v_wheels);
		b2Vec3 holonomicIK3(const b2Vec3& v_local);
		void addPlatformCamera();
		void removePlatformCamera();
	
		float p_sdev;
		PID p_pid_v;
		PID p_pid_vn;
		b2Vec3 p_v_wheels;
		b2Fixture* p_pcam;
		b2FrictionJoint* p_joint;
		Load* p_load;
		std::unique_ptr<b2MassData> p_porter_mass;
		static short p_next_id;
};

#endif
