#include "robot.h"

Robot::Robot(b2World* world, const b2Vec2& position, float angle) :
	p_pid_v(30.0f, 0.0f, 0.0f, g_rc.t_control, ""),
	p_pid_vn(30.0f, 0.0f, 0.0f, g_rc.t_control, ""),
	platform_up(false),
	p_v_wheels(0.0f, 0.0f, 0.0f),
	p_pcam(nullptr),
	p_joint(nullptr),
	p_load(nullptr),
	p_porter_mass(std::make_unique<b2MassData>())
{
	// Set body and fixture data
	p_body_data.type = ROBOT;
	p_body_data.id = p_next_id;

	p_fixture_data.resize(6);
	p_fixture_data.at(0).type = BODY;
	p_fixture_data.at(0).body_id = p_next_id;
	for (std::size_t i = 1; i < p_fixture_data.size()-1; ++i)
	{
		p_fixture_data.at(i).type = BODYCAM;
		p_fixture_data.at(i).body_id = p_next_id;
	}
	p_fixture_data.at(5).type = PLATCAM;
	p_fixture_data.at(5).body_id = p_next_id;

	// Create robot body
	b2BodyDef robot_def;
	robot_def.type = b2_dynamicBody;
	robot_def.position.Set(position.x, position.y);
	robot_def.angle = angle;
	robot_def.userData.pointer = reinterpret_cast<uintptr_t>(&p_body_data);

	p_body = world->CreateBody(&robot_def);

	b2CircleShape robot_shape;
	robot_shape.m_radius = g_rc.radius;

	b2FixtureDef robot_fixture_def;
	robot_fixture_def.shape = &robot_shape;
	robot_fixture_def.density = g_rc.density;
	robot_fixture_def.friction = g_rc.friction;
	robot_fixture_def.restitution = g_rc.restitution;
	// Robots collide with the arena and with other robots
	robot_fixture_def.filter.categoryBits = 0x0001;
	robot_fixture_def.filter.maskBits = 0x0023;
	robot_fixture_def.userData.pointer = reinterpret_cast<uintptr_t>(&p_fixture_data.at(0));

	p_body->CreateFixture(&robot_fixture_def);

	// Body cameras as wedge-shaped sensors
	b2Vec2 points[b2_maxPolygonVertices]; // 8 vertices
	float d_angle = g_rc.cam_fov / (b2_maxPolygonVertices - 2);
	for (short i = 0; i < b2_maxPolygonVertices-1; i++)
	{
		points[i].Set(g_rc.cam_max * std::cos(i * d_angle - g_rc.cam_fov / 2), 
	                  g_rc.cam_max * std::sin(i * d_angle - g_rc.cam_fov / 2));
	}
	points[b2_maxPolygonVertices-1].Set(0.0, 0.0);

	// Add body cameras
	unsigned short cam_n = 1;
	for (const auto& ca : g_rc.cam_angles)
	{
		// Transform points
		b2Vec2 pos(g_rc.radius * std::cos(ca), g_rc.radius * std::sin(ca));
		b2Rot rot(ca);
		b2Transform transform(pos, rot);

		b2Vec2 cam_points[b2_maxPolygonVertices];
		for (std::size_t i = 0; i < sizeof(points)/sizeof(points[0]); i++)
		{
			cam_points[i] = b2Mul(transform, points[i]);
		}

		// Add camera
		b2PolygonShape cam_shape;
		cam_shape.Set(cam_points, b2_maxPolygonVertices);

		b2FixtureDef cam_fixture_def;
		cam_fixture_def.shape = &cam_shape;
		cam_fixture_def.isSensor = true;
		// Body camera collides with the nest and robot bodies
		cam_fixture_def.filter.categoryBits = 0x0002;
		cam_fixture_def.filter.maskBits = 0x0041;
		cam_fixture_def.userData.pointer = reinterpret_cast<uintptr_t>(&p_fixture_data.at(cam_n));

		p_body->CreateFixture(&cam_fixture_def);

		++cam_n;
	}

	// Platform camera as rectangular sensor (projection on load bottom)
	addPlatformCamera();

	// Initialise message
	message.id_messenger = p_next_id;

	// Update ID
	p_next_id++;
}

Robot::~Robot()
{
	p_pcam = nullptr;
	p_joint = nullptr;
}

// Given a local goal vector, set wheel velocities and apply friction
void Robot::setVelocity(const VectorPolar& v_local)
{
	// Scale goal velocity with maximum robot velocity
	VectorPolar v_goal = v_local;
	if (v_local.r > 1.0f)
	{
		v_goal.r = 1.0f;
	}
	v_goal.r *= g_rc.v_max;

	// Calculate required wheel linear velocities
	b2Vec3 v_goal_b2 = b2Vec3(v_goal.r * std::cos(v_goal.a),
	                          v_goal.r * std::sin(v_goal.a),
	                          0.0f);
	p_v_wheels = holonomicIK3(v_goal_b2);

	// Holonomic robot, so use PID to set the velocity
	b2Vec2 v_ref = v_goal.to_b2Vec2();
	b2Vec2 v_global = p_body->GetLinearVelocity();
	b2Vec2 v_meas = p_body->GetLocalVector(v_global);

	float f_forward = p_pid_v.update(v_ref.x, v_meas.x);
	float f_normal = p_pid_vn.update(v_ref.y, v_meas.y);
	b2Vec2 fg = p_body->GetWorldVector(b2Vec2(f_forward, f_normal));

	p_body->ApplyForceToCenter(fg, true);
}

// Return laser TOF distance sensor position in world coordinates
b2Vec2 Robot::getLaserStart(float angle)
{
	b2Vec2 pos((g_rc.radius - 0.001) * std::cos(angle),
	           (g_rc.radius - 0.001) * std::sin(angle));
	return p_body->GetWorldPoint(pos);
	
}

// Return laser TOF distance sensor endpoint in world coordinates
b2Vec2 Robot::getLaserEnd(float angle)
{
	b2Vec2 ep((g_rc.radius + g_rc.prox_max) * std::cos(angle),
	          (g_rc.radius + g_rc.prox_max) * std::sin(angle));
	return p_body->GetWorldPoint(ep);
}

// Return orientation of the robot (wrt x-axis)
float Robot::getOrientation()
{
	// Create vector to ensure [-pi,pi) range
	VectorPolar v(1.0f, p_body->GetAngle());
	return v.a;
}

// Return heading of the robot (wrt x-axis)
float Robot::getHeading()
{
	float orientation = getOrientation();
	b2Vec3 v_local = holonomicFK3(p_v_wheels);
	float heading_local = std::atan2(v_local.y, v_local.x);
	float heading = heading_local + orientation;

	// Create vector to ensure [-pi,pi) range
	VectorPolar v(1.0f, heading);

	return v.a;
}

// Set message contents for broadcasting: load ID and votes
void Robot::setMessage()
{
	// Set load ID from sensing
	if (scene.rb_lift.r < g_lc.marker_w / 2.0f)
	{
		message.id_load = scene.id_load;
	}
	else
	{
		message.id_load = 0;
	}

	// Set output from votes
	const auto& bb = controller.getBB();

	message.v_vote = bb->get<VectorPolar>("vvote");
	message.v_vote.rotate(scene.heading);
	message.p_vote = bb->get<float>("pvote");
}

// Update Group ID and compute output variables from received messages
void Robot::processComms()
{
	// Update Group ID
	if (!platform_up)
	{
		if (message.id_group == 0) // set Group ID
		{
			if (message.id_load != 0)
			{
				unsigned short ready_count = 1;
				for (const auto& msg : comms)
				{
					if (msg.id_load == scene.id_load)
					{
						++ready_count;
					}
				}

				if (ready_count == scene.porters)
				{
					message.id_group = scene.id_load;
				}
			}
		}
		else // unset Group ID
		{
			if (message.id_load == 0)
			{
				message.id_group = 0;
			}
			else
			{
				unsigned short ready_count = 1;
				for (const auto& msg : comms)
				{
					if (msg.id_load == scene.id_load)
					{
						++ready_count;
					}
				}

				if (ready_count < scene.porters)
				{
					message.id_group = scene.id_load;
				}
			}
		}
	}

	// Compute consensus platform and robot velocities
	VectorPolar v_cons = message.v_vote;
	float p_cons = message.p_vote;

	if (message.id_group != 0)
	{
		for (const auto& msg : comms)
		{
			if (msg.id_group == message.id_group)
			{
				v_cons += msg.v_vote;
				p_cons += msg.p_vote;
			}
		}
	}

	message.v_vote = v_cons;
	message.v_vote.rotate(-scene.heading);
	message.p_vote = p_cons;
}

// Create revolute joint between robot and given load
void Robot::attachLoad(b2World* world, Load* load)
{
	// Attach load fixture
	if (load->getBodyData().id != scene.id_load)
	{
		throw std::runtime_error("Cannot attach given load");
	}

	b2FrictionJointDef joint_def;
	joint_def.Initialize(p_body, load->getBody(), p_body->GetWorldCenter());
	joint_def.maxForce = 1000.0f;
	joint_def.maxTorque = 1000.0f;

	p_joint = (b2FrictionJoint*)world->CreateJoint(&joint_def);
	p_load = load;

	// Remove platform camera
	removePlatformCamera();

	// Modify robot mass
	float load_mass = load->getBody()->GetMass() / load->getPorters();
	p_porter_mass->mass = p_body->GetMass() + load_mass;
	p_porter_mass->center = b2Vec2(0.0f, 0.0f);
	p_porter_mass->I = 0.5 * p_porter_mass->mass * g_rc.radius * g_rc.radius;
	p_body->SetMassData(p_porter_mass.get());
}

// Destroy revolute joint between robot and load
unsigned short Robot::detachLoad(b2World* world)
{
	// Store detached load id for return
	uintptr_t data = p_joint->GetBodyB()->GetUserData().pointer;
	BodyData* b_data = reinterpret_cast<BodyData*>(data);
	unsigned short id = b_data->id;

	world->DestroyJoint(p_joint);
	p_joint = nullptr;
	p_load = nullptr;

	// Add platform camera
	addPlatformCamera();

	// Modify robot mass
	p_body->ResetMassData();

	return id;
}

// Return a pointer to the load attached
Load* Robot::getLoad()
{
	return p_load;
}

// Forward kinematics
b2Vec3 Robot::holonomicFK3(const b2Vec3& v_wheels)
{
	b2Vec3 c1(-std::sqrt(3.0f) / 3.0f,
	          1.0f / 3.0f,
	          1.0f / (3.0f * g_rc.d_wheel));
	b2Vec3 c2(0.0f,
	          -2.0f / 3.0f,
	          1.0f / (3.0f * g_rc.d_wheel));
	b2Vec3 c3(std::sqrt(3.0f) / 3.0f,
	          1.0f / 3.0f,
	          1.0f / (3.0f * g_rc.d_wheel));
	b2Mat33 m(c1, c2, c3);
	b2Vec3 v_local(b2Mul(m, v_wheels));
	return v_local;
}

// Inverse kinematics
b2Vec3 Robot::holonomicIK3(const b2Vec3& v_local)
{
	b2Vec3 c1(-std::sin(M_PI / 3.0f), 0.0f, std::sin(M_PI / 3.0f));
	b2Vec3 c2(std::cos(M_PI / 3.0f), -1.0f, std::cos(M_PI / 3.0f));
	b2Vec3 c3(g_rc.d_wheel, g_rc.d_wheel, g_rc.d_wheel);
	b2Mat33 m(c1, c2, c3);
	b2Vec3 v_wheels(b2Mul(m, v_local));
	return v_wheels;
}

// Add rectangular sensor to detect loads and load markers
void Robot::addPlatformCamera()
{
	float pcam_w = (g_lc.height-g_rc.height) * 2 * std::tan(g_rc.pcam_fov_h/2);
	float pcam_l = (g_lc.height-g_rc.height) * 2 * std::tan(g_rc.pcam_fov_v/2);
	b2Vec2 p_points[4];
	p_points[0].Set( pcam_l / 2,  pcam_w / 2);
	p_points[1].Set( pcam_l / 2, -pcam_w / 2);
	p_points[2].Set(-pcam_l / 2,  pcam_w / 2);
	p_points[3].Set(-pcam_l / 2, -pcam_w / 2);

	b2PolygonShape pcam_shape;
	pcam_shape.Set(p_points, 4);

	b2FixtureDef pcam_fixture_def;
	pcam_fixture_def.shape = &pcam_shape;
	pcam_fixture_def.isSensor = true;
	// Platform camera collides with loads and lifting points
	pcam_fixture_def.filter.categoryBits = 0x0004;
	pcam_fixture_def.filter.maskBits = 0x0018;
	pcam_fixture_def.userData.pointer = reinterpret_cast<uintptr_t>(&p_fixture_data.back());

	p_pcam = (b2Fixture*)p_body->CreateFixture(&pcam_fixture_def);
}

void Robot::removePlatformCamera()
{
	p_body->DestroyFixture(p_pcam);
	p_pcam = nullptr;
}

short Robot::p_next_id = 1;
