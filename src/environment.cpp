#include <iostream>
#include <thread>
#include "environment.h"
#include "arena.h"
#include "utils.h"
#include "bt_custom.h"
#include "callbacks.h"

RobotConstants g_rc;
LoadConstants g_lc;
ArenaConstants g_ac;

Environment::Environment() :
	render(false),
	p_world(new b2World(b2Vec2(0.0f, 0.0f))),
	p_arena(nullptr),
	p_next(0),
	p_f_sim(30.0f),
	p_fitness(0.0f),
	p_window(nullptr) {}

Environment::~Environment()
{
	delete p_world;
	p_world = nullptr;
}

void Environment::addArena()
{
	p_arena = std::make_unique<Arena>(p_world);
}

void Environment::addRobot(const b2Vec2& position, float angle)
{
	std::unique_ptr<Robot> r = std::make_unique<Robot>(p_world, position, angle);
	p_robots.push_back(std::move(r));
}

void Environment::addLoad(const b2Vec2& position, unsigned short porters)
{
	std::unique_ptr<Load> l = std::make_unique<Load>(p_world, position, porters);
	p_loads.push_back(std::move(l));
}

// Returns a pointer to the next robot in the list
Robot* Environment::getNextRobot()
{
	std::size_t it = p_next++;
	if (p_robots.size() > it)
	{
		return p_robots.at(it).get();
	}

	p_next = 0;
	return nullptr;
}

float Environment::run(unsigned long long duration)
{
	// Run rendering in new thread
	std::thread render_thread;
	if (render)
	{
		render_thread = std::thread([this]{ renderSim(); });
		sctime tmp_start = sclock::now();
		sleep(tmp_start, 1.0);
	}

	// Set controller update rate
	unsigned short r = std::ceil(g_rc.t_control / (1.0f / p_f_sim));

	// Run simulation
	p_fitness = 0.0f;
	sctime start = sclock::now();
	for (unsigned long long i = 0; i < duration; ++i)
	{
		if (i % r == 0)
		{
			sense();
			control();
			commsOut();
			commsIn();
			act();
		}

		p_world->Step(1.0f / p_f_sim, 8, 3);

		// Increment load times
		for (auto& load : p_loads)
		{
			// Lifetime
			load->t_life++;

			// Time with lifting points covered
			unsigned short id = load->getBodyData().id;
			for (const auto& robot : p_robots)
			{
				if (robot->scene.id_load ==  id ||
				    robot->message.id_group == id)
				{
					load->t_cover++;
				}
			}
		}

		// Sleep to meet rendering requirements
		if (render)
		{
			sleep(start, 1.0f / p_f_sim);
		}
	}

	// Update fitness
	for (const auto& load : p_loads)
	{
		// Displacement
		float x_init = load->getStartPosition().x;
		float x_final = load->getBody()->GetPosition().x;
		float t_life_si = static_cast<float>(load->t_life) / p_f_sim;
		p_fitness += (x_final - x_init) / (t_life_si * g_rc.v_max);

		// Penalise never being lifted
		if (!load->lifted)
		{
			p_fitness -= 1.0f;
		}

		// Reward begin lowered
		if (load->lowered)
		{
			p_fitness += 1.0f;
		}

		// Lifting point coverage
		float t_coverf = static_cast<float>(load->t_cover);
		float norm_t_coverf = t_coverf / load->getPorters();
		p_fitness += norm_t_coverf / load->t_life;
	}

	if (render)
	{
		render = false;
		render_thread.join();
	}

	return p_fitness;
}

// Removes a load from the world
void Environment::destroyLoad(Load* load)
{
	load->destroy(p_world);
}

// Generates an equal load at the original position
//  -> NOT USED
void Environment::resetLoad(unsigned short id)
{
	b2Vec2 position;
	unsigned short porters;

	for (auto it = p_loads.begin(); it != p_loads.end();)
	{
		unsigned short l_id = it->get()->getBodyData().id;
		if (id == l_id)
		{
			position = it->get()->getStartPosition();
			porters = it->get()->getPorters();

			// Update fitness. Reset called only if load at nest

			// Remove load from current position
			destroyLoad(it->get());
			it = p_loads.erase(it);

			break;
		}
		else
		{
			++it;
		}
	}

	// Create new load
	addLoad(position, porters);
}

// Sense the environment and update each robot's perceived scene
void Environment::sense()
{
	for (auto& robot : p_robots)
	{
		b2Body* rb = robot->getBody();
		unsigned short r_id = robot->getBodyData().id;

		// Reset scene
		robot->scene = Scene();

		// Query colliding AABBs to reduce required number of ray casts
		QueryCallback query_callback(r_id);
		b2AABB robot_AABB;

		const b2Vec2 robot_center = rb->GetWorldCenter();
		robot_AABB.lowerBound.Set(robot_center.x - g_rc.radius - g_rc.prox_max,
		                          robot_center.y - g_rc.radius - g_rc.prox_max);
		robot_AABB.upperBound.Set(robot_center.x + g_rc.radius + g_rc.prox_max,
		                          robot_center.y + g_rc.radius + g_rc.prox_max);

		p_world->QueryAABB(&query_callback, robot_AABB);

		if (query_callback.m_hit)
		{
			// Laser TOF distance sensors
			unsigned short angle_index = 0;
			for (const auto& angle : g_rc.prox_angles)
			{
				RayCastCallbackLow callback_low;
				RayCastCallbackLift callback_lift;
				float fraction;
				b2Vec2 point;

				b2Vec2 p1 = robot->getLaserStart(angle);
				b2Vec2 p2 = robot->getLaserEnd(angle);
				if (robot->platform_up)
				{
					p_world->RayCast(&callback_lift, p1, p2);
					fraction = callback_lift.m_fraction;
					point = callback_lift.m_point;
				}
				else
				{
					p_world->RayCast(&callback_low, p1, p2);
					fraction = callback_low.m_fraction;
					point = callback_low.m_point;
				}

				// Map of distances
				robot->scene.prox[angle_index++] = 1.0f - fraction;
			}
		}

		// Orientation
		robot->scene.orientation = robot->getOrientation();

		// Heading
		robot->scene.heading = robot->getHeading();

		// Process camera feeds
		for (b2ContactEdge* ce = rb->GetContactList(); ce; ce = ce->next)
		{
			b2Contact* c = ce->contact;

			// Contacts are generated from AABB overlaps, so make sure sensors
			// are actually touching before proceeding
			if (!c->IsTouching())
			{
				continue;
			}

			b2Fixture* fA = c->GetFixtureA();
			uintptr_t fA_data = fA->GetUserData().pointer;
			FixtureData* fdA = reinterpret_cast<FixtureData*>(fA_data);

			b2Fixture* fB = c->GetFixtureB();
			uintptr_t fB_data = fB->GetUserData().pointer;
			FixtureData* fdB = reinterpret_cast<FixtureData*>(fB_data);

			b2Body* b = ce->other;
			uintptr_t b_data = b->GetUserData().pointer;
			BodyData* bd = reinterpret_cast<BodyData*>(b_data);
			b2Transform transform = b->GetTransform();

			// Neighbour detection
			if (((fdA->type == BODYCAM && fdA->body_id == r_id) &&
			     fdB->type == BODY) || 
			    ((fdB->type == BODYCAM && fdB->body_id == r_id) &&
			     fdA->type == BODY))
			{
				bool exists = false;

				for (const auto& neighbour : robot->scene.neighbours)
				{
					if (neighbour.id == bd->id)
					{
						exists = true;
						break;
					}	
				}

				if (!exists)
				{
					b2Vec2 world_pos = b->GetPosition();
					b2Vec2 local_pos = rb->GetLocalPoint(world_pos);
					VectorPolar neighbour_pos = VectorPolar(local_pos);
					neighbour_pos.r -= 2 * g_rc.radius;

					Neighbour neighbour;
					neighbour.id = bd->id;
					neighbour.rb = neighbour_pos;
					robot->scene.neighbours.push_back(neighbour);
					++(robot->scene.n);
				}
			}

			// Nest detection
			if ((fdA->type == BODYCAM && fdA->body_id == r_id) &&
			     fdB->type == MARKER)
			{
				// Nest position
				const b2AABB aabb = fB->GetAABB(0);
				b2Vec2 world_lp_center = aabb.GetCenter();
				b2Vec2 local_lp_center = rb->GetLocalPoint(world_lp_center);

				// Store only closest marker position
				VectorPolar tmp_rb_nest = VectorPolar(local_lp_center);
				tmp_rb_nest.r -= g_rc.radius;
				if (robot->scene.nest)
				{
					if (tmp_rb_nest.r < robot->scene.rb_nest.r)
					{
						robot->scene.rb_nest = tmp_rb_nest;
					}
				}
				else
				{
					robot->scene.rb_nest = tmp_rb_nest;
				}

				robot->scene.nest = true;
			}
			else if ((fdB->type == BODYCAM && fdB->body_id == r_id) &&
			         fdA->type == MARKER)
			{
				// Nest position
				const b2AABB aabb = fA->GetAABB(0);
				b2Vec2 world_lp_center = aabb.GetCenter();
				b2Vec2 local_lp_center = rb->GetLocalPoint(world_lp_center);

				// Store only closest marker position
				VectorPolar tmp_rb_nest = VectorPolar(local_lp_center);
				tmp_rb_nest.r -= g_rc.radius;
				if (robot->scene.nest)
				{
					if (tmp_rb_nest.r < robot->scene.rb_nest.r)
					{
						robot->scene.rb_nest = tmp_rb_nest;
					}
				}
				else
				{
					robot->scene.rb_nest = tmp_rb_nest;
				}

				robot->scene.nest = true;
			}

			// Lifting point detection
			if (((fdA->type == PLATCAM && fdA->body_id == r_id) &&
			     fdB->type == MARKER) ||
			    ((fdB->type == PLATCAM && fdB->body_id == r_id) &&
			     fdA->type == MARKER))
			{
				// Get load ID
				robot->scene.id_load = bd->id;

				// Number of porters required
				for (const auto& load : p_loads)
				{
					if (load->getBodyData().id == bd->id)
					{
						robot->scene.porters = load->getPorters();
						break;
					}
				}
			}

			// Load detection
			if (((fdA->type == PLATCAM && fdA->body_id == r_id) &&
			     fdB->type == BODY) ||
			    ((fdB->type == PLATCAM && fdB->body_id == r_id) &&
			     fdA->type == BODY))
			{
				robot->scene.load = true;
				VectorPolar nearest = VectorPolar(99.0f, 0.0f);
				for (b2Fixture* f = b->GetFixtureList(); f; f = f->GetNext())
				{
					uintptr_t data = f->GetUserData().pointer;
					FixtureData* fd = reinterpret_cast<FixtureData*>(data);

					if (fd->type == MARKER)
					{
						b2CircleShape* circle = reinterpret_cast<b2CircleShape*>(f->GetShape());
						b2Vec2 centre = b2Mul(transform, circle->m_p);
						b2Vec2 local = rb->GetLocalPoint(centre);
						VectorPolar v_local = VectorPolar(local);

						if (v_local.r < nearest.r)
						{
							nearest = v_local;
						}
					}
				}

				robot->scene.rb_lift = nearest;
			}
		}
	}
}

// Update each robot's blackboard and tick its behaviour tree
void Environment::control()
{
	for (auto& robot : p_robots)
	{
		robot->controller.updateBB(robot->scene, robot->message, robot->comms);
		robot->controller.tick();
	}
}

// Update each robot's message and add it to its neighbours comms list
void Environment::commsOut()
{
	// Update messages and clear old comms
	for (auto& robot : p_robots)
	{
		robot->setMessage();
		robot->comms.clear();
	}

	// Broadcast
	for (auto& robot : p_robots)
	{
		b2Vec2 p1 = robot->getBody()->GetPosition();

		for (const auto& other : p_robots)
		{
			if (other == robot)
			{
				continue;
			}

			b2Vec2 p2 = other->getBody()->GetPosition();
			float distance = (p2 - p1).Length();
			if (distance <= g_rc.comms_range)
			{
				robot->comms.push_back(other->message);
			}
		}
	}
}

// Use received messages to update the Group ID and compute output from votes
void Environment::commsIn()
{
	for (auto& robot : p_robots)
	{
		robot->processComms();
	}
}

// Act on every robot according to the output of the control step
void Environment::act()
{
	// ID of loads placed at nest
	std::vector<unsigned short> load_id;

	for (auto& robot : p_robots)
	{
		float p_goal = robot->message.p_vote;
		if (p_goal == 0.0f)
		{
			// Wheels
			VectorPolar v_goal = robot->message.v_vote;

			// Transform to local vector
			v_goal.rotate(robot->scene.heading - robot->scene.orientation);
			robot->setVelocity(v_goal);
		}
		else
		{
			// Stop the wheels
			robot->setVelocity(VectorPolar(0.0f, 0.0f));

			// Platform
			if (p_goal < 0.0f && robot->platform_up)
			{
				// Detach load from robot
				unsigned short porters = robot->getLoad()->getPorters();
				unsigned short tmp_id = robot->detachLoad(p_world);
				robot->platform_up = false;

				for (auto& load : p_loads)
				{
					if (load->getBodyData().id == tmp_id)
					{
						load->lowered = true;
						break;
					}
				}

				// If any of the robots are at the nest, add load to list
				if (robot->scene.rb_nest.r > 0.0f &&
				    robot->scene.rb_nest.r < g_ac.nest_r)
				{
					bool exists = false;
					for (const auto& id : load_id)
					{
						if (id == tmp_id)
						{
							exists = true;
							break;
						}
					}

					if (!exists)
					{
						load_id.push_back(tmp_id);
					}
				}
			}
			else if (p_goal > 0.0f && !robot->platform_up)
			{
				for (auto& load : p_loads)
				{
					if (load->getBodyData().id == robot->message.id_group)
					{
						// Attach load to robot
						robot->attachLoad(p_world, load.get());
						robot->platform_up = true;

						load->lifted = true;
						break;
					}
				}
			}
		}
	}

	// Reset/Delete loads placed at nest
	for (const auto& id : load_id)
	{
		//resetLoad(id);

		for (auto it = p_loads.begin(); it != p_loads.end();)
		{
			unsigned short l_id = it->get()->getBodyData().id;
			if (id == l_id)
			{
				// Update fitness:
				// Displacement
				float x_init = it->get()->getStartPosition().x;
				float x_final = it->get()->getBody()->GetPosition().x;
				float t_life_si = static_cast<float>(it->get()->t_life) / p_f_sim;
				p_fitness += (x_final - x_init) / (t_life_si * g_rc.v_max);

				// Lifting point coverage
				float t_coverf = static_cast<float>(it->get()->t_cover);
				float norm_t_coverf = t_coverf / it->get()->getPorters();
				p_fitness += norm_t_coverf / it->get()->t_life;

				// Reward being lowered
				p_fitness += 1.0f;

				// Remove load from current position
				destroyLoad(it->get());
				it = p_loads.erase(it);

				break;
			}
			else
			{
				++it;
			}
		}
	}
}

void Environment::renderSim()
{
	if (!render)
	{
		return;
	}

	// Initialise GLFW window
	if (glfwInit() == 0)
	{
		throw std::runtime_error("Failed to initialise GLFW");
	}

	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
	glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
	glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GLFW_TRUE);
	glfwWindowHint(GLFW_RESIZABLE, GLFW_FALSE);

	g_camera.m_width = 800; // hard-coded for now
	g_camera.m_height = 800; // hard-coded for now
	p_window = glfwCreateWindow(g_camera.m_width,
	                            g_camera.m_height, "OpenGL", NULL, NULL);
	if (p_window == NULL)
	{
		glfwTerminate();
		throw std::runtime_error("Failed to open window");
	}
	glfwMakeContextCurrent(p_window);

	// Initialise GLEW
	glewExperimental = GL_TRUE;
	glewInit();

	// Initialise debug draw
	g_debug_draw.Create();
	p_world->SetDebugDraw(&g_debug_draw);

	// Colours
	b2Color black = b2Color(0.0f, 0.0f, 0.0f);
	b2Color red = b2Color(1.0f, 0.0f, 0.0f);
	b2Color green = b2Color(0.0f, 0.7f, 0.0f);
	b2Color orange = b2Color(1.0f, 0.6f, 0.0f);
	b2Color blue = b2Color(0.0f, 0.0f, 1.0f);

	// Arena vertices
	b2Vec2 a_v[4];
	a_v[0].Set(-g_ac.width / 2.0f,  g_ac.length / 2.0f);
	a_v[1].Set( g_ac.width / 2.0f,  g_ac.length / 2.0f);
	a_v[2].Set( g_ac.width / 2.0f, -g_ac.length / 2.0f);
	a_v[3].Set(-g_ac.width / 2.0f, -g_ac.length / 2.0f);

	// Rendering loop
	while (render)
	{
		glfwGetWindowSize(p_window, &g_camera.m_width, &g_camera.m_height);

		int buffer_width, buffer_height;
		glfwGetFramebufferSize(p_window, &buffer_width, &buffer_height);
		glViewport(0, 0, buffer_width, buffer_height);

		glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

		// Draw arena
		g_debug_draw.DrawPolygon(a_v, 4, b2Color(0.0f, 0.0f, 0.0f));

		b2Vec2 p1 = b2Vec2((g_ac.width / 2.0f) - g_ac.nest_r,
		                     g_ac.length / 2.0f);
		b2Vec2 p2 = b2Vec2((g_ac.width / 2.0f) - g_ac.nest_r,
		                    -g_ac.length / 2.0f);
		g_debug_draw.DrawSegment(p1, p2, green);

		// Draw loads
		for (const auto& load : p_loads)
		{
			b2Body* body = load->getBody();
			b2Transform transform = body->GetTransform();

			for (b2Fixture* f = body->GetFixtureList(); f; f = f->GetNext())
			{
				b2Shape* shape = f->GetShape();
				if (shape->m_type == 0)
				{
					b2CircleShape* circle = reinterpret_cast<b2CircleShape*>(shape);
					b2Vec2 centre = b2Mul(transform, circle->m_p);
					g_debug_draw.DrawCircle(centre, circle->m_radius, blue);
				}
				else
				{
					b2PolygonShape* poly = reinterpret_cast<b2PolygonShape*>(shape);
					int n_vertices = poly->m_count;
					b2Vec2 vertices[n_vertices];

					for (auto i = 0; i < n_vertices; ++i)
					{
						vertices[i] = b2Mul(transform, poly->m_vertices[i]);
					}

					g_debug_draw.DrawPolygon(vertices, n_vertices, blue);
				}
			}
		}

		// Draw robots
		for (const auto& robot : p_robots)
		{
			b2Body* body = robot->getBody();
			b2Transform transform = body->GetTransform();

			for (b2Fixture* f = body->GetFixtureList(); f; f = f->GetNext())
			{
				uintptr_t data = f->GetUserData().pointer;
				FixtureData* fd = reinterpret_cast<FixtureData*>(data);
				FixtureType type = fd->type;
				if (type == BODY)
				{
					b2CircleShape* circle = reinterpret_cast<b2CircleShape*>(f->GetShape());
					b2Vec2 centre = b2Mul(transform, circle->m_p);
					b2Color colour = robot->message.id_group ? orange : red;
					g_debug_draw.DrawCircle(centre, circle->m_radius, colour);

					break;
				}
			}
		}

		// Draw
		p_world->DebugDraw();
		g_debug_draw.Flush();

		glfwSwapBuffers(p_window);
		glfwPollEvents();
	}

	// Clean up
	g_debug_draw.Destroy();
	glfwTerminate();
}
