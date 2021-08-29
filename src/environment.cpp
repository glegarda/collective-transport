#include <iostream>
#include <thread>
#include <cstdio>
#include "environment.h"
#include "arena.h"
#include "utils.h"
#include "bt_custom.h"
#include "callbacks.h"

#include <math.h>


RobotConstants g_rc;
LoadConstants g_lc;
ArenaConstants g_ac;

SimParams g_sp;


Environment::Environment(CommandLineParams &g_cmd) :
    render(g_cmd.gui),
	p_world(new b2World(b2Vec2(0.0f, 0.0f))),
	p_arena(nullptr),
	p_next(0),
	p_f_sim(g_cmd.hz),
	p_fitness(0.0f),
	p_window(nullptr),
    new_prox(g_cmd.new_prox),
    g_cmd(g_cmd)
{
    physics_per_ctrl = std::ceil(g_rc.t_control / (1.0f / p_f_sim));
    p_world->SetContactListener(&contactlistener_inst);
}

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


void Environment::addRandomRobots()
{
    // Brute force a random placement
    const float initial_min_x   = 0.0;
    const float initial_max_x   = 2.2;
    const float initial_min_y   = -2.2;
    const float initial_max_y   = 2.2;
    const float min_sep         = 0.6;
    std::vector<b2Vec2> positions(g_cmd.num_agents);
    int good_points;
    for(int trials = 0; trials < 100; trials++)
    {
        good_points = 1;
        positions[0] = b2Vec2(rndf(initial_min_x, initial_max_x), rndf(initial_min_y, initial_max_y));
        int attempts = 0;
        while ((good_points < g_cmd.num_agents) && (attempts++ < 10000))
        {
            positions[good_points] = b2Vec2(rndf(initial_min_x, initial_max_x), rndf(initial_min_y, initial_max_y));
            // Check
            bool ok = true;
            for(int i = 0; i <= good_points; i++)
            {
                for(int j = i + 1; j <= good_points; j++)
                {
                    auto d = b2Distance(positions[i], positions[j]);
                    //printf("%d %d %d %2d %2d %f\n", trials, attempts, good_points, i, j, d);
                    if (d < min_sep)
                    {
                        ok = false;
                        break;
                    }
                }
                if (!ok) break;
            }
            if (ok) good_points++;
        }
        if (good_points == g_cmd.num_agents)
            break;
    }
    if (good_points < g_cmd.num_agents)
    {
        printf("Failed to find placement solution! Only %d\n", good_points);
        exit(1);
    }
    for(int n = 0; n < g_cmd.num_agents; n++)
    {
        addRobot(positions[n], rndf(-M_PI, M_PI));
    }
}

void Environment::addLoad(const b2Vec2& position, unsigned short porters,
	                      float dx_init)
{
	std::unique_ptr<Load> l = std::make_unique<Load>(p_world, position, porters,
	                                                 dx_init);
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

void Environment::initFitness()
{
    p_fitness = 0.0;
}

void Environment::updateFitness()
{
    // Increment load times
    for (auto& load : p_loads)
    {
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
}

float Environment::finalFitness()
{
    // Update fitness
    float load_velocity = 0;
    float load_coverage = 0;
    float load_actions = 0;
    for (const auto& load : p_loads)
    {
        // Displacement
        float x_init = load->getStartPosition().x;
        float x_final = load->getBody()->GetPosition().x;
        load_velocity += (x_final - x_init + load->dx) / (g_cmd.simtime * g_rc.v_max);
        
        // Penalise never being lifted
        if (!load->lifted)
        {
            load_actions -= 1.0f;
        }
        
        // Reward begin lowered
        if (load->lowered)
        {
            load_actions += 1.0f;
        }
        
        // Lifting point coverage
        float t_coverf = static_cast<float>(load->t_cover);
        float norm_t_coverf = t_coverf / load->getPorters();
        load_coverage += norm_t_coverf / (g_cmd.simtime * p_f_sim);
    }
    
    p_fitness = load_velocity + load_coverage + load_actions;
    if (g_cmd.verbose)
        printf("% 8f % 8f % 8f % 8f\n", load_velocity, load_coverage, load_actions, p_fitness);

    return p_fitness;
}

void Environment::do_logging(unsigned long long step)
{
    if (!logfile) return;
    if (g_cmd.loglevel == 0)
    {
        for(int i = 0; i < p_robots.size(); i++)
        {
            auto &r = p_robots[i];
            auto p = r->getPose();
            fprintf(logfile, "r%02d %10llu % 6.3f % 6.3f % 6.3f\n", i, step, p.p.x, p.p.y, p.q.GetAngle());
        }
    }
}

float Environment::run(unsigned long long duration)
{
    // Run the simulation for duration physics timesteps.
    // If render is active, use a GUI to control the sim,
    // with fractional to multiple timesteps per rendered frame.
    // Displaying at 1x requires two frames per timestep, since
    // the default physics timestep is 30Hz
    if (g_cmd.logfile.size())
    {
        logfile = fopen(g_cmd.logfile.c_str(), "w");
    }
    
    sctime start = sclock::now();
    initFitness();
    
    renderSetup();
	for (uint32_t i = 0; i < duration; )
	{
		if (render) renderPredraw();
        if (!render || (!g_sp.paused || (g_sp.paused && g_sp.step)))
        {
            do_logging(i);
            simStep();
            updateFitness();
			resetLoads();
            g_sp.step = false;
            //printf("Loop %5u time %f\r", i, g_sp.model_time);
            i++;
        }
		if (render) renderStep();
	}
    //printf("\n");
    
    if (g_cmd.logfile.size())
    {
        fclose(logfile);
    }
    renderCleanup();

	return finalFitness();
}

void Environment::simStep()
{
    // Run one physics step, then, if necessary, run controller
    p_world->Step(1.0f / p_f_sim, g_cmd.vi, g_cmd.pi);
    if (loop % physics_per_ctrl == 0)
    {
        sctime start = sclock::now();
        sense();
        control();
        commsOut();
        commsIn();
        act();
        sctime end = sclock::now();
        g_sp.model_time = delta(start, end);
        g_sp.push_mt(g_sp.model_time);
    }
}

// Removes a load from the world
void Environment::destroyLoad(Load* load)
{
	load->destroy(p_world);
}

// Generates an equal load at the original position
void Environment::resetLoads()
{
	std::vector<b2Vec2> positions;
	std::vector<unsigned short> porters;
	std::vector<float> dx_final;

	for (auto it = p_loads.begin(); it != p_loads.end();)
	{
		if (it->get()->reset)
		{
			// Store relevant data
			positions.push_back(it->get()->getStartPosition());
			porters.push_back(it->get()->getPorters());
        	float x_init = it->get()->getStartPosition().x;
			float x_final = it->get()->getBody()->GetPosition().x;
        	dx_final.push_back(x_final - x_init + it->get()->dx);

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

	// Create new loads
	for (int i = 0; i < positions.size(); ++i)
	{
		addLoad(positions.at(i), porters.at(i), dx_final.at(i));
	}
}


bool get_sensed_entity(b2Contact *c, Robot *&robot, b2Fixture *&sensor_fixture,  uintptr_t &entity, b2Fixture *&detected_fixture)
{
    b2Fixture* fA = c->GetFixtureA();
    b2Fixture* fB = c->GetFixtureB();
    // Check one and only one fixture is a sensor
    bool sensorA = fA->IsSensor();
    bool sensorB = fB->IsSensor();
//    printf("%02x:%02x %02x:%02x\n", fA->GetFilterData().categoryBits, fA->GetFilterData().maskBits, fB->GetFilterData().categoryBits, fA->GetFilterData().maskBits);
    if (!(sensorA ^ sensorB))
        return false;
    if (sensorA)
    {
        robot = reinterpret_cast<Robot*>(reinterpret_cast<BodyData*>(fA->GetBody()->GetUserData().pointer)->entity);
        sensor_fixture = fA;
        entity = reinterpret_cast<BodyData*>(fB->GetBody()->GetUserData().pointer)->entity;
        detected_fixture = fB;
    }
    else
    {
        robot = reinterpret_cast<Robot*>(reinterpret_cast<BodyData*>(fB->GetBody()->GetUserData().pointer)->entity);
        sensor_fixture = fB;
        entity = reinterpret_cast<BodyData*>(fA->GetBody()->GetUserData().pointer)->entity;
        detected_fixture = fA;
    }
    return true;
}

void ContactListener::BeginContact(b2Contact* c)
{
    Robot* r;
    b2Fixture *sf;
    uintptr_t e;
    b2Fixture *df;
    if (get_sensed_entity(c, r, sf, e, df))
        r->aquired_entity(e, sf, df);
}
void ContactListener::EndContact(b2Contact* c)
{
    Robot *r;
    b2Fixture *sf;
    uintptr_t e;
    b2Fixture *df;
    if (get_sensed_entity(c, r, sf, e, df))
        r->lost_entity(e, sf, df);
}


bool is_zero(float x)
{
    return fabs(x) < 1e-7;
}
bool check_intersection(b2Vec2 &p0, b2Vec2 &p1, b2Vec2 &q0, b2Vec2 &q1, b2Vec2 &i)
{
    // line segment - line segment intersection
    // https://www.codeproject.com/Tips/862988/Find-the-Intersection-Point-of-Two-Line-Segments
    auto r = p1 - p0;
    auto s = q1 - q0;
    auto rxs = b2Cross(r, s);
    auto qpxr = b2Cross(q0 - p0, r);
    if (is_zero(rxs) && is_zero(qpxr))
        // Collinear
        return false;
    if (is_zero(rxs) && ! is_zero(qpxr))
        // Parallel
        return false;
    auto t = b2Cross(q0 - p0, s) / rxs;
    auto u = b2Cross(q0 - p0, r) / rxs;
    if (!is_zero(rxs) && (0 <= t && t <= 1) && (0 <= u && u <= 1))
    {
        // We can calculate the intersection point using either t or u.
        i = p0 + t * r;
        return true;
    }
    return false;
}

bool check_intersection(b2Vec2 &p0, b2Vec2 &p1, b2Vec2 &q, float r, b2Vec2 &i)
{
    // line segment - circle intersection
    // https://stackoverflow.com/questions/1073336/circle-line-segment-collision-detection-algorithm
    auto d = p1 - p0;
    auto f = p0 - q;
    float a = b2Dot(d, d);
    float b = 2 * b2Dot(f, d);
    float c = b2Dot(f, f) - r * r;
    
    float discriminant = b * b - 4 * a * c;
    if (discriminant < 0)
    {
        // no intersection
        return false;
    }
    else
    {
        // ray didn't totally miss sphere,
        // so there is a solution to
        // the equation.
        discriminant = sqrt(discriminant);
        
        // either solution may be on or off the ray so need to test both
        // t1 is always the smaller value, because BOTH discriminant and
        // a are nonnegative.
        float t1 = (-b - discriminant)/(2*a);
        float t2 = (-b + discriminant)/(2*a);
        
        // 3x HIT cases:
        //          -o->             --|-->  |            |  --|->
        // Impale(t1 hit,t2 hit), Poke(t1 hit,t2>1), ExitWound(t1<0, t2 hit),
        
        // 3x MISS cases:
        //       ->  o                     o ->              | -> |
        // FallShort (t1>1,t2>1), Past (t1<0,t2<0), CompletelyInside(t1<0, t2>1)
        if(t1 >= 0 && t1 <= 1)
        {
            // t1 is the intersection, and it's closer than t2
            // (since t1 uses -b - discriminant)
            // Impale, Poke
            i = p0 + t1 * d;
            return true ;
        }
        // here t1 didn't intersect so we are either started
        // inside the sphere or completely past it
        if( t2 >= 0 && t2 <= 1 )
        {
            // ExitWound
            i = p0 + t2 * d;
            return true ;
        }
        // no intn: FallShort, Past, CompletelyInside
        return false ;
    }
}

void Environment::process_visible(std::unique_ptr<Robot> &robot)
{
    // Process all the entities that are visible to the sensors of a robot
    b2Transform transform = robot->getBody()->GetTransform();
    const float big_dist = 1000;
    for (auto &e : robot->visible_entities)
    {
        // Just handle ltof for now
        auto sensor_fdata = reinterpret_cast<FixtureData*>(e.sensor_fixture->GetUserData().pointer);
        auto detected_fdata = reinterpret_cast<FixtureData*>(e.detected_fixture->GetUserData().pointer);
        SimObject *detected_object = reinterpret_cast<SimObject*>(e.entity);
        if (sensor_fdata->type == BODYLTOF)
        {
            float min_dist = big_dist;
            b2Vec2 intersect;

            auto sshape = reinterpret_cast<b2PolygonShape*>(e.sensor_fixture->GetShape());
            b2Vec2 p[sshape->m_count];
            for(int i = 0; i < sshape->m_count; i++)
            {
                p[i] = b2Mul(transform, sshape->m_vertices[i]);
                //printf("Sensor % f % f\n", p[i].x, p[i].y);
            }
            
            auto shape_type = e.detected_fixture->GetShape()->GetType();
            //printf("%d->%d:%d[%d]%d\n", sensor_fdata->type, detected_object->getBodyData().type, detected_fdata->type, sensor_fdata->fixture_idx, shape_type);
            
            // We know that some part of the detected fixture is within the sensor fixture.
            // Work out the points of intersection between the fixtures, keeping the closest
            if (shape_type == b2Shape::e_polygon)
            {
                // Used for loads
                auto shape = reinterpret_cast<b2PolygonShape*>(e.detected_fixture->GetShape());
                //printf(" %d ", shape->m_count);
            }
            else if (shape_type == b2Shape::e_circle)
            {
                // Used for robots
                auto radius = reinterpret_cast<b2CircleShape*>(e.detected_fixture->GetShape())->m_radius;
                // Position of circle is always 0,0 resp robot body, so get the position from the body
                auto rp = e.detected_fixture->GetBody()->GetPosition();
                //printf("Detected circle % f % f\n", rp.x, rp.y);
                b2Vec2 local_intersect;
                for(int j = 0; j < sshape->m_count; j++)
                {
                    auto hit = check_intersection(p[j], p[(j + 1) % sshape->m_count], rp, radius, local_intersect);
                    if (hit)
                    {
                        float dist = b2Distance(transform.p, local_intersect);
                        if (dist < min_dist)
                        {
                            min_dist = dist;
                            intersect = local_intersect;
                        }
//                        printf("intersect (% f % f) (% f % f) (% f % f) % f (% f % f) %f\n",
//                               p[j].x, p[j].y, p[(j+1)%sshape->m_count].x, p[(j+1)%sshape->m_count].y, rp.x, rp.y, radius,
//                        local_intersect.x, local_intersect.y, min_dist);
                        sensor_contacts.push_back(local_intersect);
                        //g_debug_draw.DrawCircle(intersect, 0.1, green);
                    }
                }
            }
            else if (shape_type == b2Shape::e_chain)
            {
                // Used for arena walls
                auto shape = reinterpret_cast<b2ChainShape*>(e.detected_fixture->GetShape());
                for(int i = 0; i < shape->m_count - 1; i++)
                {
                    //printf("Detected chain % f % f\n", shape->m_vertices[i].x, shape->m_vertices[i].y);
                    b2Vec2 local_intersect;
                    auto &q = shape->m_vertices;
                    for(int j = 0; j < sshape->m_count; j++)
                    {
                        auto hit = check_intersection(p[j], p[(j + 1) % sshape->m_count], q[i], q[i + 1], local_intersect);
                        if (hit)
                        {
                            float dist = b2Distance(transform.p, local_intersect);
                            if (dist < min_dist)
                            {
                                min_dist = dist;
                                intersect = local_intersect;
                            }
//                            printf("intersect (% f % f) (% f % f) (% f % f) (% f % f) (% f % f) %f\n",
//                                   p[j].x, p[j].y, p[j+1].x, p[j+1].y, q[i].x, q[i].y, q[i+1].x, q[i+1].y, local_intersect.x, local_intersect.y, min_dist);
                            sensor_contacts.push_back(local_intersect);
							//g_debug_draw.DrawCircle(local_intersect, 0.1, green);
                        }
                        
                    }
                }
            }
            if (min_dist < big_dist)
            {
                // We hit with an LTOF sensor, assign to correct prox entry
                // FIXME define magic numbers properly
                int idx = sensor_fdata->fixture_idx - 5;
                robot->scene.prox_ltof[idx] = 1 - ((min_dist - g_rc.radius) / g_rc.ltof_max);
            }
        }
    }
    //printf("\n");
}


// Sense the environment and update each robot's perceived scene
void Environment::sense()
{
    sensor_contacts.clear();
	for (auto& robot : p_robots)
	{
		b2Body* rb = robot->getBody();
		unsigned short r_id = robot->getBodyData().id;

		// Reset scene
		robot->scene = Scene();

        if (!new_prox)
        {
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
        }
        else
        {
            process_visible(robot);
            for(int i = 0; i < 16; i++) robot->scene.prox[i] = robot->scene.prox_ltof[i];
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
    
//    for (int i = 0; i < p_robots.size(); i++)
//    {
//        for(int j = 0; j < 16; j++)
//            printf("%6.3f %6.3f\n", p_robots[i]->scene.prox[j], p_robots[i]->scene.prox_ltof[j]);
//    }
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
	for (auto& robot : p_robots)
	{
		float p_goal = robot->message.p_vote;
		if (p_goal == 0.0f)
		{
			// Reset wait counter
			robot->wait_count = 0;

			// Wheels
			VectorPolar v_goal = robot->message.v_vote;

			// Transform to local vector
			v_goal.rotate(robot->scene.heading - robot->scene.orientation);
			robot->setVelocity(v_goal);
		}
		else
		{
			// Wait 1s for the wheels to stop before actuating the platform
			robot->setVelocity(VectorPolar(0.0f, 0.0f));

			if (robot->wait_count++ == std::round(1.0f / g_rc.t_control))
			{
				robot->wait_count = 0;
			}
			else
			{
				continue;
			}

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

						// If robot is at nest, reset load
						if (robot->scene.rb_nest.r > 0.0f &&
						    robot->scene.rb_nest.r < g_ac.nest_r)
						{
							load->reset = true;
						}

						break;
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
}

void Environment::renderSetup()
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

    
    glfwSwapInterval(1);
    
	g_camera.m_width = 800; // hard-coded for now
	g_camera.m_height = 950; // hard-coded for now
    g_camera.m_zoom = zoom = 1.05;
    g_camera.m_center.Set(0.0f, 0.5f);
    centre = &g_camera.m_center;
	p_window = glfwCreateWindow(g_camera.m_width,
	                            g_camera.m_height, "OpenGL", NULL, NULL);
	if (p_window == NULL)
	{
		glfwTerminate();
		throw std::runtime_error("Failed to open window");
	}
	glfwMakeContextCurrent(p_window);

    // Set up some callbacks for mouse
    glfwSetWindowUserPointer(p_window, this);
    glfwSetCursorPosCallback(p_window, cursor_pos_callback);
    glfwSetMouseButtonCallback(p_window, mouse_button_callback);
    glfwSetScrollCallback(p_window, scroll_callback);
    
	// Initialise GLEW
	glewExperimental = GL_TRUE;
	glewInit();

    // Initialise dear imgui
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGui::StyleColorsClassic();

    ImGui_ImplGlfw_InitForOpenGL(p_window, true);
    ImGui_ImplOpenGL3_Init("#version 150");
    
	// Initialise debug draw
	g_debug_draw.Create();
	p_world->SetDebugDraw(&g_debug_draw);


	// Arena vertices
	a_v[0].Set(-g_ac.width / 2.0f,  g_ac.length / 2.0f);
	a_v[1].Set( g_ac.width / 2.0f,  g_ac.length / 2.0f);
	a_v[2].Set( g_ac.width / 2.0f, -g_ac.length / 2.0f);
	a_v[3].Set(-g_ac.width / 2.0f, -g_ac.length / 2.0f);
}


void Environment::renderPredraw()
{
	// Start of frame setup
		// Rendering loop
    glfwPollEvents();
    
    // Dear imgui
    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();
    
    //ImGui::ShowMetricsWindow();
    auto bsize = ImVec2(50, 20);
    ImGui::SetNextWindowPos(ImVec2(0, 0));
    ImGui::SetNextWindowSize(ImVec2(g_camera.m_width, 150));
    ImGui::Begin("Control", NULL, ImGuiWindowFlags_NoTitleBar);
    //ImGui::SetCursorScreenPos(ImVec2(0, 0));
    ImGui::Checkbox("prox", &g_sp.render_prox);
    ImGui::SameLine();
    ImGui::Checkbox("cams", &g_sp.render_cams);
    ImGui::SameLine();
    ImGui::Checkbox("ltof", &g_sp.render_ltof);
    ImGui::Checkbox("ctact", &g_sp.render_contacts);
    if (g_sp.paused)
        g_sp.paused ^= ImGui::Button("Run", bsize);
    else
        g_sp.paused ^= ImGui::Button("Pause", bsize);
    ImGui::SameLine();
    g_sp.step = ImGui::Button("Step", bsize);
    auto wm = g_camera.ConvertScreenToWorld(mouse);
    ImGui::Text("%10.8f %f %f", g_sp.model_time, wm.x, wm.y);
    ImGui::Text("%f %f", zoom, scroll.y);
    ImGui::SetCursorPos(ImVec2(400, 0));
    ImGui::PlotLines("Model time", g_sp.mt_array.data(), g_sp.mt_array.size(), 0, "Time (ms)", 0.0, 0.005, ImVec2(400, 100));
    //ImGui::SetCursorPos(ImVec2(300, 0));
	// Display robot 0 BB
	auto &r = p_robots[0];
	r->controller.guiBB(200, 0, r->message);
    ImGui::End();
    
    g_camera.m_zoom = zoom;
    //centre = g_camera.m_center;
    if (drag)
    {
        auto sd = g_camera.ConvertScreenToWorld(start_drag);
        g_camera.m_center = drag_origin + sd - wm;
    }
    glfwGetWindowSize(p_window, &g_camera.m_width, &g_camera.m_height);

    int buffer_width, buffer_height;
    glfwGetFramebufferSize(p_window, &buffer_width, &buffer_height);
    glViewport(0, 0, buffer_width, buffer_height);

    glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

}
    
void Environment::renderStep()
{

    // Draw simulation features
    drawArena();
    drawLoads();
    drawRobots();
    drawContacts();
    p_world->DebugDraw();
    g_debug_draw.Flush();
    
    // Imgui
    ImGui::Render();
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
    

    glfwSwapBuffers(p_window);
}

void Environment::renderCleanup()
{
	// Clean up
    if (render)
    {
        g_debug_draw.Destroy();
        glfwTerminate();
        render = false;
    }
}


void Environment::drawArena()
{
    // Draw arena
    g_debug_draw.DrawPolygon(a_v, 4, b2Color(0.0f, 0.0f, 0.0f));

    b2Vec2 p1 = b2Vec2((g_ac.width / 2.0f) - g_ac.nest_r,
                         g_ac.length / 2.0f);
    b2Vec2 p2 = b2Vec2((g_ac.width / 2.0f) - g_ac.nest_r,
                        -g_ac.length / 2.0f);
    g_debug_draw.DrawSegment(p1, p2, green);
}

void Environment::drawContacts()
{
    if (g_sp.render_contacts)
    {
        for(auto &s :  sensor_contacts)
        {
            g_debug_draw.DrawCircle(s, 0.005, green);
        }
    }
}

void Environment::drawLoads()
{
    
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
}

void Environment::drawRobots()
{
    
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
                auto &r = circle->m_radius;
                g_debug_draw.DrawCircle(centre, r, colour);
                b2Vec2 front = centre + r * b2Vec2(transform.q.c, transform.q.s);
                g_debug_draw.DrawSegment(centre, front, green);
                break;
            }
            if ((type == BODYCAM) && g_sp.render_cams)
            {
                b2PolygonShape* poly = reinterpret_cast<b2PolygonShape*>(f->GetShape());
                int n_vertices = poly->m_count;
                b2Vec2 vertices[n_vertices];
                for (auto i = 0; i < n_vertices; ++i)
                {
                    vertices[i] = b2Mul(transform, poly->m_vertices[i]);
                }
                g_debug_draw.DrawPolygon(vertices, n_vertices, blue);
                
            }
            if ((type == BODYLTOF) && g_sp.render_ltof)
            {
                b2PolygonShape* poly = reinterpret_cast<b2PolygonShape*>(f->GetShape());
                int n_vertices = poly->m_count;
                b2Vec2 vertices[n_vertices];
                for (auto i = 0; i < n_vertices; ++i)
                {
                    vertices[i] = b2Mul(transform, poly->m_vertices[i]);
                }
                g_debug_draw.DrawPolygon(vertices, n_vertices, blue);
                
            }
        }


        auto &prox = robot->scene.prox;
        int idx = 0;
        for (auto  &angle : g_rc.prox_angles)
        {
            b2Vec2 p1 = robot->getLaserStart(angle);
            b2Vec2 p2 = robot->getLaserEnd(angle);
            auto frac = 1.0f - prox[idx++];
            //if (frac < 1.0f) printf("%f\n",frac);
            b2Vec2 pr = frac * (p2 - p1) + p1;
            g_debug_draw.DrawSegment(p1, p2, red);
        }
        
//        for (auto &e : robot->visible_entities)
//        {
//            SimObject *s = reinterpret_cast<SimObject*>(e);
//            for(auto &f : s->p_fixture_data)
//                printf("%d-%d ", s->getBodyData().type, f.type);
//        }
//        printf("\n");
    }
}
