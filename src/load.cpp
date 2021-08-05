#include <cmath>
#include <string>
#include <stdexcept>
#include "load.h"

Load::Load(b2World* world, const b2Vec2& position, unsigned short porters) :
	t_life(0), t_cover(0), lifted(false),
	p_start_position(position), p_porters(porters)
{
	// Set body and fixture data
	p_body_data.type = LOAD;
	p_body_data.id = p_next_id;
    p_body_data.entity = reinterpret_cast<uintptr_t>(this);

	p_fixture_data.resize(porters + 1);
	p_fixture_data.at(0).type = BODY;
	p_fixture_data.at(0).body_id = p_next_id;

	for (std::size_t i = 1; i < p_fixture_data.size(); ++i)
	{
		p_fixture_data.at(i).type = MARKER;
		p_fixture_data.at(i).body_id = p_next_id;
	}

	// Check for correct input
	if (porters < 1 || porters > b2_maxPolygonVertices)
	{
		std::string e_msg = "Number of porters must be between 1 and ";
		e_msg += std::to_string(b2_maxPolygonVertices);
		e_msg += "\n";
		throw std::domain_error(e_msg);
	}

	// Issue warning if porters won't be fully connected
	if (porters > 4)
	{
		std::cout << "WARNING: porter network is not fully connected for n > 4"
		          << std::endl;
	}

	float r_loose = g_rc.radius + 0.05f;

	// Create load body
	b2BodyDef load_def;
	load_def.type = b2_dynamicBody;
	load_def.position.Set(position.x, position.y);
	load_def.allowSleep = true;
	load_def.awake = true;
	load_def.userData.pointer = reinterpret_cast<uintptr_t>(&p_body_data);

	p_body = world->CreateBody(&load_def);

	// Lifting points
	b2Vec2 lifting_points[porters];
	if (porters == 1)
	{
		lifting_points[0].Set(0.0f, 0.0f);
	}
	else
	{
		float poly_angle = 2.0f * M_PI / porters;
		float poly_radius = g_lc.separation / (2.0f * std::sin(poly_angle/2.0f));
		for (unsigned short i = 0; i < porters; i++)
		{
			lifting_points[i].Set(poly_radius * std::cos(i * poly_angle),
			                      poly_radius * std::sin(i * poly_angle));
		}
	}

	// Load corners
	std::vector<b2Vec2> load_corners;
	if (porters == 1)
	{
		load_corners.push_back(b2Vec2(  r_loose,  r_loose));
		load_corners.push_back(b2Vec2(  r_loose, -r_loose));
		load_corners.push_back(b2Vec2( -r_loose,  r_loose));
		load_corners.push_back(b2Vec2( -r_loose, -r_loose));
	}
	else if (porters == 2)
	{
		load_corners.push_back(b2Vec2( r_loose + g_lc.separation / 2.0f, r_loose));
		load_corners.push_back(b2Vec2( r_loose + g_lc.separation / 2.0f,-r_loose));
		load_corners.push_back(b2Vec2(-r_loose - g_lc.separation / 2.0f, r_loose));
		load_corners.push_back(b2Vec2(-r_loose - g_lc.separation / 2.0f,-r_loose));
	}
	else
	{
		float offset = r_loose / std::sin(M_PI_2 * (1.0f - 2.0f / porters));
		for (unsigned short i = 0; i < porters; i++)
		{
			b2Vec2 v_offset = lifting_points[i];
			v_offset.Normalize();
			v_offset *= offset;
			load_corners.push_back(b2Vec2(lifting_points[i] + v_offset));
		}
	}		

	// Add load
	b2PolygonShape load_shape;
	load_shape.Set(load_corners.data(), load_corners.size());

	b2FixtureDef load_fixture_def;
	load_fixture_def.shape = &load_shape;
	load_fixture_def.density = g_lc.density;
	// Load collides with the arena, other loads and platform cameras
	load_fixture_def.filter.categoryBits = CT_LOAD;
	load_fixture_def.filter.maskBits = CT_ARENA | CT_LOAD | CT_PCAM |CT_LTOF;
	load_fixture_def.userData.pointer = reinterpret_cast<uintptr_t>(&p_fixture_data.at(0));

	p_body->CreateFixture(&load_fixture_def);

	// Add lifting point markers
	unsigned short marker_n = 1;
	for (const auto& lp : lifting_points)
	{
		b2CircleShape marker_shape;
		marker_shape.m_p = lp;
		marker_shape.m_radius = 0.01;

		b2FixtureDef marker_fixture_def;
		marker_fixture_def.shape = &marker_shape;
		// Lifting point collides with other lifting points and with platform
		// cameras
		marker_fixture_def.filter.categoryBits = CT_LOAD_MARKER;
		marker_fixture_def.filter.maskBits = CT_LOAD_MARKER | CT_PCAM;
		marker_fixture_def.userData.pointer = reinterpret_cast<uintptr_t>(&p_fixture_data.at(marker_n++));

		p_body->CreateFixture(&marker_fixture_def);
	}

/*
	b2Vec2 marker_corners[4];
	unsigned short marker_n = 1;
	for (const auto& lp : lifting_points)
	{
		marker_corners[0].Set( g_lc.marker_w / 2.0f + lp.x,
		                       g_lc.marker_w / 2.0f + lp.y);
		marker_corners[1].Set( g_lc.marker_w / 2.0f + lp.x,
		                      -g_lc.marker_w / 2.0f + lp.y);
		marker_corners[2].Set(-g_lc.marker_w / 2.0f + lp.x,
                              -g_lc.marker_w / 2.0f + lp.y);
		marker_corners[3].Set(-g_lc.marker_w / 2.0f + lp.x,
                               g_lc.marker_w / 2.0f + lp.y);
	
		b2PolygonShape marker_shape;
		marker_shape.Set(marker_corners, 4);
	
		b2FixtureDef marker_fixture_def;
		marker_fixture_def.shape = &marker_shape;
		// Lifting point collides with other lifting points and with platform
		// cameras
		marker_fixture_def.filter.categoryBits = 0x0010;
		marker_fixture_def.filter.maskBits = 0x0014;
		marker_fixture_def.userData.pointer = reinterpret_cast<uintptr_t>(&p_fixture_data.at(marker_n++));

		p_body->CreateFixture(&marker_fixture_def);
	}
*/

	// Update ID;
	p_next_id++;
}

Load::~Load() {}

void Load::destroy(b2World* world)
{
	world->DestroyBody(p_body);
	p_body = nullptr;
}

unsigned short Load::getPorters()
{
	return p_porters;
}

b2Vec2 Load::getStartPosition()
{
	return p_start_position;
}

short Load::p_next_id = 1;
