#include "arena.h"

Arena::Arena() {}

Arena::Arena(b2World* world)
{
	// Set body and fixture data
	p_body_data.type = ARENA;
	p_body_data.id = 0;
    p_body_data.entity = reinterpret_cast<uintptr_t>(this);

	p_fixture_data.resize(6);
	p_fixture_data.at(0).type = BODY;
	p_fixture_data.at(0).body_id = 0;

	for (std::size_t i = 1; i < p_fixture_data.size(); ++i)
	{
		p_fixture_data.at(i).type = MARKER;
		p_fixture_data.at(i).body_id = 0;
	}

	// Create rectangular arena
	b2BodyDef arena_def;
	arena_def.position.Set(0.0f, 0.0f);
	arena_def.allowSleep = true;
	arena_def.awake = true;
	arena_def.userData.pointer = reinterpret_cast<uintptr_t>(&p_body_data);

	p_body = world->CreateBody(&arena_def);

	b2Vec2 corners[4];
	corners[0].Set(-g_ac.width / 2.0f,  g_ac.length / 2.0f);
	corners[1].Set( g_ac.width / 2.0f,  g_ac.length / 2.0f);
	corners[2].Set( g_ac.width / 2.0f, -g_ac.length / 2.0f);
	corners[3].Set(-g_ac.width / 2.0f, -g_ac.length / 2.0f);

	b2ChainShape arena_shape;
	arena_shape.CreateLoop(corners, 4);

	b2FixtureDef arena_fixture_def;
	arena_fixture_def.shape = &arena_shape;
	arena_fixture_def.density = 2000.0;
	arena_fixture_def.friction = g_rc.friction;
	// Arena collides with robots, loads and other arenas
	arena_fixture_def.filter.categoryBits = CT_ARENA;
	arena_fixture_def.filter.maskBits = CT_ARENA | CT_LOAD | CT_ROBOT | CT_LTOF;
	arena_fixture_def.userData.pointer = reinterpret_cast<uintptr_t>(&p_fixture_data.at(0));

	p_body->CreateFixture(&arena_fixture_def);

	// Add nest markers
	float fraction[5] = {4.0f/5.0f, 2.0f/5.0f, 0.0f, -2.0f/5.0f, -4.0f/5.0f};
	unsigned short marker_n = 1;
	for (const auto& f : fraction)
	{
		b2EdgeShape marker_shape;
		marker_shape.SetTwoSided(b2Vec2(g_ac.width / 2.0f,
		                                g_ac.width*f/2.0f + g_ac.marker_w/2.0f),
		                         b2Vec2(g_ac.width / 2.0f,
		                                g_ac.width*f/2.0f - g_ac.marker_w/2.0f));

		b2FixtureDef marker_fixture_def;
		marker_fixture_def.shape = &marker_shape;
		// Nest marker collides with body cameras and with other nest markers
		marker_fixture_def.filter.categoryBits = CT_ARENA_MARKER;
		marker_fixture_def.filter.maskBits = CT_ARENA_MARKER | CT_CAM;
		marker_fixture_def.userData.pointer = reinterpret_cast<uintptr_t>(&p_fixture_data.at(marker_n++));

		p_body->CreateFixture(&marker_fixture_def);
	}
}

Arena::~Arena() {}
