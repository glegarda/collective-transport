#ifndef COMMON_H
#define COMMON_H

// Contains data to which all other classes should have access
// Allows modification of parameters (e.g. sensors)

#include <vector>
#include "utils.h"

enum BodyType
{
	BUNDEF,
	ARENA,
	ROBOT,
	LOAD
};

struct BodyData
{
	BodyData() : type(BUNDEF), id(0) {}
	~BodyData() {}

	enum BodyType type;
	unsigned short id;
};

enum FixtureType
{
	FUNDEF,
	BODY,
	BODYCAM,
	PLATCAM,
	MARKER
};

struct FixtureData
{
	FixtureData() : type(FUNDEF), body_id(0) {}
	~FixtureData() {}

	enum FixtureType type;
	unsigned short body_id;
};

struct RobotConstants
{
	RobotConstants() :
		t_control(0.1),
		radius(0.125),
		height(0.115),
		d_wheel(0.12),
		v_max(0.2),
		density(40.7437),
		friction(0.15),
		restitution(0.1),
		prox_angles{-2.7489, -2.3562, -1.9635, -1.5708, -1.1781, -0.7854,
		            -0.3927,  0.0000,  0.3927,  0.7854,  1.1781,  1.5708,
		             1.9635,  2.3562,  2.7489, 3.1416},
		prox_max(0.15),
		cam_angles{0.1745, 2.1526, 4.1306, 6.1087},
		cam_fov(2.0944),
		cam_max(1.0),
		cam_min(0.0),
		pcam_fov_h(2.0944),
		pcam_fov_v(1.5708),
		comms_range(1.0) {}
	~RobotConstants() {}

	// Physical properties
	float t_control; // seconds
	float radius;
	float height;
	float d_wheel;
	float v_max;
	float density; // of a disc
	float friction;
	float uw_max;
	float ut_max;
	float restitution;
	// Laser TOF distance sensors
	float prox_angles[16];
	float prox_max;
	// Body cameras
	float cam_angles[4];
	float cam_fov;
	float cam_max;
	float cam_min;
	// Platform camera
	float pcam_fov_h;
	float pcam_fov_v;
	// Communications
	float comms_range;
};

struct LoadConstants
{
	LoadConstants() :
		height(0.185),
		density(9.0),
		separation(0.65),
		marker_w(0.1) {}
	~LoadConstants() {}

	float height;
	float density;
	float separation;
	float marker_w;
};

struct ArenaConstants
{
	ArenaConstants() :
		width(5.0f),
		length(5.0f),
		nest_r(1.0f),
		marker_w(0.1f) {}
	~ArenaConstants() {}

	float width;
	float length;
	float nest_r;
	float marker_w;
};

extern RobotConstants g_rc;
extern LoadConstants g_lc;
extern ArenaConstants g_ac;

struct Message
{
	Message() :
		id_messenger(0),
		id_load(0),
		id_group(0),
		v_vote(VectorPolar(0.0, 0.0)),
		p_vote(0.0) {}
	~Message() {}

	short id_messenger;       // messenger identifier
	short id_load;            // load identifier
	short id_group;           // group identifier
	VectorPolar v_vote;       // proposed velocity
	float p_vote;             // proposed platform velocity
};

struct Neighbour
{
	Neighbour() :
		id(0),
		rb(VectorPolar(0.0, 0.0)) {}
	~Neighbour() {}

	unsigned short id;
	VectorPolar rb;
};

struct Scene
{
	Scene() :
		prox{0.0},
		orientation(0.0),
		heading(0.0),
		n(0),
		nest(false),
		rb_nest(VectorPolar(0.0, 0.0)),
		load(false),
		rb_lift(VectorPolar(0.0, 0.0)),
		id_load(0),
		porters(0) {}
	~Scene() {}

	float prox[16];                         // proximity sensors
	float orientation;                      // orientation
	float heading;                          // heading
	unsigned short n;                       // number of neighbours
	std::vector<Neighbour> neighbours;      // id, range and bearing of neighbour 
	bool nest;                              // nest detection
	VectorPolar rb_nest;                    // nest location
	bool load;                              // load detection
	VectorPolar rb_lift;                    // lifting point location
	short id_load;                          // load identifier
	unsigned short porters;                 // porters required
};

#endif
