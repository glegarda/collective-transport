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
    uintptr_t entity;
};

enum FixtureType
{
	FUNDEF,
	BODY,
	BODYCAM,
    BODYLTOF,
	PLATCAM,
	MARKER
};

enum CategoryType
{
    CT_ROBOT            = 0x0001,
    CT_CAM              = 0x0002,
    CT_PCAM             = 0x0004,
    CT_LOAD             = 0x0008,
    CT_LOAD_MARKER      = 0x0010,
    CT_ARENA            = 0x0020,
    CT_ARENA_MARKER     = 0x0040,
    CT_LTOF             = 0x0080,
};

struct FixtureData
{
	FixtureData() : type(FUNDEF), body_id(0), fixture_idx(0) {}
	~FixtureData() {}

	enum FixtureType type;
	unsigned short body_id;
    // Use this to identify ltof sensor (and possibly cameras)
    // 0    - robot shape
    // 1-4  - cameras
    // 5-20 - ltof
    // 21   - platform camera
    int fixture_idx;
    uintptr_t entity;
};


struct SensorData
{
    SensorData(b2Fixture *sf, b2Fixture *df, uintptr_t e)
    : sensor_fixture(sf), detected_fixture(df), entity(e) {}
    b2Fixture   *sensor_fixture;
    b2Fixture   *detected_fixture;
    uintptr_t   entity;
    // Define equality so erase works
    bool operator==(const SensorData &rhs)
    {
        return (this->sensor_fixture == rhs.sensor_fixture) &&
            (this->detected_fixture == rhs.detected_fixture) &&
            (this->entity == rhs.entity);
    }
};

struct SimParams
{
    // Container for global parameters controlling aspects of GUI etc.
    // Mostly initialised by command line parameters, and altered by GUI
    // widgets
    
    // Rendering
    bool render_prox = false;
    bool render_cams = false;
	bool render_ltof = false;
	bool render_contacts = false;
    bool single_step = false;
    bool paused = true;
    bool step = false;
    
    // mouse control
    float scale = 0.1;
    
    // Stats
    double model_time = 0.0;
    std::vector<float> mt_array;
    SimParams() : mt_array(500, 0.0) {}
    void push_mt(float x)
    {
        for(int i = 0; i < mt_array.size() - 1; i++)
            mt_array[i] = mt_array[i + 1];
        mt_array[mt_array.size() - 1] = x;
    }
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
        // SJ change to give same indices as hardware
		prox_angles{ 0.0000,  0.3927,  0.7854,  1.1781,  1.5708,  1.9635,  2.3562,  2.7489,
                     3.1416, -2.7489, -2.3562, -1.9635, -1.5708, -1.1781, -0.7854, -0.3927},
		prox_max(0.15),
		cam_angles{0.1745, 2.1526, 4.1306, 6.1087},
		cam_fov(2.0944),
		cam_max(1.0),
		cam_min(0.0),
		pcam_fov_h(2.0944),
		pcam_fov_v(1.5708),
		comms_range(0.2),
        ltof_fixture(true),
        ltof_fov(0.40),
        ltof_max(0.15)
    {}
	~RobotConstants() {}

	// Physical properties
	float t_control; // seconds
	float radius;
	float height;
	float d_wheel;
	float v_max;
	float density; // of a disc
	float friction;
	float restitution;
	// Laser TOF distance sensors
	std::vector<float> prox_angles;
	float prox_max;
	// Body cameras
	std::vector<float> cam_angles;
	float cam_fov;
	float cam_max;
	float cam_min;
	// Platform camera
	float pcam_fov_h;
	float pcam_fov_v;
	// Communications
	float comms_range;
    bool ltof_fixture;
    float ltof_fov;
    float ltof_max;
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
        prox_ltof{0.0},
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
    float prox_ltof[16];                         // proximity sensors
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
