set(ECT_SOURCE_FILES
	arena.cpp
	controller.cpp
	draw.cpp
	environment.cpp
	load.cpp
	pid.cpp
	callbacks.cpp
	robot.cpp
	sim_object.cpp
	utils.cpp)

set(ECT_HEADER_FILES
	../include/arena.h
	../include/bt_custom.h
	../include/common.h
	../include/controller.h
	../include/draw.h
	../include/environment.h
	../include/load.h
	../include/openGA.hpp
	../include/pid.h
	../include/callbacks.h
	../include/robot.h
	../include/sim_object.h
	../include/utils.h)

add_library(ect ${ECT_SOURCE_FILES} ${ECT_HEADER_FILES})

target_include_directories(ect PUBLIC ../include)

target_link_libraries(ect PUBLIC
	BT::behaviortree_cpp_v3
	box2d::box2d
	glfw
	GLEW::GLEW
	GL
	Threads::Threads)

add_executable(gp gp.cpp)

target_link_libraries(gp PRIVATE ect)
