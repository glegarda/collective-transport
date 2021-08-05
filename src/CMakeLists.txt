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


set(IMGUI_FILES
	imgui.cpp
	imgui_draw.cpp
	imgui_tables.cpp
	imgui_widgets.cpp
	imgui_impl_glfw.cpp
	imgui_impl_opengl3.cpp
	imgui_demo.cpp
	../include/imgui.h
	../include/imgui_internal.h
	../include/imconfig.h
	../include/imstb_rectpack.h
	../include/imstb_textedit.h
	../include/imstb_truetype.h
	../include/imgui_impl_glfw.h
	../include/imgui_impl_opengl3.h
)

add_library(ect 
	${ECT_SOURCE_FILES} 
	${ECT_HEADER_FILES}
	${IMGUI_FILES})

target_include_directories(ect PUBLIC ../include)

target_link_libraries(ect PUBLIC
	BT::behaviortree_cpp_v3
	box2d::box2d
	glfw
	GLEW::GLEW
	${OPENGL_LIBRARIES}
	Threads::Threads)

add_executable(gp gp.cpp)

target_link_libraries(gp PRIVATE ect)
