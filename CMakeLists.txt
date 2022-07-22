cmake_minimum_required(VERSION 3.8)

project(ECT)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED True)

find_package(box2d REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
# For older releases of BehaviorTree.CPP, use the following line instead
#find_package(BehaviorTreeV3 REQUIRED)
find_package(OpenGL REQUIRED)
find_package(glfw3 3.3 REQUIRED)
find_package(GLEW REQUIRED)
find_package(Threads REQUIRED)


add_subdirectory(src)
add_subdirectory(tests)
