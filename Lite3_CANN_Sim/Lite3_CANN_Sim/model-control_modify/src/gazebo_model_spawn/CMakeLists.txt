cmake_minimum_required(VERSION 2.8.3)
project(gazebo_model_spawn)

find_package(catkin REQUIRED COMPONENTS
    controller_manager
    genmsg
    joint_state_controller
    robot_state_publisher
    roscpp
    gazebo_ros
    std_msgs
    tf
    geometry_msgs
)

find_package(gazebo REQUIRED)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES gazebo_model_spawn
    CATKIN_DEPENDS roscpp
)

include_directories(
    include
    ${Boost_INCLUDE_DIR}
    ${catkin_INCLUDE_DIRS}
    ${GAZEBO_INCLUDE_DIRS}
)

file(GLOB_RECURSE sources "src/*.cpp")
add_library(gazebo_model_spawn ${sources})

link_directories(${GAZEBO_LIBRARY_DIRS})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")



link_libraries(${catkin_LIBRARIES} gazebo_model_spawn)
add_executable(spawn_model node/spawn_model.cpp)





