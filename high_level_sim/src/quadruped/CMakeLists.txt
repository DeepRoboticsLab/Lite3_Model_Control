cmake_minimum_required(VERSION 3.10)
project(quadruped LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_BUILD_TYPE "RELEASE") #"RELEASE", "Debug" -DEIGEN_NO_DEBUG
set(CMAKE_CXX_FLAGS "-O3 -DEIGEN_NO_DEBUG -fopenmp ${CMAKE_CXX_FLAGS}")
#set(CMAKE_CXX_FLAGS "-O3 -g -Wall ${CMAKE_CXX_FLAGS}")
message("CMAKE_CXX_FLAGS = " ${CMAKE_CXX_FLAGS})
ADD_COMPILE_OPTIONS(-D_cplusplus)
ADD_COMPILE_OPTIONS(-D_useros)
# ADD_COMPILE_OPTIONS(-DQPOASES_AVOID_LA_NAMING_CONFLICTS)

option(USE_BLAS                            "USE MKL BLAS"                   OFF)
option(USE_SIM                            "USE Simulation"                  ON)

if (NOT ${USE_SIM})
    message("not use sim >>>>>>>>>>>>>>>>>>>>>>>")
    add_definitions(-DNOT_USE_SIM)
endif()

set(includePath)
set(linkPath)

list(APPEND includePath "/usr/include/eigen3")

#To use BLAS acceleration, uncomment the following lines.
if (${USE_BLAS})
   message("--------------USE_BLAS-----------------")
   set(BLA_VENDOR Intel10_64lp)
   add_definitions(-DEIGEN_USE_MKL_ALL)
endif()

find_package(BLAS REQUIRED)
message(${BLAS_LIBRARIES})


# library yaml
find_package(yaml-cpp REQUIRED)
list(APPEND includePath ${YAML_INCLUDE_DIRS})

# Deeprobotics SDK
set(DEEPROBOTICS_SDK_DIR ${PROJECT_SOURCE_DIR}/extern/deeprobotics_legged_sdk)
add_subdirectory(${DEEPROBOTICS_SDK_DIR})
list(APPEND includePath ${DEEPROBOTICS_SDK_DIR}/include)
list(APPEND linkPath ${CMAKE_CURRENT_BINARY_DIR}/../../devel/lib)

# TinyEKF
set(TinyEKF_DIR ${PROJECT_SOURCE_DIR}/extern/TinyEKF)
list(APPEND includePath "${TinyEKF_DIR}/src")
file(GLOB_RECURSE TinyEKF_SRC "${TinyEKF_DIR}/src/*.c")

# library matplotlib-cpp
find_package(PythonLibs REQUIRED)
list(APPEND includePath ${PYTHON_INCLUDE_DIRS})
#list(APPEND includePath ${PROJECT_SOURCE_DIR}/extern/matplotlib-cpp)
find_package(Python3 COMPONENTS Interpreter Development REQUIRED)

# MIT AMD
add_subdirectory("include/quadruped/utils/amd")

# library TinyNURBS
add_subdirectory(${PROJECT_SOURCE_DIR}/extern/tinynurbs)

# library QuadProgpp
set(QuadProgpp_DIR ${PROJECT_SOURCE_DIR}/extern/QuadProgpp)
add_subdirectory(${QuadProgpp_DIR})
list(APPEND includePath "${QuadProgpp_DIR}/src")
link_directories(${CMAKE_CURRENT_BINARY_DIR}/extern/QuadProgpp)

# library qpOASES
set(qpOASES_DIR ${PROJECT_SOURCE_DIR}/extern/qpOASES)
add_subdirectory(${qpOASES_DIR})
list(APPEND includePath "${qpOASES_DIR}")
list(APPEND includePath "${qpOASES_DIR}/include")

# ROS
find_package(catkin REQUIRED COMPONENTS
        roscpp
        geometry_msgs
        std_msgs
        sensor_msgs
        tf
        roslib
        )

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS roscpp
                   geometry_msgs
                   std_msgs
                   sensor_msgs
                   roslib
)

list(APPEND includePath include/quadruped src)

# add source files
file(GLOB_RECURSE sources "src/*.cpp")
if (NOT ${USE_SIM})
list(REMOVE_ITEM includePath "include/quadruped/robots/qr_robot_sim.h"
                             "include/quadruped/robots/qr_lite2_sim.h"
                             "include/quadruped/robots/qr_lite3_sim.h"
                             )
set(filtered_sources ${PROJECT_SOURCE_DIR}/src/robots/qr_robot_sim.cpp
                    ${PROJECT_SOURCE_DIR}/src/robots/qr_robot_lite2_sim.cpp
                    ${PROJECT_SOURCE_DIR}/src/robots/qr_robot_lite3_sim.cpp)
list(REMOVE_ITEM sources ${filtered_sources})
message("${filtered_sources}") 
message("${PROJECT_SOURCE_DIR}/src/robots/qr_robot_sim.cpp")
message("//////////////////////////////////////: , ${sources}")
endif()

# add include files
list(APPEND includePath "config/")

list(APPEND includePath ".")

# create library
add_library(quadruped SHARED ${sources} ${TinyEKF_SRC})

target_include_directories(quadruped PUBLIC ${includePath} ${catkin_INCLUDE_DIRS})

target_link_directories(quadruped PUBLIC ${linkPath})

target_link_libraries(quadruped PUBLIC
    ${PYTHON_LIBRARIES}
    ${YAML_CPP_LIBRARIES}
    ${catkin_LIBRARIES}
    ${BLAS_LIBRARIES}
    deeprobotics_legged_sdk
    lcm tinynurbs::tinynurbs
    MITAMD quadprog qpOASES)

add_dependencies(quadruped ${catkin_EXPORTED_TARGETS})



