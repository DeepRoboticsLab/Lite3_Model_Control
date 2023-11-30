cmake_minimum_required(VERSION 2.8.3)

project(lite3_motion_sim)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  geometry_msgs
  tf
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES robot_gazebo
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

# SET(CMAKE_C_COMPILER "aarch64-linux-gnu-gcc")
# SET(CMAKE_CXX_COMPILER "aarch64-linux-gnu-g++") 
set(CMAKE_BUILD_TYPE Release)
set(SRC_DIR_LIST "." src)

set(QUADRUPED "../lite3_motion_sdk" src)

add_definitions(-w) # warning ignore
set(CMAKE_CXX_FLAGS "-std=c++11 ${CAMKE_CXX_FLAGS}")

foreach(VAR ${SRC_DIR_LIST})
  set(TEMP)
  aux_source_directory(./src/ TEMP)
  set(SRC_LIST ${RCS_SRC_LIST} ${TEMP})
endforeach(VAR)

foreach(VAR ${QUADRUPED})
  set(TEMP)
  aux_source_directory(../lite3_motion_sdk/src/ TEMP)
  set(SRC_LIST ${RCS_SRC_LIST} ${TEMP})
endforeach(VAR)

#set(SRC_LIST src/main.cpp)

include_directories(
  ./include/
  ./lib/eigen3
)

include_directories(
 ../lite3_motion_sdk/lib/eigen3
 ../lite3_motion_sdk/include
)

include_directories(${catkin_INCLUDE_DIRS})

add_executable(${PROJECT_NAME} "src/gazebo_sim_and_real.cpp" "src/sim_fun.cpp" ${SRC_LIST})
target_link_libraries(${PROJECT_NAME} -lpthread -lm -lrt -ldl -lstdc++ )
target_link_libraries(${PROJECT_NAME}  PUBLIC ${catkin_LIBRARIES} )


#add_executable(${PROJECT_NAME}_sim "src/gazebo_sim.cpp" ${SRC_LIST})
#target_link_libraries(${PROJECT_NAME}_sim -lpthread -lm -lrt -ldl -lstdc++ )
#target_link_libraries(${PROJECT_NAME}_sim  PUBLIC ${catkin_LIBRARIES} )