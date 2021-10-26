cmake_minimum_required(VERSION 3.5)
project(rtreach)

# Default to C++14
if (NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
  roscpp
  rospy
  std_msgs
  sensor_msgs
  message_generation
  geometry_msgs
  tf
)

## Specify additional locations of header files
include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_message_files(
  FILES
  velocity_msg.msg
  angle_msg.msg
  reach_tube.msg
  stamped_ttc.msg
  interval.msg
  obstacle_list.msg 
)

generate_messages(
   DEPENDENCIES
   sensor_msgs
   std_msgs
   geometry_msgs
 )

catkin_package(
  CATKIN_DEPENDS roscpp rospy sensor_msgs std_msgs geometry_msgs gazebo_ros
)

FIND_LIBRARY(CUSTOMLIB Rtreach src/)
FIND_LIBRARY(CUSTOMLIB2 Rtreachvis src/)
FIND_LIBRARY(CUSTOMLIB3 RtreachDynamicvis src/)
FIND_LIBRARY(CUSTOMLIB4 RtreachObs src/)
FIND_LIBRARY(CUSTOMLIB5 Rtreachdyn src/)


add_executable(reach_node src/reach_node_sync.cpp)
target_link_libraries(reach_node ${catkin_LIBRARIES} ${CUSTOMLIB})
add_dependencies(reach_node rtreach_generate_messages_cpp)

add_executable(visualize_node src/visualize_reachset.cpp)
add_dependencies(visualize_node rtreach_generate_messages_cpp)
target_link_libraries(visualize_node ${catkin_LIBRARIES} ${CUSTOMLIB2})

add_executable(visualize_node_dynamic src/vis_dynamic.cpp)
add_dependencies(visualize_node_dynamic rtreach_generate_messages_cpp)
target_link_libraries(visualize_node_dynamic ${catkin_LIBRARIES} ${CUSTOMLIB3})

add_executable(benchmark_node src/benchmarking_node.cpp)
target_link_libraries(benchmark_node ${catkin_LIBRARIES} ${CUSTOMLIB})
add_dependencies(benchmark_node rtreach_generate_messages_cpp)


add_executable(benchmark_hardware_node src/benchmarking_node_hardware.cpp)
target_link_libraries(benchmark_hardware_node ${catkin_LIBRARIES} ${CUSTOMLIB})
add_dependencies(benchmark_hardware_node rtreach_generate_messages_cpp)

add_executable(benchmark_node_dyn src/benchmark_dynamic.cpp)
target_link_libraries(benchmark_node_dyn ${catkin_LIBRARIES} ${CUSTOMLIB5})
add_dependencies(benchmark_node_dyn rtreach_generate_messages_cpp)

add_executable(visualize_obs src/visualize_obstacle.cpp)
add_dependencies(visualize_obs rtreach_generate_messages_cpp)
set_target_properties(visualize_obs PROPERTIES COMPILE_FLAGS "-DOBSTACLE_MODEL")
target_link_libraries(visualize_obs ${catkin_LIBRARIES} ${CUSTOMLIB4})

add_executable(visualize_obs_particles src/visualize_obstacle_particles.cpp)
add_dependencies(visualize_obs_particles rtreach_generate_messages_cpp)
set_target_properties(visualize_obs_particles PROPERTIES COMPILE_FLAGS "-DOBSTACLE_MODEL")
target_link_libraries(visualize_obs_particles ${catkin_LIBRARIES} ${CUSTOMLIB4})


add_executable(vis_node_param src/vis_dynamic_param.cpp)
add_dependencies(vis_node_param rtreach_generate_messages_cpp)
add_dependencies(vis_node_param ${catkin_EXPORTED_TARGETS})
target_link_libraries(vis_node_param  ${catkin_LIBRARIES} ${CUSTOMLIB5})


add_executable(reach_node_dyn src/reach_node_dynamic.cpp)
add_dependencies(reach_node_dyn rtreach_generate_messages_cpp)
add_dependencies(reach_node_dyn ${catkin_EXPORTED_TARGETS})
target_link_libraries(reach_node_dyn  ${catkin_LIBRARIES} ${CUSTOMLIB5})


add_executable(reach_node_hardware src/reach_node_hardware.cpp)
add_dependencies(reach_node_hardware rtreach_generate_messages_cpp)
add_dependencies(reach_node_hardware ${catkin_EXPORTED_TARGETS})
target_link_libraries(reach_node_hardware  ${catkin_LIBRARIES} ${CUSTOMLIB5})


add_executable(visualize_hardware src/visualize_hardware.cpp)
add_dependencies(visualize_hardware rtreach_generate_messages_cpp)
add_dependencies(visualize_hardware ${catkin_EXPORTED_TARGETS})
target_link_libraries(visualize_hardware  ${catkin_LIBRARIES} ${CUSTOMLIB5})

### Install laser converter node 
install(TARGETS 
  reach_node
  reach_node_dyn
  reach_node_hardware
  visualize_node
  visualize_node_dynamic
  visualize_obs
  vis_node_param
  benchmark_node
  benchmark_node_dyn
  benchmark_hardware_node
  visualize_hardware
  visualize_obs_particles
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch and other files 
install(DIRECTORY
        benchmarking
        launch
        logs
        obstacles
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

