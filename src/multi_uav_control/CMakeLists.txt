cmake_minimum_required(VERSION 2.8.3)
project(multi_uav_control)

#add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp	  
  trajectory_msgs    
  mav_msgs
  geometry_msgs
)

find_package(osqp REQUIRED)

#find_package(Eigen3 REQUIRED)

catkin_package(
  #INCLUDE_DIRS include ${Eigen_INCLUDE_DIRS}
  CATKIN_DEPENDS roscpp trajectory_msgs mav_msgs geometry_msgs
)

include_directories(
  MPC.h
  ${catkin_INCLUDE_DIRS}
)

add_executable(decentralized_control_node_uav_0 src/nodes/decentralized_control_node_uav_0.cpp src/library/MPC.cpp src/library/Consensus.cpp src/library/IOEigen.cpp)
#add_dependencies(multi_uav_control ${catkin_EXPORTED_TARGETS})
target_link_libraries(decentralized_control_node_uav_0 PRIVATE osqp::osqp ${catkin_LIBRARIES})

add_executable(decentralized_control_node_uav_1 src/nodes/decentralized_control_node_uav_1.cpp src/library/MPC.cpp src/library/Consensus.cpp src/library/IOEigen.cpp)
#add_dependencies(multi_uav_control ${catkin_EXPORTED_TARGETS})
target_link_libraries(decentralized_control_node_uav_1 PRIVATE osqp::osqp ${catkin_LIBRARIES})

add_executable(decentralized_control_node_uav_2 src/nodes/decentralized_control_node_uav_2.cpp src/library/MPC.cpp src/library/Consensus.cpp src/library/IOEigen.cpp)
#add_dependencies(multi_uav_control ${catkin_EXPORTED_TARGETS})
target_link_libraries(decentralized_control_node_uav_2 PRIVATE osqp::osqp ${catkin_LIBRARIES})

add_executable(decentralized_control_node_uav_3 src/nodes/decentralized_control_node_uav_3.cpp src/library/MPC.cpp src/library/Consensus.cpp src/library/IOEigen.cpp)
#add_dependencies(multi_uav_control ${catkin_EXPORTED_TARGETS})
target_link_libraries(decentralized_control_node_uav_3 PRIVATE osqp::osqp ${catkin_LIBRARIES})

add_executable(distributed_control_node_uav_0 src/nodes/distributed_control_node_uav_0.cpp src/library/MPC.cpp src/library/Consensus.cpp src/library/IOEigen.cpp)
#add_dependencies(multi_uav_control ${catkin_EXPORTED_TARGETS})
target_link_libraries(distributed_control_node_uav_0 PRIVATE osqp::osqp ${catkin_LIBRARIES})

add_executable(distributed_control_node_uav_1 src/nodes/distributed_control_node_uav_1.cpp src/library/MPC.cpp src/library/Consensus.cpp src/library/IOEigen.cpp)
#add_dependencies(multi_uav_control ${catkin_EXPORTED_TARGETS})
target_link_libraries(distributed_control_node_uav_1 PRIVATE osqp::osqp ${catkin_LIBRARIES})

add_executable(distributed_control_node_uav_2 src/nodes/distributed_control_node_uav_2.cpp src/library/MPC.cpp src/library/Consensus.cpp src/library/IOEigen.cpp)
#add_dependencies(multi_uav_control ${catkin_EXPORTED_TARGETS})
target_link_libraries(distributed_control_node_uav_2 PRIVATE osqp::osqp ${catkin_LIBRARIES})

add_executable(distributed_control_node_uav_3 src/nodes/distributed_control_node_uav_3.cpp src/library/MPC.cpp src/library/Consensus.cpp src/library/IOEigen.cpp)
#add_dependencies(multi_uav_control ${catkin_EXPORTED_TARGETS})
target_link_libraries(distributed_control_node_uav_3 PRIVATE osqp::osqp ${catkin_LIBRARIES})

