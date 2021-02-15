#!/bin/bash

rosrun multi_uav_control distributed_control_node_uav_0 _resources_path:="/home/jbhayet/opt/repositories/devel/quadrotor-simulation-gazebo/src/multi_uav_control/"&

rosrun multi_uav_control distributed_control_node_uav_1 _resources_path:="/home/jbhayet/opt/repositories/devel/quadrotor-simulation-gazebo/src/multi_uav_control/"&

rosrun multi_uav_control distributed_control_node_uav_2 _resources_path:="/home/jbhayet/opt/repositories/devel/quadrotor-simulation-gazebo/src/multi_uav_control/"&

rosrun multi_uav_control distributed_control_node_uav_3 _resources_path:="/home/jbhayet/opt/repositories/devel/quadrotor-simulation-gazebo/src/multi_uav_control/"&
