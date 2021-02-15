# quadrotor-simulation-gazebo

## Installation (with [ros-noetic](http://wiki.ros.org/noetic/Installation)):

1. Install ROS.

2. Install osqp:
```
      git clone --recursive git@github.com:oxfordcontrol/osqp
      cd osqp
      mkdir build
      cd build
      cmake ..
      sudo make install
 ``` 
3. Download the project and its submodules (mav_comm and rotors)
```
      git clone --recursive git@github.com:cimat-ris/quadrotor-simulation-gazebo.git
```      
4. Compile with catkin
```
      catkin_make_isolated
```
In case you have a problem with the python3 interpreter not found, you can use:
```
      catkin_make_isolated -DPYTHON_EXECUTABLE="/usr/bin/python3"
```
(or whatever your python3 interpreter is)

## To run

```
     source devel_isolated/setup.bash
```   

Launch the Gazebo simulation, the quadrotors should be starting in hovering state:

```
      roslaunch multi_uav_control multiple_hummingbird.launch
```

Edit the multi_distributed_control.sh script to use your own path to resources and launch the script:
```
      sh multi_distributed_control.sh
```

