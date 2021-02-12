# quadrotor-simulation-gazebo

Installation (with [ros-noetic](http://wiki.ros.org/noetic/Installation)):

1. Install ROS.

2. Install osqp:
```
      git clone git@github.com:oxfordcontrol/osqp
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
