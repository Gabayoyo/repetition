# For installation


Must manually install ros2 interface packages


See ROS2 documentation for humble [here](https://docs.ros.org/en/humble/index.html)

Install colcon [here](https://docs.ros.org/en/kilted/Tutorials/Beginner-Client-Libraries/Colcon-Tutorial.html)



# Helpful commands:

`ros2 pkg create --build-type ament_python --node-name my_node my_package` : builds new package with node - shouldn't have to do this again

Remember to source workspace from this directory after colcon building: `source install/local_setup.bash`

To run kinematics (temporarily): `ros2 run kinematics_package my_node`



source /opt/ros/lyrical/setup.bash
source ~/interbotix_ws/install/setup.bash
source install/setup.bash

THEN 

QT_QPA_PLATFORM=xcb ros2 launch kinematics_package custom_arm.launch.py use_sim:=true



source /opt/ros/lyrical/setup.bash
mkdir -p ~/interbotix_ws/src && cd ~/interbotix_ws/src
git clone -b humble https://github.com/Interbotix/interbotix_ros_core.git
git clone -b humble https://github.com/Interbotix/interbotix_ros_toolboxes.git
git clone -b humble https://github.com/Interbotix/interbotix_ros_manipulators.git


cd ~/interbotix_ws
sudo rosdep init   # only if you haven't already
rosdep update
rosdep install --from-paths src -ryi
colcon build --symlink-install --packages-up-to interbotix_xsarm_control






