# For installation

See ROS2 documentation for humble [here](https://docs.ros.org/en/humble/index.html)

Install colcon [here](https://docs.ros.org/en/kilted/Tutorials/Beginner-Client-Libraries/Colcon-Tutorial.html)



# Helpful commands:

`ros2 pkg create --build-type ament_python --node-name my_node my_package` : builds new package with node - shouldn't have to do this again

Remember to source workspace from this directory: `source install/local_setup.bash`

To run kinematics (temporarily): `ros2 run kinematics_package my_node`