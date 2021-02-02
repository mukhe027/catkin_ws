Original Workspace to conduct 6 agent Gazebo simulation. The worskpace is primarily using Rotors Simulator from ETHZ-ASL. However, a AprilTags package is also included to implement 
CV applications like AprilTag detection using aerial vehicles. For R-AL submission, the main FOV controller is written in Rotors Simulator--> Rotors Control packages with the name fov_controller.cpp.
To run the simulator. 
1) Download this complete package and compile it as a ROS workspace.
2) Run "roslaunch rotors_gazebo mav_rate_example.launch"
3)Run "rosrun rotors_control fov_controller"
