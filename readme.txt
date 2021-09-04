Moin!

Requirements for using the acoustic simulation from csv-files:
- ROS Noetic
- Python 3
- https://github.com/nsi18/acousticSimulation.git

additional requirements for using the acoustic simulation in Gazebo, with BlueRov as Agent and Path control:
- Gazebo
- QGroundControl

https://github.com/FormulasAndVehicles/PX4-Autopilot.git
https://github.com/FormulasAndVehicles/bluerov_sim.git
https://github.com/HippoCampusRobotics/control.git
https://github.com/HippoCampusRobotics/hippocampus_sim.git
https://github.com/HippoCampusRobotics/hippocampus_msgs.git
https://github.com/HippoCampusRobotics/mavlink.git
https://github.com/HippoCampusRobotics/mavros.git
https://github.com/HippoCampusRobotics/path_planning.git

Structur should be:
PX4-Autopilot
catkin_ws
    ├── build
    ├── devel
    ├── logs
    └── src
      ├── bluerov_sim
      ├── control
      ├── hippocampus_sim
      |── hippocampus_msgs
      ├── mavlink
      ├── mavros
      ├── path_planning
      └── acoustic_sim
      
Don't forget to rebuild ros before running simulation in Gazebo!
