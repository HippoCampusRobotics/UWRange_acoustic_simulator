Moin!

Requirements for using the acoustic simulation from csv-files:
- ROS Noetic
- Python 3
git clone https://github.com/nsi18/acoustic_sim.git

additional requirements for using the acoustic simulation in Gazebo, with BlueRov as Agent and Path control:
- Gazebo
- QGroundControl

git clone https://github.com/FormulasAndVehicles/PX4-Autopilot.git
git clone https://github.com/FormulasAndVehicles/bluerov_sim.git
git clone https://github.com/HippoCampusRobotics/control.git
git clone https://github.com/HippoCampusRobotics/hippocampus_sim.git
git clone https://github.com/HippoCampusRobotics/hippocampus_common.git
git clone https://github.com/HippoCampusRobotics/hippocampus_msgs.git
git clone https://github.com/HippoCampusRobotics/mavlink.git
git clone https://github.com/HippoCampusRobotics/mavros.git
git clone https://github.com/HippoCampusRobotics/path_planning.git

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
