# An Open ROS Simulation Framework for TWR-Based Acoustic Localization of Underwater Robots: Design and Experimental Validation
by Fabian Steinmetz, Daniel Duecker, Nils Sichert, Christian Busse, Edwin Kreuzer, Christian Renner

implementation by Nils Sichert

## Short installation guide:

Requirements for using the acoustic simulation from csv-files:
- ROS Noetic
- Python 3

      
```
git clone https://github.com/nsi18/acoustic_sim.git
```
additional requirements for using the acoustic simulation in Gazebo, with BlueRov as agent and ability to follow a path:
- QGroundControl

Structur should be:
PX4-Autopilot
catkin_ws
- build
- devel
- logs
- src
  - bluerov_sim
  - control
  - hippocampus_sim
  - hippocampus_msgs
  - mavlink
  - mavros
  - path_planning
  - acoustic_sim
```
git clone https://github.com/FormulasAndVehicles/PX4-Autopilot.git
```
```
git clone https://github.com/nsi18/acoustic_sim.git
git clone https://github.com/nsi18/bluerov_sim.git
git clone https://github.com/nsi18/control.git
git clone https://github.com/nsi18/hippocampus_common.git
git clone https://github.com/nsi18/hippocampus_msgs.git
git clone https://github.com/nsi18/mavlink.git
git clone https://github.com/nsi18/mavros.git
git clone https://github.com/nsi18/path_planning.git
```

Don't forget to rebuild your workspace, before running simulation in Gazebo!

## Configuration
There are two config-files (acoustic simulation, filter settings):
You can finde them: acoustic_sim/config/

### Acoustic Simulation settings (/acoustic_config.json)
Timing-Table:

#### general settings
- Timings: refer to notation in timing-table
- SOS: speed of sound
- algortihm: alternating/ broadcast (MAC-protocoll)
- pollcircle: timetrgd (time triggered), lstAcktrgd (last Response and time triggered)
- TimeOut: time soundwave exists

#### Modem settings
```
{
    "type": "[agent or anchor",
    "name": "[Name]",
    "shortname": "[Shortname]",
    "modem": {
               "id": [INT],
               "DelayTime": [Float],
               "PacketReceptionRate": [Float between 1 = 100% and 0 = 0%],
               "dst": "broadcast",
               "packetTyp": "[TYPE_RANGING_POLL or TYPE_RANGING_ACK]"
             },
    "position":[0.8, 2.55,-0.3]        
}
```

### Filter settings (/filter_config.json)
#### general settings
- filterTyp: UKF/EKF selection of applied filter
- PredictFrequency: frequency of prediction iterations per second
- lengthDatabag: length of a Databag which record the last filter settings of the last few simulations runs; important for correcting the time step
- MeasErrLoc/ MeasErrScale: Mean and standard deviation [m] applied on prediction data

#### UKF settings
- beta, alpha, kappa, process noise, initial state, inital covariance, covarianz of measurements

#### EKF settings
- process noise, inital state, inital covariance, covarianz of measurements


## Usage
There are 2 independent functionalities:
1. running within Gazebo and doing a live simulation
  1. set flag in acoustic_config: "RosRun":true
  2. to start simulation launch .launchfile with following code:
  >roslaunch acoustic_sim simulation_path.launch vehicle_type:=bluerov use_external_controller:=true vehicle_name:=bluero
2. simulate from recorded gps-data (must be interpolated to 100Hz!)
  1. set flag in acoustic_config: "RosRun":false
  set CSV-file path in acoustic_sim/src/acoustic_sim/dataloader_class.py
  2. run simulation by executing acoustic_sim/nodes/simulation_class.py or:
  >rosrun acoustic_sim simulation_class.py

If you have issues with installing the simulator or other questions, feel free to contact:
- Daniel Duecker: daniel.duecker@tuhh.de
- Nils Sichert: nils.sichert@tuhh.de

