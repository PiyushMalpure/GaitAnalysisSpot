# GAIT ANALYSIS OF QUADRUPED ROBOT

This project was run on Ubuntu 20.04 with ROS Noetic

This project uses the champ package, for more information see: https://github.com/chvmp/champ

To install and run this package, follow the below commands:

A catkin workspace should be used for this project:
use catkin_make to create the workspace

When a new terminal is launched, please always run the following first:
```
cd /path/to/catkin_ws
source devel/setup.bash
```

Clone the Repo in src of your workspace:
```
sudo apt install -y python-rosdep
cd <your_ws>/src
gitclone https://github.com/PiyushMalpure/GaitAnalysisSpot.git
```

#### Launch gazebo 
` roslaunch spot_config gazebo.launch gazebo_world:="default"`
#### Launch keyboard controller
`roslaunch champ_teleop teleop.launch `
#### Controller Files
Controller Files are located inside src/spot_ros/RobotController
#### Inverse Kinematics 
Inverse Kinematics files are located inside src/spot_ros/InverseKinematics
