# galileo_arm
This Repository contains the necessary files for Team Anveshak's Galileo Manipulator
<img src="home/om/Anveshak/galileo_arm.png">
## Getting Started

#### Installation

###### ROS Installation

* Follow the Official Installation instruction from the [ROS-Noetic] http://wiki.ros.org/noetic website. 



` sudo apt install ros-noetic-desktop-full `

##### Package Installation 

```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
```
` git clone https://github.com/gokulmk-12/galileo_arm.git `

```
cd ~/catkin_ws
source devel/setup.bash
```

#### How to Run
* Inside your ~/cakin_ws, enter the following command to launch galileo_arm.
  
```
cd ~/catkin_ws
roslaunch galileo_moveit_config demo.launch
```



