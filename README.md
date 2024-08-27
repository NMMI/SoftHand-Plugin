# Gazebo Simulation of the QBRobotics v1.2 and IIT SoftHandWide and IIT SoftHand_V3

## 1) Description 

![SoftHand](https://github.com/IIT-SoftBots/SoftHand-Plugin/blob/master/images/soft-hand-gazebo.jpg)

This repository contains the ROS/Gazebo packages for the simulation of the Pisa/IIT hand:
* [NMMI web page](https://www.naturalmachinemotioninitiative.com/softhand)
* [Centro Piaggio web page](https://www.centropiaggio.unipi.it/pisaiit-softhand)

### Packages
* **softhands_description** - contains the urdf descriptions of the different versions of the hand
* **softhand_plugin** - contains the two Gazebo model plugins for simulating the hand

## 2) Getting Started

The above packages are tested on ROS Noetic and no external dependencies are required. To install the packages in this repository, clone it into your catkin workspace source folder (e.g., `catkin_ws/src`), checkout to the desired branch, and `catkin_make`.

```
cd catkin_ws/src
git clone https://github.com/NMMI/SoftHand-Plugin.git
cd ..
catkin_make
```

## 3) Usage

Three different versions (1.2, Wide and 3) of the SoftHand are present. Launch examples are present:


* To spawn a hand inside Gazebo
```
roslaunch softhands_description softhands_gazebo.launch sh_version:=v1_2_research
```

Available versions:
```
sh_version:=v1_2_research
sh_version:=v1_wide
sh_version:=v3
```

In order to open and close the softhand gazebo model with the plugin you have to publish on the following topic:
```
/right_hand_v1_2_research/synergy_command
```
The exampla is for the softhand v1_2_research right, so the topic is changed respect to the side and version, and the value has to be between 0(open) to 1(close):
```
rostopic pub /right_hand_v1_2_research/synergy_command std_msgs/Float64 "data: 0.0" 
```
Hence the available topics are:
```
/right_hand_v1_2_research/synergy_command
/left_hand_v1_2_research/

/right_hand_v1_wide/synergy_command
/left_hand_v1_wide/synergy_command

/right_hand_v3/synergy_command
```


## 4) Missing model
The softhand version 3 is still in working progress, therefore there is only the right side.

## 5) Relevant Publications

* M. G. Catalano, Grioli, G., Farnioli, E., Serio, A., Piazza, C., and Bicchi, A., “Adaptive Synergies for the Design and Control of the Pisa/IIT SoftHand”, International Journal of Robotics Research, vol. 33, no. 5, pp. 768–782, 2014 [IJRR version (access required)](http://ijr.sagepub.com/content/33/5/768.abstract)
