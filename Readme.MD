# Gazebo Simulation of the PISA/IIT SoftHand

## 1) Description 

![SoftHand](https://github.com/NMMI/SoftHand-Plugin/blob/repair/images/soft-hand-gazebo.png)

This repository contains the ROS/Gazebo packages for the simulation of the Pisa/IIT hand:
* [NMMI web page](https://www.naturalmachinemotioninitiative.com/softhand)
* [Centro Piaggio web page](https://www.centropiaggio.unipi.it/pisaiit-softhand)

### Pacakges
* **softhands_description** - contains the urdf descriptions of the different versions of the hand
* **softhand_plugin** - contains the two Gazebo model plugins for simulating the hand

## 2) Getting Started

The above packages are tested on ROS Noetic and no external dependencies are required. To install the packages in this repository, clone it into your catkin workspace source folder (e.g., `catkin_ws/src`), checkout to the desired branch, and `catkin_make`.

```
cd catkin_ws/src
git clone https://github.com/NMMI/SoftHand-Plugin.git
git checkout repair
cd ..
catkin_make
```

## 3) Usage

Two different versions (1.0 and 2.0) of the SoftHand are present. Launch examples are present:

* To launch the RViz visualization
```
roslaunch softhands_description rviz_softhand_v1.launch 
```

* To spawn a hand inside Gazebo
```
roslaunch softhands_description softhand_v1_gazebo.launch 
```

## 4) Known Issues

* **TODO** - currently, a ros_control integration is missing in this repository. Work in progress!

## 5) Relevant Publications

* M. G. Catalano, Grioli, G., Farnioli, E., Serio, A., Piazza, C., and Bicchi, A., “Adaptive Synergies for the Design and Control of the Pisa/IIT SoftHand”, International Journal of Robotics Research, vol. 33, no. 5, pp. 768–782, 2014 [IJRR version (access required)](http://ijr.sagepub.com/content/33/5/768.abstract)

