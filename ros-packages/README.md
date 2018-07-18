V-REP simulator for the da Vinci Research Kit (dVRK)
====================
This repository contains three ros packages:

* blob-tracker: performs tracking of features in the simulator images
* visual-servoing: contains the robot control functions
* needle-tracker: performs tracking of the needle

# Dependences
* Download and install ROS http://www.ros.org/
* Download and install visp http://visp.inria.fr/


Create a catkin_ws and download the packages in your src folder

Additionally you need the visp_ros package  http://wiki.ros.org/vision_visp

# Build
* Build using catkin build

# Run
To run the visual servoing example you need to run the tracker

`rosrun blob-tracker tracker_blobs_ibvs`

then start the visual servoing

`rosrun visual-servoing visual_servoing_ecm`

