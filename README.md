# IVR Assignment
## Matt Timmons-Brown (s1823424) and Neil Weidinger (s1759126)

Code, resources and report for the Introduction to Vision and Robotics coursework at the University of Edinburgh, conducted in Year 3, Semester 1 of studies.

### Instructions to run Q4.2 - Null-space Control
* Clone this repository into your caktin workspace, make and source accordingly. Ubuntu 20, ROS Noetic.
* Run Gazebo simulation: ```roslaunch ivr_assignment spawn.launch```
* Run each camera: ```rosrun ivr_assignment image1.py``` and ```rosrun ivr_assignment image2.py``` (imshow previews disabled by default for better virtual machine performance)
* Run joint estimator/target tracking: ```rosrun ivr_assignment joint_target_estimation.py```
* Run null-space controller: ```rosrun ivr_assignment kinematics.py```
* Observe robot following sphere target, and avoiding cube obstacle