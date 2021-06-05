# ESPEE package for visual odometry

## config file
The YAML file allows to adpat the main parameters of the algorithm.

**NB**: For float value, be careful of the notation for floating point value with "5.0" instead of "5". Especially, "5e-8" will not be properly understood.

This localization is a node where the Visual Odometry is computed without any mapping for now. It requires 2 Davis camera (compatibility is on the way if a driver gives similar output as the davis_ros_driver).

To be used, the following topics need to be available, especially if the launch file is in the same format as in replay.launch:
/davis_left/camera_info /davis_left/davis_ros_driver/parameter_descriptions /davis_left/davis_ros_driver/parameter_updates /davis_left/events /davis_right/camera_info /davis_right/davis_ros_driver/parameter_descriptions /davis_right/davis_ros_driver/parameter_updates /davis_right/events

The following files are modified from the commit (merge with master)
from the 25/09/2018 in the dual_camera branch of the atis_linux repository

AER_display.h 
EKF_localization.h
filters.h
AER_display.cpp
EKF_localization.cpp
filters.cpp

if update are made on this branch, an update could be needed here.
