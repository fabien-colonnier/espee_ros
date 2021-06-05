#include <ros/ros.h>

#include "espee/espee_ros_interface.h"

int main(int argc, char* argv[])
{
	ros::init(argc, argv, "espee");

	ros::NodeHandle nh;
	ros::NodeHandle nh_private("~");

	espee_ns::espee_ros node_obj(nh, nh_private);

	ros::spin();
	
	return 0;
}
