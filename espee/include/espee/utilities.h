#ifndef UTILITIES_DEFINITION_H
#define UTILITIES_DEFINITION_H

#include <unistd.h>
#include <math.h>
#include <vector>

#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>

#include <thread>	//for multithreaded tasks

#include <Eigen/Dense>

/* include for writing into file */
#include <iostream>  // Stream class to read/write from files
#include <fstream>  // Stream class to both read and write from/to files.


///constant
#define PI   3.141592653589793238462643383279502884L /* pi */

#define STATE_SIZE 7  // x,y,z,qw,qx,qy,qz

#define DISPLAY_INTERVAL 0.033       //minimum number of seconds to occur between frames
// #define ENSURE_POSE_PUB							 //if not used pose is updated after each event message or at PUB_SAMPLINGTIME otherwise
#define PUB_SAMPLINGTIME 0.005

///parameters of the algorithm
#define DEFAULT_DEPTH 0.5   //default input depth for all the cameras in [m]

//#define FILTER_DATA 		//apply filters Refractory and undistort

///For debugging purposes:
//#define VERBOSE  //----prints intermediate results in plain text to the terminal
//#define WRITE_EVENT_FILE   //--prints all the events processed by espee_vo in a file named "event_list_processed_ros.txt"
//#define WRITE_EKFSTATE_FILE   //write the output state into a file named "ekfstate.txt"
//#define WRITE_LUT1   //--prints the first LUT used by espee_vo in a file named "first_LUT<sensor_num>.txt"

namespace espee_ns {
	
struct event
{
	int id_sensor;
	unsigned char polarity;
	uint16_t x;
	uint16_t y;
	ros::Time ts;		//stays as ros::Time instead of uint64_t as easier to work with. It wouldn't be too hard to change if required for use outside of ROS environment
	event(int id_sensor_arg, unsigned char polarity_arg, int16_t x_arg, int16_t y_arg, ros::Time ts_arg) :
	id_sensor(id_sensor_arg), polarity(polarity_arg), x(x_arg), y(y_arg), ts(ts_arg)
	{}
};

//prototype of the operators for the event type
bool operator>(const event &first, const event &second); 
bool operator<(const event &first, const event &second); 
bool operator<=(const event &first, const event &second);
bool operator>=(const event &first, const event &second);

/******* Deal with camera parameters ******************************/
struct pose
{
	Eigen::Vector3f p; //position of the sensor/rig (x y z)
	Eigen::Quaternionf q; //quaternion q representing the rotation from the body coordinate frame B (w, x, y, z) to the world coordinate
};

class sensor_info {
public:
	bool got_sensor_info = false;
	
	uint16_t x_dim=0, y_dim=0;  //size of the frame
	
	//parameters for undistort filter
	float fxp; // = (251.936603);
	float fyp; // = (252.081315);
	float cxp; // = 116.536931;
	float cyp; // = 82.816373;
	
	float k1; // = -0.338614;
	float k2; // = 0.218341;
	float k3; // = 0;
	float p1; // = 0.001271;
	float p2; // = 0.003511;
	
	///camera related functions
	float project(const Eigen::Vector3f& point_world, const Eigen::Vector3f& X, 
								const Eigen::Quaternionf& quat, int16_t* pixels); //finds the pixel location a point in the world would project to (based on sensor state X, quat)
	void camera_to_world(const Eigen::Vector3f& point_camera, const Eigen::Vector3f& position, 
											 Eigen::Quaternionf& quat, Eigen::Ref<Eigen::Vector3f> point_world);  //convert from camera co-ordinates to world co-ordinates
	void world_to_camera(const Eigen::Vector3f& point_world, const Eigen::Vector3f& position, 
											 const Eigen::Quaternionf& quat, Eigen::Ref<Eigen::Vector3f> point_camera);  //convert from world co-ordinates to camera co-ordinates
	void camera_to_pixels(const Eigen::Vector3f& point_camera, int16_t* pixels);  //convert from camera co-ordinates to pixel co-ordinates
	void pixel_to_camera(int16_t* pixels, Eigen::Ref<Eigen::Vector3f> point_camera);  //convert from pixel co-ordinates to camera co-ordinates (up to scale)
};
	
	//quaternion related functions 
	void vec2quat(const Eigen::Vector3f& vec, Eigen::Quaternionf& quat);
	void quat2vec(const Eigen::Quaternionf& quat, Eigen::Ref<Eigen::Vector3f>& vec);
	void vec2skew(const Eigen::Vector3f& u, Eigen::Ref<Eigen::Matrix<float, 3, 3>> u_skew);

	//get X2 state from knowledge of state X and the relative state between the two.
	void get_state2_from_state1(const pose& X1, const pose& X_relative, pose& X2);
	void get_state1_from_state2(const pose& X2, const pose& X_relative, pose& X1);
}

#endif
