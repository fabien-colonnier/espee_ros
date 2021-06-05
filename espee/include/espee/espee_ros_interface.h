#ifndef ESPEE_ROS_INTERFACE_H
#define ESPEE_ROS_INTERFACE_H

/// standard includes 
#include <ros/ros.h>

/// messages
#include <dvs_msgs/Event.h>
#include <dvs_msgs/EventArray.h>
#include <sensor_msgs/CameraInfo.h>
#include <geometry_msgs/PoseStamped.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

#include <input_depth_msgs/input_depth.h>

///non standard library
#include "espee/utilities.h"
#include "espee/color_map.h"
#include "espee/filters.h"
#include "espee/espee.h"
#include "espee/mapping.h"

//Parameters for depth sampling for LUT display
//ratio of input depth (val*input_depth)
// if Mapping is not defined otherwise use the parameter for max and min depths 
#define D_MIN_LUT_RATIO 0.2
#define D_MAX_LUT_RATIO 5

/***** ****************************************** *****************/
namespace espee_ns {

/******* acquisition of parameter function ******************************/
// to get parameters from yaml files
template<typename T>
bool GetParam(const ros::NodeHandle& nh, const::std::string& where_str, const::std::string& key, T& val, const T& default_val)
{
  nh.param(key, val, default_val);
  ROS_INFO_STREAM("[" << where_str << "] Param " << key << " : " << val);
  return true;
};


/******* definition of the different classes ******************************/
class espee_ros;		//forward declaration of espee_ros to be use as a pointer
class Camera {
	public:
		int num_sensor;   //get the number of the sensor

		///store the events of each sensor
		std::vector<event> td_Cam;  //td events 	
		
		///get the param of sensor
		sensor_info sensor_param;		//pointer to the correct sensor_param_list element
		
		///filter data
		#ifdef FILTER_DATA
			std::unique_ptr<refractory_filter> RF_filt;
			std::unique_ptr<filter_Undistort> Undist_filt;
		#endif		
		
		Camera(ros::NodeHandle nh, int id_cam, espee_ros* p);
		void events_Cam_Callback(const dvs_msgs::EventArray::ConstPtr& msg);
		void Cam_Info_Callback(const sensor_msgs::CameraInfo::ConstPtr& msg);
		
	private:
		ros::NodeHandle nh_;
		
		ros::Subscriber event_sub_;
		ros::Subscriber Cam_info_sub_;
		
		espee_ros* epf_object_pointer;  //TODO use as a shared pointer in order to call the functions after event callback;
};

class display_LUT_ros{
	private:
		///To show LUT used by EKF and the ones updated by mapping
		ros::Publisher LUT_pub;
		ros::Publisher Mapping_pub;

		Color_maps colr;	//Used to display color in the LUT display
		float D_max;
		float D_min;
		float depth_scale; 

		void show_LUT(uint32_t Window_sizeX, uint32_t Window_sizeY, pcl::PointCloud<pcl::PointXYZ>::Ptr features_ptr, 
						pose& X, ros::Publisher& pub_img, sensor_info* param, ros::Time display_time);

	public:
		void Publish_LUT(sensor_info* param, pose& X,pose& X_rel_in,
											pcl::PointCloud<pcl::PointXYZ>::Ptr pc_ptr, ros::Time display_time);
		void Publish_LUT_withMAP(sensor_info* param, pose& X,pose& X_rel_in,
															pcl::PointCloud<pcl::PointXYZ>::Ptr pc_ptr, ros::Time display_time, Mapping_EMVS& obj_list);

		display_LUT_ros(ros::NodeHandle nh, int idx_sensor, int publish_map, float min_dist, float max_dist);
};

class output_results_ros{
	private:
		ros::NodeHandle nh_, nh_private;
		
		ros::Publisher odom_publisher;  	//pose publisher
		ros::Publisher pntcld_publisher; 	//point cloud publisher

	public:
		void Publish_results_inGT(std::array<float, STATE_SIZE>& pose_curr, pose& X_riginit, pcl::PointCloud<pcl::PointXYZ>::Ptr pc_ptr, ros::Time last_evt_time );
		void Publish_results_raw(const pose& pose_c, pcl::PointCloud<pcl::PointXYZ>::Ptr pc_ptr, ros::Time last_evt_time);
		output_results_ros(ros::NodeHandle nh);
};

class espee_ros {
	private:
		ros::NodeHandle nh_, nh_private;
		ros::Subscriber input_depth_sub_;	//get the input depth	
		
		///fusion of event vectors
		std::vector<event> td_all;  //td events for all
		
		///variables and functions for mapping if defined
		Mapping_param_struct map_param;
		
		//output and display variables
		ros::Time last_display_time;  //last time since update of LUT
		std::vector<display_LUT_ros> display_vec;
		output_results_ros output_res_obj;
		
		///Localization variables
		EKF_param_struct ekf_param;
		ESPEE_param_struct espee_param;
		Init_param_struct initmap_param;
		bool ekf_initialized;
		std::unique_ptr<espee_vo> espee_obj;

		std::vector<float> input_depth_values;

		void Parameter_aquisition(ros::NodeHandle nh_private); 
		void GetPose(ros::NodeHandle nh, std::string param_name,  pose& Xout);
		void input_Callback(const input_depth_msgs::input_depth::ConstPtr& msg);
		void Multi_cam_fusion(int idx_new);
		void Multi_cam_fusion_old(int idx_new); //TODO to remove used non original code (kept for now as the one used to get the paper's results, just need to change code in events_callback function)
		void process();

	public:
		///camera list
		std::vector<Camera> Sensor;   // N-cameras data and callback functions

		espee_ros(ros::NodeHandle nh, ros::NodeHandle nh_private);
		~espee_ros();
		void events_Callback(int num);
		void get_current_pose(std::array<float, STATE_SIZE>& curr_pose);			
};

}

#endif
