#include <ros/ros.h>

#include "espee/espee_ros_interface.h"


#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <rosgraph_msgs/Clock.h>    //To publish the clock

///for vicon data
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>

//function for read and publish vicon data
void read_n_publish_vicondata(ros::Publisher pub, const geometry_msgs::TransformStamped::ConstPtr& msg)
{
	pub.publish(msg);
}

//function for read and publish events
void read_n_publish_events(ros::Publisher pub, const dvs_msgs::EventArray::ConstPtr& msg)
{
	pub.publish(msg);
}

void write_file(std::ofstream& fd, std::array<float, STATE_SIZE>& cur, 
								const geometry_msgs::PoseStamped::ConstPtr& msg_ref)
{
	static double time_init=-1;
	if(time_init==-1)
		time_init = msg_ref->header.stamp.toSec();


	fd << (msg_ref->header.stamp.toSec()-time_init) << " ";
	for(int ii=1; ii<7; ++ii)
		fd << float(0.0) << " ";
	for(int ii=0; ii<STATE_SIZE; ++ii)
		fd << cur[ii] << " ";
	fd << msg_ref->pose.position.x << " ";
	fd << msg_ref->pose.position.y << " ";
	fd << msg_ref->pose.position.z << " ";
	fd << msg_ref->pose.orientation.w << " ";
	fd << msg_ref->pose.orientation.x << " ";
	fd << msg_ref->pose.orientation.y << " ";
	fd << msg_ref->pose.orientation.z << " ";
	fd << std::endl;
}

int main(int argc, char* argv[])
{
	ros::init(argc, argv, "espee_replay");

	ros::NodeHandle nh;
	ros::NodeHandle nh_private("~");
	std::string param_name;

  //file to print results
  std::ofstream pose_res_file;
	pose_res_file.open("Pose_recording_node.txt", std::ios::out);

	//get the number of sensors
	int NB_sensors = 0;
	nh_private.param("ESPEE_rig_p/NB_sensors", NB_sensors, 0);

	//to deal with vicon data and propagate events topics
	std::vector<ros::Publisher> event_cam_pub;
	event_cam_pub.reserve(NB_sensors);
	for(int ii=0; ii<NB_sensors; ++ii)
	{
		event_cam_pub.push_back(nh.advertise<dvs_msgs::EventArray>("/localization/events_Cam" + std::to_string(ii), 10));
	}

	ros::Publisher tfadd, clock_pub;
	nh_private.param<std::string>("groundtruth_topic", param_name, "");
	tfadd = nh.advertise<geometry_msgs::PoseStamped>( param_name, 10);
	
	//for clock management
	clock_pub = nh.advertise<rosgraph_msgs::Clock>("/clock", 10);
	rosgraph_msgs::Clock time_update;
	

	//variable related to bag reading
	nh_private.param<std::string>("path2bag", param_name, "");
	ROS_INFO_STREAM("path2bag = " << param_name);
	
	rosbag::Bag bag;
	bag.open(param_name, rosbag::bagmode::Read);  // BagMode open in Read mode
	
	//declare the list of topics to read
	std::vector<std::string> topics;
	
	topics.reserve(NB_sensors*2+1);
	for(int ii=0; ii<NB_sensors; ++ii)
	{
		nh_private.param<std::string>("cam" + std::to_string(ii), param_name, "");
		ROS_INFO_STREAM("cam topic name = " << param_name);

		topics.push_back(std::string(param_name + "/events"));
		topics.push_back(std::string(param_name + "/camera_info"));
	}

	///for comparing results
	nh_private.param<std::string>("groundtruth_topic", param_name, "");
	topics.push_back(std::string(param_name));
	
	//get the view to query
	rosbag::View view(bag, rosbag::TopicQuery(topics));
	static uint64_t event_count=0;
	
	//create the espee_ros object
	espee_ns::espee_ros node_obj(nh, nh_private);

	ROS_INFO("start of the rosbag reading");
	
	for (const rosbag::MessageInstance& m : view)
	{	
		int num_topic;
		for(int i=0; i<(NB_sensors); ++i)
		{			
			num_topic = (2*i+1);
			if (m.getTopic() == topics.at(num_topic) || ("/" + m.getTopic() == topics.at(num_topic)))
			{
				sensor_msgs::CameraInfo::ConstPtr cam_info = m.instantiate<sensor_msgs::CameraInfo>();
				if (cam_info != NULL)
					node_obj.Sensor.at(i).Cam_Info_Callback(cam_info);		//process the topic callback		
			}

			num_topic = (2*i);
			if (m.getTopic() == topics.at(num_topic) || ("/" + m.getTopic() == topics.at(num_topic)))
			{
				dvs_msgs::EventArray::ConstPtr events = m.instantiate<dvs_msgs::EventArray>();
				if (events != NULL){	
					node_obj.Sensor.at(i).events_Cam_Callback(events);			//process the topic callback
					read_n_publish_events(event_cam_pub[i], events);
				}
				event_count += events->events.size();

				time_update.clock = events->events[0].ts;
				clock_pub.publish(time_update);
			}	
		}
		/*
		printf("press a button to continue\n");
		char name; 
		name = getchar(); 
		*/
	
		num_topic = NB_sensors*2;
		if (m.getTopic() == topics.at(num_topic) || ("/" + m.getTopic() == topics.at(num_topic)))
		{
			geometry_msgs::PoseStamped::Ptr vicon_pose = m.instantiate<geometry_msgs::PoseStamped>();
			if (vicon_pose != NULL)
			{
				std::array<float, STATE_SIZE> Pose_espee = {};
				node_obj.get_current_pose(Pose_espee);
				write_file( pose_res_file, Pose_espee, vicon_pose);
				vicon_pose->header.frame_id = "world";
				tfadd.publish(vicon_pose);			//process the topic callback
			}
		}
		if(!ros::ok()){		//|| event_count>=2e6
			pose_res_file.close();
			break;
		}
	}

	bag.close();  // close Bag reading
	ROS_INFO("end processing rosbag");
	
	return 0;
}
