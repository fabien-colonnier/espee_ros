#include "espee/espee_ros_interface.h"    

namespace espee_ns {
	
Camera::Camera(ros::NodeHandle nh, int id_cam, espee_ros* p):
	nh_(nh), epf_object_pointer(p), num_sensor(id_cam)
{
	printf("num_sensor : %d\n", num_sensor);

	std::string topic_event("events_Cam" + std::to_string(id_cam));
	std::string topic_info("info_Cam" + std::to_string(id_cam));

	/// setup subscribers and publishers
	event_sub_ = nh_.subscribe(topic_event, 1, &Camera::events_Cam_Callback, this);
	Cam_info_sub_ = nh_.subscribe(topic_info, 1, &Camera::Cam_Info_Callback, this);
}

void Camera::events_Cam_Callback(const dvs_msgs::EventArray::ConstPtr& msg)
{		
	#ifdef VERBOSE
		static int num_call=0;
		printf("cam %d: events_Callback call %d \n", num_sensor, ++num_call);
	#endif
		
	td_Cam.reserve(td_Cam.size() + msg->events.size());	//allocate the space for the new vector of events
	for (int i = 0; i < msg->events.size(); ++i)
	{
		///put event at the end of the vector
		td_Cam.emplace_back(event(num_sensor, msg->events[i].polarity, msg->events[i].x, msg->events[i].y, msg->events[i].ts));
	}
	
	///call functions that process events: apply filters
	#ifdef FILTER_DATA
	unsigned long Nb_new_evts = msg->events.size();
	if(sensor_param.got_sensor_info)
	{
		if(RF_filt==nullptr)
		{
			printf("Initialization of Refractory filter, sensor %d\n", num_sensor);
			//get filter parameter
			int input_RF_period;
			ros::NodeHandle nh_private("~");
			nh_private.param<int>("RF_period", input_RF_period, 0);
			//create RF filter
			RF_filt = std::make_unique<refractory_filter>(input_RF_period, sensor_param.x_dim, sensor_param.y_dim);
		}
		else
		{
			Nb_new_evts = RF_filt->processData(td_Cam, Nb_new_evts );
			#ifdef VERBOSE
				printf("cam %d: %lu events filtered with Refractory filter \n", num_sensor, msg->events.size()-Nb_new_evts);
			#endif
		}
		
		if(Undist_filt==nullptr)
		{
			printf("Initialization of undistort filter, sensor %d\n", num_sensor);
			//create undistort filter
			Undist_filt = std::make_unique<filter_Undistort>(sensor_param);
		}
		else
		{
			Nb_new_evts = Undist_filt->processData(td_Cam, Nb_new_evts );
			#ifdef VERBOSE
				printf("cam %d: %lu events filtered with all filters \n", num_sensor, msg->events.size()-Nb_new_evts);
			#endif
		}
		
	}
	#endif

	epf_object_pointer->events_Callback(num_sensor);
	
	
}

void Camera::Cam_Info_Callback(const sensor_msgs::CameraInfo::ConstPtr& msg)
{
	if(!sensor_param.got_sensor_info)
	{
		std::cout << " cam "<< num_sensor << ": \n\tframe height " << msg->height << std::endl;
		std::cout << "\tframe width " << msg->width << std::endl;
		
		sensor_param.x_dim = msg->width;
		sensor_param.y_dim = msg->height;
		
		sensor_param.fxp = msg->K[0];
		sensor_param.fyp = msg->K[4];
		sensor_param.cxp = msg->K[2];
		sensor_param.cyp = msg->K[5];
		
		if(msg->D.empty()==0){     //msg->D.empty() return 1 if empty
			sensor_param.k1 = msg->D[0];
			sensor_param.k2 = msg->D[1];
			sensor_param.k3 = msg->D[4];
			sensor_param.p1 = msg->D[2];
			sensor_param.p2 = msg->D[3]; 
			std::cout << "\tdistortion coefficient: k1=" << sensor_param.k1 << ", k2=" << sensor_param.k2 << ", k3="
				<< sensor_param.k3 << ", p1=" << sensor_param.p1 << ", p2=" << sensor_param.p2 << std::endl;
		}
			
		//indication that the data have been loaded
		sensor_param.got_sensor_info = true;
		
		ROS_INFO_STREAM("init cam " << num_sensor << " done");
	}
}

void espee_ros::Parameter_aquisition(ros::NodeHandle nh_private) 
{
	//get EKF parameters
	std::string module = "EKF Param";
	ros::NodeHandle nh_ekf_param(nh_private, "EKF_p");

	GetParam(nh_ekf_param, module, "Ptrans_init", ekf_param.Ptrans_init_, 0.001);
	GetParam(nh_ekf_param, module, "Prot_init", ekf_param.Prot_init_, 0.1);
	GetParam(nh_ekf_param, module, "QtransXY_init", ekf_param.QtransXY_init_, 5.0e-9);
	GetParam(nh_ekf_param, module, "QtransZ_init", ekf_param.QtransZ_init_, 5.0e-9);
	GetParam(nh_ekf_param, module, "QrotXY_init", ekf_param.QrotXY_init_, 5.0e-5);
	GetParam(nh_ekf_param, module, "QrotZ_init", ekf_param.QrotZ_init_, 5.0e-5);
	GetParam(nh_ekf_param, module, "R_coeff", ekf_param.R_coeff_, 25.0);

	//get espee parameters
	module = "ESPEE_FE Param";
	ros::NodeHandle nh_espee_param(nh_private, "ESPEE_FE_p");
	GetParam(nh_espee_param, module, "edge_range", espee_param.Edge_range_, 5);
	GetParam(nh_espee_param, module, "ReMapTimeThreshold", espee_param.Remap_Time_Threshold_, 400);

	module = "ESPEE_rig Param";
	ros::NodeHandle nh_rignum_param(nh_private, "ESPEE_rig_p");
	GetParam(nh_rignum_param, module, "NB_sensors", espee_param.nb_sensors_, 0);

	ros::NodeHandle nh_rig_param(nh_private, "ESPEE_rig_p");
	
	GetPose(nh_rig_param, "absolute_xyz_qwqxqyqz", espee_param.Xinit_);

	for(int i=0; i<espee_param.nb_sensors_; i++)
	{
		pose pose_temp;
		GetPose(nh_rig_param, "relative_xyz_qwqxqyqz/sensor" + std::to_string(i), pose_temp);
		espee_param.Xrelative_.push_back(pose_temp);
	}

	//get init PCD parameters
	module = "Init Map Param";
	ros::NodeHandle nh_init_param(nh_private, "Init_map_p");
	GetParam(nh_init_param, module, "Mode", initmap_param.Mode_, 0); 		
	GetParam(nh_init_param, module, "Num_points", initmap_param.Num_points_, NUM_INIT_POINTS_DEFAULT); 	
	if(initmap_param.Mode_==1)
	{
		GetParam(nh_init_param, module, "PCDfile_path", initmap_param.path2PCD_, std::string("")); 
		GetPose(nh_init_param, "PCDrelative_xyz_qwqxqyqz", initmap_param.Pose_PCDrelative_);
	}	     	 

	///Initialization of the parameters for mapping
	module = "MAPPING Param";
	ros::NodeHandle nh_map_param(nh_private, "mapping_p");
	GetParam(nh_map_param, module, "Mapping_mode", map_param.mode_, 0);
	GetParam(nh_map_param, module, "D_MIN_MAPPING", map_param.D_min_mapping_, 0.4); 		//closest plane distance
	GetParam(nh_map_param, module, "D_MAX_MAPPING", map_param.D_max_mapping_, 5.0); 		//furthest plane distance
	GetParam(nh_map_param, module, "dz_min", map_param.dz_min_, 0.0075);   				//closest distance between 2 planes (the 2 closest)
	GetParam(nh_map_param, module, "Nz", map_param.Nz_, 200);	     	 //number of depth planes

	//criteria for creating a new keyframe
	GetParam(nh_map_param, module, "percent_Dmean_1", map_param.Percent_Dmean_1_, 0.3);     		//percentage of the mean depth for the DSI update
	GetParam(nh_map_param, module, "percent_Dmean_2", map_param.Percent_Dmean_2_, 0.4);    		//percentage of the mean depth for the DSI update
	GetParam(nh_map_param, module, "MAX_EVENT_PER_DSI", map_param.max_event_per_DSI_, 15000000);  	//maximum of event for a new DSI to be created

	//Parameters for thresholding confidence map (like EVO algorithm)
	GetParam(nh_map_param, module, "Dim_Gauss_kernel", map_param.Dim_Gauss_kernel_, 5); 			//size of the Gaussian kernel for the filtering of Cmap  //init_mapping should be adapted in case of modifications
	GetParam(nh_map_param, module, "Sigma_Gauss_kernel", map_param.Sigma_Gauss_kernel_, 0.56); 		//std deviation of the Gaussian filter  
	GetParam(nh_map_param, module, "Filt_offset", map_param.Filt_offset_, -6.0); 				//offset to threshold some data in the comparison   

	//parameters for filtering Zmap
	GetParam(nh_map_param, module, "MedianFilt_window_size", map_param.MedianFilt_window_size_, 10); 	//size of the window for the median filter applied to the semi-dense depth map

	//parameters for Point Cloud filtering
	GetParam(nh_map_param, module, "PCl_Radius_thres", map_param.PCl_Radius_thres_, 0.02); 		//number of neighbors in the area of a features to be taken into account
	GetParam(nh_map_param, module, "PCl_nb_neighbors", map_param.PClF_nb_neighbors_, 3 );  	//number of nearest neighbors to take in account for the computation of the mean distance

	//parameters for the feature list
	GetParam(nh_map_param, module, "MinFeatures_to_update", map_param.MinFeatures_to_update_, 750);
}

void espee_ros::GetPose(ros::NodeHandle nh, std::string param_name,  pose& Xout)
{
	std::vector<float> yaml_list;

	if( nh.hasParam(param_name) )
		{
			nh.getParam(param_name, yaml_list);
			
			#ifdef VERBOSE
				printf("yaml_list : ");
				for(int j=0; j<7; j++)
					printf("%f, ", yaml_list[j]);
			#endif

			Xout.p = Eigen::Vector3f(yaml_list[0], yaml_list[1], yaml_list[2]);
			Xout.q = Eigen::Quaternionf(yaml_list[3], yaml_list[4], yaml_list[5], yaml_list[6]);

			ROS_INFO_STREAM("[" << param_name << "]  pose = [" 
											<< Xout.p(0) << ", " << Xout.p(1) << ", " << Xout.p(2) << ", " 
											<< Xout.q.w() << ", " << Xout.q.x() << ", " << Xout.q.y() << ", " << Xout.q.z() << "]");


		}
		else
		{
			ROS_INFO_STREAM("[" << param_name << "] has not been detected. Default [0,0,0,1,0,0,0] is applied. ");
			Xout.p = Eigen::Vector3f::Zero();
			Xout.q = Eigen::Quaternionf(1, 0, 0, 0);
		}
}

espee_ros::espee_ros(ros::NodeHandle nh, ros::NodeHandle nh_private) : 
	nh_(nh), 
	nh_private(nh_private),
	output_res_obj(nh)
{	
	/// Get the parameters ///
	Parameter_aquisition(nh_private); 

	///create the subscriber for the input depth and associated variables
	input_depth_sub_ = nh_.subscribe("/localization/input_depths", 1, &espee_ros::input_Callback, this);
	//initialized input values
	input_depth_values.reserve(espee_param.nb_sensors_);
	for(int i=0; i<espee_param.nb_sensors_; ++i){
		float temp;
		nh_private.param<float>("Depth_cam" + std::to_string(i), temp, DEFAULT_DEPTH); 
		input_depth_values.push_back(temp);
		printf("depth at initialization cam %d = %f\n", i, input_depth_values.at(i));
	}
	
	///create the sensor lists
	//sensor camera objects for callback
	Sensor.reserve(espee_param.nb_sensors_);
	for(int i=0; i<espee_param.nb_sensors_; ++i){
		Sensor.emplace_back(nh_, i, this);
	}

	//initialize output displays
	last_display_time = ros::Time(0);
	display_vec.reserve(espee_param.nb_sensors_);
	for(int i=0; i<espee_param.nb_sensors_; ++i){
		if(map_param.mode_==0)
			display_vec.emplace_back(nh_, i, 0, D_MIN_LUT_RATIO*input_depth_values.at(i),
																					D_MAX_LUT_RATIO*input_depth_values.at(i));
		else if(map_param.mode_==0)
			display_vec.emplace_back(nh_, i, 1, map_param.D_min_mapping_, map_param.D_max_mapping_);
		else
			display_vec.emplace_back(nh_, i, 0, map_param.D_min_mapping_, map_param.D_max_mapping_);

	}

}

espee_ros::~espee_ros()
{
	ROS_INFO("Exiting from espee_ros node, freeing all allocated memory");
}

void espee_ros::input_Callback(const input_depth_msgs::input_depth::ConstPtr& msg)
{		
	if(msg->N_sensor < espee_param.nb_sensors_)
		ROS_WARN("Not enough input depths specified in the messages, no changed applied\n");
	else{
		for(int i=0; i < espee_param.nb_sensors_; ++i){
			input_depth_values.at(i) = msg->depths[i];
			ROS_INFO("Cam %d input depth set at %f\n", i, input_depth_values.at(i));
		}
		
		espee_obj.reset();
	}
}

void espee_ros::events_Callback(int cam_num)
{
	#ifdef VERBOSE
		static int num_call=0;
		printf("events_Callback call %d \n", ++num_call);
	#endif
	
	//merge the sensors data
	Multi_cam_fusion(cam_num);  //sort out the data of all sensors

	//Do the localization processing
	process();
}	

void espee_ros::Multi_cam_fusion(int idx_new)
{
	int i=0;
	
	while(i<espee_param.nb_sensors_ && !Sensor.at(i).td_Cam.empty())		// test also if one vector is empty	
	{
		++i;
	}

	///merge data of all sensors into one vector until the time of the last updated (only if all contain data)
	if(i>=espee_param.nb_sensors_)		//  && idx_cam_old!=-1 means that the previous loop went through all the sensors or they are all not empty
	{	
		//copy the new events at the end of the fused vector
		for(int ii=0; ii<espee_param.nb_sensors_; ++ii)
		{
			std::vector<event>::iterator end_temp;

			td_all.reserve(td_all.size() + Sensor[ii].td_Cam.size());
			end_temp = td_all.end();
			td_all.insert(td_all.end(), Sensor[ii].td_Cam.begin(), Sensor[ii].td_Cam.end());

			std::inplace_merge(td_all.begin(), end_temp, td_all.end());

			//remove the data copied
			Sensor[ii].td_Cam.erase(Sensor[ii].td_Cam.begin(), Sensor[ii].td_Cam.end());

			#ifdef VERBOSE
				//check if the events are properly sorted
				for(std::vector<event>::iterator it = td_all.begin(); it+1 < td_all.end(); ++it )
				{
					if(it->ts>(it+1)->ts)
					{
						std::cout << "[ERROR] iteration " << ii << ", mid event time = " << end_temp->ts << "\n";

						for(std::vector<event>::iterator j = std::max(it-2,td_all.begin()) ; j<= std::min(it+3,td_all.end()-1) ; ++j )
						{
							std::cout << "\t event " << " : id_sensor = " << (j->id_sensor ? 0 : 1) << 
							", time = " << j->ts << "\n";
						}
					}
				}
			#endif

		}

	}	
}

void espee_ros::process()
{
	if(espee_obj == nullptr){
		///check that the sensor data have been received
		bool init_sensor_done = false;
		int j=0;
		for(int i=0; i<espee_param.nb_sensors_; ++i){
			if(Sensor[i].sensor_param.got_sensor_info)
			{
				printf("[Process] sensor %d initialized\n", i);
				++j;
			}
		}
		if(j==espee_param.nb_sensors_)
			init_sensor_done=true;

		///initialized the EKF object
		if(init_sensor_done)
		{				
			///initialize the EKF object
			std::vector<sensor_info*> sensor_param_list;
			sensor_param_list.reserve(espee_param.nb_sensors_);
			for(int ii=0; ii<espee_param.nb_sensors_; ++ii)
				sensor_param_list.push_back(&Sensor[ii].sensor_param);
			espee_obj= std::make_unique<espee_vo>(ekf_param, espee_param, initmap_param, map_param, sensor_param_list, input_depth_values);
			printf(" espee_vo is initialized \n");
		}
	}
	else
	{
		/// Process the event packet
		if(!td_all.empty())
		{
			
			///check time order of the events
			#ifdef VERBOSE
				static ros::Time temp0, temp1, temp_a;
				for(std::vector<event>::iterator it = td_all.begin(); it != td_all.end(); ++it ) 
				{
					if(it->ts < temp0 || it->ts < temp1)
						ROS_ERROR("wrong order of data, temp = %f, time = %f", temp0.toSec(), it->ts.toSec());
					
					if (it->id_sensor == 0)
					{
						if(it->ts< temp0)
							ROS_ERROR("wrong order of data, temp = %f, time = %f", temp0.toSec(), it->ts.toSec());
						temp0 = it->ts;
					}
					else if(it->id_sensor == 1)
					{
						if(it->ts< temp1)
							ROS_ERROR("wrong order of data, temp = %f, time = %f", temp1.toSec(), it->ts.toSec());
						temp1 = it->ts;
					}
					else
						ROS_ERROR("wrong camera number...");
				}
			#endif
			
			#ifdef ENSURE_POSE_PUB
				event ev_temp = event(0, 0, 0, 0, td_all[0].ts + ros::Duration(PUB_SAMPLINGTIME));
				std::vector<event>::iterator first = td_all.begin();
				std::vector<event>::iterator lb =	std::lower_bound(td_all.begin(), td_all.end(), ev_temp);
				while(td_all.end()!=lb)
				{
					espee_obj->process_batch(first, lb);
					first = lb;
					ev_temp.ts = first->ts + ros::Duration(PUB_SAMPLINGTIME);
					lb =	std::lower_bound(first, td_all.end(), ev_temp);
				
					std::array<float, STATE_SIZE> X_GT;
					get_current_pose(X_GT );
					output_res_obj.Publish_results_inGT(X_GT, espee_param.Xinit_, espee_obj->features_EKF_ptr, (lb-1)->ts);
				}
			#else
				espee_obj->process_batch(td_all.begin(), td_all.end());
				std::array<float, STATE_SIZE> X_GT;
				get_current_pose(X_GT);
				output_res_obj.Publish_results_inGT(X_GT, espee_param.Xinit_, espee_obj->features_EKF_ptr, (td_all.end()-1)->ts);
			#endif


			if(td_all.back().ts - last_display_time > ros::Duration(DISPLAY_INTERVAL) )
			{
				//printf("current time : %f , last display time : %f \n", td_all.back().ts.toSec(), last_display_time.toSec());
				last_display_time =td_all.back().ts;
				for(int n=0; n<espee_param.nb_sensors_;n++)	
				{
					if(map_param.mode_==1)
					{
						display_vec[n].Publish_LUT_withMAP(&Sensor[n].sensor_param, espee_obj->X, espee_obj->X_relative[n], 
																						espee_obj->features_EKF_ptr, td_all.back().ts, espee_obj->Mapping_obj_list[n]);
					}
					else
						display_vec[n].Publish_LUT(&Sensor[n].sensor_param, espee_obj->X, espee_obj->X_relative[n], 
																						espee_obj->features_EKF_ptr, td_all.back().ts);	
				}
			}

			///erase the event vector
			//printf("size event vector %lu \n", td_all.size());
			td_all.clear();
		}
		
		
	}
}

void espee_ros::get_current_pose(std::array<float, STATE_SIZE>& curr_pose)
{
	if(espee_obj != nullptr){
		pose pose_w;

		// ///Position of the camera according to init pose	
		pose_w.p = espee_param.Xinit_.q * (espee_obj->X.q *espee_obj->X.p)+espee_param.Xinit_.p;
		pose_w.q = espee_param.Xinit_.q * espee_obj->X.q;

		curr_pose[0] = pose_w.p.x();
		curr_pose[1] = pose_w.p.y();
		curr_pose[2] = pose_w.p.z();
		curr_pose[3] = pose_w.q.w();
		curr_pose[4] = pose_w.q.x();
		curr_pose[5] = pose_w.q.y();
		curr_pose[6] = pose_w.q.z();
		
	}
	else{
		for(int i=0; i<STATE_SIZE; ++i)
			curr_pose[i] = 0;
		curr_pose[3] =1;
	}

}

output_results_ros::output_results_ros(ros::NodeHandle nh)
{
	///create the publisher for the odometry and pointcloud topics
	odom_publisher = nh.advertise<geometry_msgs::PoseStamped>("/localization/EKF_odom", 10);
	pntcld_publisher = nh.advertise<pcl::PointCloud<pcl::PointXYZ>> ("/localization/Features", 10);
}

void output_results_ros::Publish_results_inGT(std::array<float, STATE_SIZE>& curr_pose, pose& Xinit, pcl::PointCloud<pcl::PointXYZ>::Ptr pc_ptr, ros::Time last_evt_time )
{

	///publish the pose estimation
	geometry_msgs::PoseStamped odom_msg;
	odom_msg.header.stamp = last_evt_time; // use the time of the event to cancel computing delay (depend what you want to show)
	odom_msg.header.frame_id = "world";
	odom_msg.pose.position.x = curr_pose[0];
	odom_msg.pose.position.y = curr_pose[1];
	odom_msg.pose.position.z = curr_pose[2];

	odom_msg.pose.orientation.w = curr_pose[3];
	odom_msg.pose.orientation.x = curr_pose[4];
	odom_msg.pose.orientation.y = curr_pose[5];
	odom_msg.pose.orientation.z = curr_pose[6];
	odom_publisher.publish(odom_msg);
	
	///publish point cloud;
	if(pc_ptr->size() !=0)
	{
		//convert the from camera to world frame
		pcl::PointCloud<pcl::PointXYZ> pcl_msg;
		pcl_msg.reserve(pc_ptr->size());
		Eigen::Vector3f points_in;
		Eigen::Vector3f points_out;
		for(int k=0; k<pc_ptr->size(); k++){
			points_in[0] = pc_ptr->points[k].x;
			points_in[1] = pc_ptr->points[k].y;
			points_in[2] = pc_ptr->points[k].z;
			
			//rotation to vicon frame
			points_out = Xinit.q * points_in;
			//translation to vicon frame	
			points_out+= Xinit.p;

			pcl_msg.points.push_back (pcl::PointXYZ(points_out(0), points_out(1), points_out(2)));
		}

		pcl_msg.header.frame_id = "world";
		pcl_msg.width = pc_ptr->size();
		pcl_msg.height = 1; 		//1 for unorganized data
		pcl_msg.header.stamp = uint64_t(last_evt_time.toSec()*1e6);  //uint64_t(ros::Time::now().toSec()*1e6);
		pntcld_publisher.publish(pcl_msg);				
	}
}

void output_results_ros::Publish_results_raw(const pose& pose_c, pcl::PointCloud<pcl::PointXYZ>::Ptr pc_ptr, ros::Time last_evt_time )
{
	///publish the pose estimation
	geometry_msgs::PoseStamped odom_msg;
	odom_msg.header.stamp = last_evt_time; // use the time of the event to cancel computing delay (depend what you want to show)
	odom_msg.header.frame_id = "world";
	odom_msg.pose.position.x = pose_c.p.x();
	odom_msg.pose.position.y = pose_c.p.y();
	odom_msg.pose.position.z = pose_c.p.z();

	odom_msg.pose.orientation.w = pose_c.q.w();
	odom_msg.pose.orientation.x = pose_c.q.x();
	odom_msg.pose.orientation.y = pose_c.q.y();
	odom_msg.pose.orientation.z = pose_c.q.z();
	odom_publisher.publish(odom_msg);

	///publish point cloud;
	if(pc_ptr->size() !=0)
	{
		//pcl::PointCloud<pcl::PointXYZ> pcl_msg;
		pc_ptr->header.frame_id = "world";
		
		pc_ptr->width = pc_ptr->size();
		pc_ptr->height = 1; 		//1 for unorganized data
		pc_ptr->header.stamp = uint64_t(last_evt_time.toSec()*1e6);  //uint64_t(ros::Time::now().toSec()*1e6);
		pntcld_publisher.publish(pc_ptr);				
	}
}

display_LUT_ros::display_LUT_ros(ros::NodeHandle nh, int idx_sensor, int publish_map, float min_dist, float max_dist)
: colr(1,NUM_COLORS)
{	
	///initialize for publishing LUT	
	LUT_pub = nh.advertise<sensor_msgs::Image> ("/localization/EKF_tracking_Cam" + std::to_string(idx_sensor), 10);
	if(publish_map)
		Mapping_pub = nh.advertise<sensor_msgs::Image> ("/localization/Mapping_Cam" + std::to_string(idx_sensor), 10);

	D_min = min_dist;
	D_max = max_dist;
	depth_scale = colr.NB_colors*D_min/D_max;

}

///publish the LUT image	and the MAP computed 	
void display_LUT_ros::Publish_LUT_withMAP(sensor_info* param, pose& X, pose& X_rel_in, pcl::PointCloud<pcl::PointXYZ>::Ptr pc_ptr,
 																					ros::Time display_time, Mapping_EMVS& map_obj)
{	
	pose X_temp;
	get_state2_from_state1(X, X_rel_in, X_temp);
	show_LUT(param->x_dim, param->y_dim, pc_ptr, X_temp, LUT_pub, param,  display_time);
	
	///mapping display
	if(map_obj.features_ptr->size()!=0){
		//printf("\t Should show mapping cam %d with %lu features \n", n, map_obj.features_ptr->size());
		show_LUT(param->x_dim, param->y_dim, map_obj.features_ptr, X_temp, Mapping_pub, param, display_time);
	}
}

///publish the LUT image		
void display_LUT_ros::Publish_LUT(sensor_info* param, pose& X, pose& X_rel_in,
																  pcl::PointCloud<pcl::PointXYZ>::Ptr pc_ptr, ros::Time display_time)
{	
	
	pose X_temp;
	get_state2_from_state1(X, X_rel_in, X_temp);
	show_LUT(param->x_dim, param->y_dim, pc_ptr, X_temp, LUT_pub, param, display_time);
	
}

//Show the contents of the lookup table
void display_LUT_ros::show_LUT(uint32_t Window_sizeX, uint32_t Window_sizeY, pcl::PointCloud<pcl::PointXYZ>::Ptr features_ptr, 
						pose& X, ros::Publisher& pub_img, sensor_info* param, ros::Time display_time)
{
	int16_t pixel_loc[2];
	float depth;
	int16_t evt_x, evt_y;

	uint32_t EKF_pixels[Window_sizeX*Window_sizeY];
	// clear the old table
	for(int y=0; y<Window_sizeY; y++)
		for(int x=0; x<Window_sizeX; x++)
			EKF_pixels[y * Window_sizeX + x] = colr.OFFcolor;
	
	//printf("depth scale is %f, num_colors is %i, D_min is %f, D_max is %f\n", depth_scale, colr.NB_colors, D_min, D_max);
	for(int feature_num = 0; feature_num < features_ptr->size(); ++feature_num )  
	{
		//printf("projecting feature %i sensor 1\n", feature_num);
		Eigen::Vector3f temp3Dpoint = Eigen::Vector3f(features_ptr->points[feature_num].x, features_ptr->points[feature_num].y, features_ptr->points[feature_num].z );
		depth = param->project(temp3Dpoint, X.p, X.q, pixel_loc);
		evt_x = pixel_loc[0];
		evt_y = pixel_loc[1];
		if ((evt_x>=0) & (evt_y>=0) & (evt_x<Window_sizeX) & (evt_y<Window_sizeY))
		{
			//uint32_t idx_depth = uint32_t (round((depth-D_min)*100));
			uint32_t idx_depth = (D_max/depth)*depth_scale;
			if(idx_depth <0)
				//printf("error in depth colors : idx_depth = %d, depth = %f, default idx value 0 used instead \n", idx_depth, depth);
				idx_depth = 0;
			else if(idx_depth>= colr.NB_colors){
				//printf("error in depth colors : idx_depth = %d, depth = %f, default idx value %d used instead \n", idx_depth, depth, colr.NB_colors-1);
				idx_depth = colr.NB_colors-1;
			}
			//printf("depth is %f, index is %i\n", depth, idx_depth);
			EKF_pixels[evt_y * Window_sizeX + evt_x] = colr.Colors_val[idx_depth];			//ONcolor; //
		}
	}

	//printf("displaying %s\n", EKF_display.window_name.c_str());

	///publish image in topic
	sensor_msgs::Image output_image;
	output_image.header.stamp     = display_time;  	//ros::Time::now();
	output_image.height           = Window_sizeY;
	output_image.width            = Window_sizeX;
	output_image.encoding         = "rgb8";
	output_image.is_bigendian     = false;
	output_image.step             = 3 * Window_sizeX;

	for(int i=0; i<(Window_sizeX*Window_sizeY);i++)
	{
	   output_image.data.push_back(uint8_t(EKF_pixels[i]>>16));
	   output_image.data.push_back(uint8_t((EKF_pixels[i] & 0x0000FF00)>>8));
	   output_image.data.push_back(uint8_t(EKF_pixels[i] & 0x000000FF));
	}
	
	pub_img.publish(output_image);
}

}
