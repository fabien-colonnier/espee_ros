#include "espee/espee.h"
#include <pcl/io/pcd_io.h>

namespace espee_ns {
EKF_BE::EKF_BE(EKF_param_struct ekf_param, ESPEE_param_struct rig_param, const std::vector<sensor_info*>& info_sensors_ptr)
{
	#ifdef WRITE_EKFSTATE_FILE
		output_file.open("ekfstate.txt", std::ios::out);
		output_file << "x \t y \t z \t qw \t qx \t qy \t qz \n";
	#endif

	//------------------- initialize P
	float PtraXYZ = ekf_param.Ptrans_init_*ekf_param.Ptrans_init_;
	float ProtXYZ = (ekf_param.Prot_init_*PI/180)*(ekf_param.Prot_init_*PI/180);

	P << PtraXYZ, 0, 0, 0, 0, 0,
			 0, PtraXYZ, 0, 0, 0, 0,
			 0, 0, PtraXYZ, 0, 0, 0,
			 0, 0, 0, ProtXYZ, 0, 0,
			 0, 0, 0, 0, ProtXYZ, 0,
			 0, 0, 0, 0, 0, ProtXYZ;


	//------------------- initialize Q
  float QtraXY = ekf_param.QtransXY_init_;
	float QtraZ = ekf_param.QtransZ_init_;
	float QrotXY = ekf_param.QrotXY_init_*PI/180*PI/180;;
	float QrotZ = ekf_param.QrotZ_init_*PI/180*PI/180;

	Q << QtraXY, 0, 0, 0, 0, 0,
			 0, QtraXY, 0, 0, 0, 0,
			 0, 0, QtraZ, 0, 0, 0,
			 0, 0, 0, QrotXY, 0, 0,
			 0, 0, 0, 0, QrotXY, 0,
			 0, 0, 0, 0, 0, QrotZ;

	printf("Q initialized\n");

	//------------------- initialize R
	//R = ((pix/f).*eye(2)).^2; %covariance of measurement noise in measurement units (pixels)
	//a larger diagonal R means the diagonal of S = R+HPH' is larger, which makes inv_S smaller, which reduces the Kalman Gain K
	
	R.reserve(rig_param.nb_sensors_);
	for(int n_sensor=0; n_sensor<rig_param.nb_sensors_; ++n_sensor)
	{
		Eigen::Matrix<float, 2, 2> Rtemp;
		Rtemp << ekf_param.R_coeff_/(info_sensors_ptr[n_sensor]->fxp*info_sensors_ptr[n_sensor]->fxp), 0,
						 0, ekf_param.R_coeff_/(info_sensors_ptr[n_sensor]->fyp*info_sensors_ptr[n_sensor]->fyp);
		R.push_back(Rtemp); 
	}
	printf("R initialized\n");

	//------------------- initialize T_dh transformation of the H matrix
	for(int n_sensor=0; n_sensor<rig_param.nb_sensors_; ++n_sensor)
	{
		Eigen::Matrix<float, 3, 3> R_rot = rig_param.Xrelative_[n_sensor].q.normalized().toRotationMatrix();
		Eigen::Matrix<float, 3, 3> T_skew;
		vec2skew(rig_param.Xrelative_[n_sensor].p, T_skew);

		Eigen::Matrix<float, 6, 6> T_dh_in;
		T_dh_in.block<3,3>(0,0) = R_rot;
		T_dh_in.block<3,3>(3,3) = R_rot;
		T_dh_in.block<3,3>(0,3) = R_rot*T_skew;
		T_dh_in.block<3,3>(3,0) = Eigen::Matrix<float, 3, 3>::Zero();
	
		T_dh.push_back(T_dh_in);
		std::cout << "T_dh(" << n_sensor << ") = \n" << T_dh[n_sensor] << "\n";
	}
	printf("T_dh initialized\n");

}

EKF_BE::~EKF_BE()
{
	//Destructor, add value if necessary
}

espee_vo::espee_vo(EKF_param_struct ekf_param, ESPEE_param_struct espee_param, Init_param_struct init_param,
  Mapping_param_struct map_param, const std::vector<sensor_info*>& info_sensors_ptr, std::vector<float>& init_depths)
: ekf_computation_obj(ekf_param, espee_param, info_sensors_ptr),
 features_EKF_ptr(new pcl::PointCloud<pcl::PointXYZ>),
 init_depth_values(init_depths),
 nb_sensors(espee_param.nb_sensors_)
{
	#ifdef WRITE_EVENT_FILE
		//writing the events used for localization
		event_file.open("event_list_processed_ros.txt", std::ios::out);
		event_file << "id_sensor, time,  x, y, polarity" << std::endl;
	#endif

	//------------------- Init point cloud params
	if(init_param.Mode_==1)
	{
	  PC_initialized = init_features_PCD(features_EKF_ptr,
		                                   init_param.path2PCD_, init_param.Pose_PCDrelative_, espee_param.Xinit_);
		if(!PC_initialized)
		{
			init_param.Mode_=0;
			std::cout << "[ESPEE_VO] wrong PCD initialization, planar assumption will be used" << std::endl;
		}
	}
	if(init_param.Mode_!=1)
	{
		NB_points2get = init_param.Num_points_ * nb_sensors;

  	//reserve memory space for the point cloud
		features_EKF_ptr->reserve( NB_points2get ); 
	}	

	//------------------- Initialization of the MAPPING object
	mapping_mode = map_param.mode_;
	if(map_param.mode_ == 0){
		process_Mapping =  std::bind( &espee_vo::Mapping_DoNothing, this, std::placeholders::_1 );
	}
	else{
		Mapping_obj_list.reserve(nb_sensors);	
		for(int n=0; n<nb_sensors; ++n){
			Mapping_obj_list.emplace_back(map_param, *info_sensors_ptr[n], init_depth_values.at(n));
			printf("\t initialization mapping cam %d with %lu features \n", n, Mapping_obj_list.at(n).features_ptr->size());
		}

		switch(map_param.mode_){
			case 1: 
				process_Mapping =  std::bind( &espee_vo::Mapping_EMVSonly, this, std::placeholders::_1);
				break;
			case 2:
				process_Mapping =  std::bind( &espee_vo::Mapping_EMVScloseloop, this, std::placeholders::_1);
				break;
			default:
				process_Mapping =  std::bind( &espee_vo::Mapping_DoNothing, this, std::placeholders::_1);
		}

	}
	

	//------------------- initialize Front end
	all_fe.reserve(nb_sensors);
	for(int i=0; i<nb_sensors; ++i)
	{
		all_fe.emplace_back(espee_param, info_sensors_ptr[i]);
		printf("Camera %d, parameters: fxp = %f , cx = %f, cy = %f \n", i, all_fe.at(i).camera_info->fxp, all_fe.at(i).camera_info->cxp, all_fe.at(i).camera_info->cyp); 
	}

	LUT_update_period = espee_param.Remap_Time_Threshold_;

	//------------------- initialize X (the sensor/rig state)
	X.p = Eigen::Vector3f::Zero();
	X.q = Eigen::Quaternionf(1,0,0,0);

	//----------------- what is the relative pose between each of the sensors in our system?
	X_relative.reserve(nb_sensors);
	pose temp;
	for(int n_sensor=0; n_sensor<nb_sensors; n_sensor++)
	{
		// temp.p = Eigen::Vector3f(espee_param.Xrelative_[n_sensor][0],
		// 												 espee_param.Xrelative_[n_sensor][1],
		// 												 espee_param.Xrelative_[n_sensor][2]);
		// temp.q = Eigen::Quaternionf(espee_param.Xrelative_[n_sensor][3],
		// 												 espee_param.Xrelative_[n_sensor][4],
		// 												 espee_param.Xrelative_[n_sensor][5],
		// 												 espee_param.Xrelative_[n_sensor][6]);
		X_relative.push_back(espee_param.Xrelative_[n_sensor]);
	}
	
}

espee_vo::~espee_vo()
{
	features_EKF_ptr.reset();
	std::cout <<"EKF object destroyed\n";
}

bool espee_vo::init_features_PCD(pcl::PointCloud<pcl::PointXYZ>::Ptr features_ptr, std::string& filepath, 
                                 pose& relpose, pose& init)
{
	/* READ the PCD file  */
	pcl::PointCloud<pcl::PointXYZI>	cloud;  //temporary cloud as there is 
  int pcd_version;
	int	data_type;
	unsigned int data_idx;
	const int offset = 0; 

	std::cout << "enter init point cloud";

	if (pcl::io::loadPCDFile<pcl::PointXYZI> (filepath, cloud) == -1) //* load the file
  {
    PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
    return false;
  }
  std::cout << "Loaded "
            << cloud.width * cloud.height
            << " data points from test_pcd.pcd with the following fields: "
            << std::endl;

	Eigen::Quaternionf quat, quat_init, quat_PCD;
	// quat_init = Eigen::Quaternionf(init[3], init[4], init[5], init[6]);
	// quat_PCD = Eigen::Quaternionf(relpose[3], relpose[4], relpose[5], relpose[6]);
	quat = init.q.inverse() * relpose.q;

	Eigen::Vector3f temp_pt;
  for (size_t i = 0; i < cloud.points.size (); ++i)
	{
		Eigen::Vector3f in_pt(cloud.points[i].x, cloud.points[i].y, cloud.points[i].z);
		temp_pt = quat*in_pt;
		temp_pt += relpose.p-init.p;
		features_ptr->push_back (pcl::PointXYZ (temp_pt(0), temp_pt(1), temp_pt(2))); 
	}

	return true;
}

//build a list of the first map with the events to a predefined depth
bool espee_vo::init_features_planar(pcl::PointCloud<pcl::PointXYZ>::Ptr features_ptr, event& evt_it)
{
	//Dynamic allocation is needed as the size of the frame is not known at compilation time
	static std::vector<edge_map_type> edges; //boolean edge map keep track of already mapped pixels
	static bool edges_initialized = false;
	static int num_initialized_points = 0;

	int sensor = evt_it.id_sensor;

	/// initializing edges
	if(!edges_initialized){
		edges.reserve(nb_sensors);
		for(int n=0; n<nb_sensors; n++)
		{
			edges.emplace_back(edge_map_type::Zero(all_fe[n].camera_info->y_dim,all_fe[n].camera_info->x_dim));
		}
		edges_initialized=true;
	}


	if(!edges[sensor](evt_it.y,evt_it.x)) //if there is not already an edge at this location
	{
		edges[sensor](evt_it.y,evt_it.x) = 1; //mark the edge as existing in the edge map
		int16_t pix[2] = {(int16_t) evt_it.x, (int16_t) evt_it.y};

		// add the feature's 3D location to the feature list
		Eigen::Vector3f temp3Dpoint;
		all_fe.at(sensor).camera_info->pixel_to_camera(pix, temp3Dpoint);
		temp3Dpoint = temp3Dpoint*init_depth_values[sensor];
		
		Eigen::Vector3f temp3Dpoint2;
		pose X_temp;
		get_state2_from_state1(X, X_relative[sensor], X_temp);
		all_fe[sensor].camera_info->camera_to_world(temp3Dpoint, X_temp.p, X_temp.q, temp3Dpoint2);
		
		if(mapping_mode!=0){	
			//create the point into the mapping feature list (as it was coming from the mapping )
			Mapping_obj_list[sensor].features_ptr->push_back(pcl::PointXYZ (temp3Dpoint2(0), temp3Dpoint2(1), temp3Dpoint2(2))); 
		}
		else
			features_ptr->push_back (pcl::PointXYZ (temp3Dpoint2(0), temp3Dpoint2(1), temp3Dpoint2(2))); 


		//check if we have enough features to begin localization
		num_initialized_points++;
		if(num_initialized_points>=NB_points2get)
		{
			if(mapping_mode!=0)
			{
				for(int n=0;n<nb_sensors;n++)
				{
					*features_ptr += *(Mapping_obj_list[n].features_ptr);
				}
			}


			printf("Enough points acquired ... starting Localization algorithm\n");
			return true;
		}			
	}
	return false;
}

void espee_vo::Mapping_DoNothing(event& evt)
{
}

void espee_vo::Mapping_EMVSonly(event& evt)
{
	int sensor = evt.id_sensor;	
	pose X_temp;
	get_state2_from_state1(X, X_relative[sensor], X_temp);
	Mapping_obj_list[sensor].Process_event(evt, X_temp);
}

void espee_vo::Mapping_EMVScloseloop(event& evt)
{
	espee_vo::Mapping_EMVSonly(evt);

	///check if one sensor has an updated feature list
	bool need_update = false;
	for(int n_sensor=0; n_sensor<nb_sensors; ++n_sensor)
		if(Mapping_obj_list[n_sensor].new_features_extracted == true)
		{
			need_update = true;
		}	
	
	///updated the feature list if needed
	if(need_update)
	{
		printf("copying feature list into EKF one\n");
		features_EKF_ptr->clear();  //erase the old
		for(int n_sensor=0; n_sensor<nb_sensors; ++n_sensor){
			*features_EKF_ptr += *(Mapping_obj_list[n_sensor].features_ptr);
			Mapping_obj_list[n_sensor].new_features_extracted = false;
		}
		
		need_update = false;
	}
}

void espee_vo::process_batch(std::vector<event>::iterator it_begin, std::vector<event>::iterator it_end)
{
	///compute EKF and mapping			
	for(std::vector<event>::iterator it = it_begin; it != it_end; ++it ) 
	{
		if(!PC_initialized)
		{
			////acquisition of the initial point cloud
			PC_initialized = init_features_planar(features_EKF_ptr, *it);
		}
		else
		{
			///process MAPPING  with previous pose
			process_Mapping(*it);

			//printf(" ekf_ computation\n");
			process_event(*it);		//processing the events
		}
	}
}

bool espee_vo::process_event( event& evt_it)
{
	uint64_t time;
	
	int16_t z_pix[2];
	int16_t h_pix[2];
	float depth;
	int sensor = evt_it.id_sensor;

	#ifdef WRITE_EVENT_FILE
		//writing the events used for localization
		event_file << (evt_it.id_sensor) << ", " << evt_it.ts << ", " << evt_it.x << ", " << evt_it.y << ", " << (evt_it.polarity ? "off":"on") << std::endl;
	#endif

	bool feature_matched = 0;
	if(sensor<nb_sensors)
	{
		time = uint64_t(evt_it.ts.toSec()*1e6);

		///event location in pixel co-ordinates
		z_pix[0] = evt_it.x;
		z_pix[1] = evt_it.y;

		bool LUT_update = false;
		feature_matched = Process_fe(z_pix, time, sensor, h_pix, &depth, &LUT_update);
				
		if(feature_matched) //if a feature was found...
		{
			Eigen::Vector3f z, h;

			// projection in the focal plane
			all_fe.at(sensor).camera_info->pixel_to_camera(z_pix, z); //convert the pixel co-ordinates (z_pix) to image plane co-ordinates (z)
			all_fe.at(sensor).camera_info->pixel_to_camera(h_pix, h); //convert the pixel co-ordinates (h_pix) to image plane co-ordinates (h)

			//computation of the EKF
			ekf_computation_obj.process_ekf(z, h, depth, X, sensor);
	
		}
	}

	return true;
}

bool espee_vo::Process_fe(int16_t* pix_in, uint64_t time_evt, int sensor, int16_t* pix_out, float* depth_pointer, bool* update_LUT)
{		
	//printf("initialized, processing events\n");
	if(time_evt - last_time > LUT_update_period) //Repopulate LUT if it is old (>param.Remap_Time_Threshold)
	{					
		*update_LUT = true;

		///perform remake_LUT for the remaining sensors [1 ... NB_SENSORS] in other threads
		std::vector<std::thread> thread_LUT;
		//Launch a group of threads
		thread_LUT.reserve(nb_sensors);
		for(int n=1;n<nb_sensors;n++)
		{
			thread_LUT.emplace_back(&espee_vo::Multithreaded_remake_LUT, this, features_EKF_ptr, X, n);				
		}
		
		///perform remake_LUT for sensor 0 (in the main thread)
		pose X_temp;
		get_state2_from_state1(X, X_relative[0], X_temp);  

		all_fe.at(0).remake_LUT(features_EKF_ptr, X);  //TODO replace with X_temp
		
		#ifdef WRITE_LUT1
			if(all_fe[0].First_LUT)
			{
				all_fe[0].SaveLUT(0);
				all_fe[0].First_LUT = false;
			}
		#endif

		//Join the threads with the main thread
		for(int n=0;n<nb_sensors-1;n++) {
			thread_LUT[n].join();
		}
		
		///update last time
		last_time = time_evt;
	}
	else
		*update_LUT = false;

	//Given an event pixel location (z_pix), calculate the pixel location of the nearest feature (h_pix), and its depth in camera co-ordinates (depth). Return whether a matching feature was found
	bool feature_matched = all_fe.at(sensor).match_feature(pix_in, pix_out, depth_pointer);	
	
	return feature_matched;
}

	
void espee_fe::SaveLUT(int n_sensor){
	std::ofstream LUT_file;

	LUT_file.open("first_LUT" + std::to_string(n_sensor) + ".txt", std::ios::out);
	for(int y=0; y<LUT_max_y; y++){
		for(int x=0; x<LUT_max_x; x++)
			LUT_file << edge_LUT(y,x) << ", ";
		LUT_file << std::endl;
	}		
}


void espee_vo::Multithreaded_remake_LUT(pcl::PointCloud<pcl::PointXYZ>::Ptr features_ptr, const pose& X, int n_sensor)
{
	pose X_temp;
	get_state2_from_state1(X, X_relative[n_sensor], X_temp);
	all_fe[n_sensor].remake_LUT(features_ptr, X_temp);

	#ifdef WRITE_LUT1
		if(all_fe[n_sensor].First_LUT)
		{
			all_fe[n_sensor].SaveLUT(n_sensor);
			all_fe[n_sensor].First_LUT = false;
		}
	#endif

}

espee_fe::espee_fe(ESPEE_param_struct espee_param, sensor_info* info_sensors_ptr)
: camera_info(info_sensors_ptr)
{
	Edge_range = espee_param.Edge_range_;

	//------------------- initialize the edge lookup table 
	LUT_max_x = camera_info->x_dim;
	LUT_max_y = camera_info->y_dim;
	printf("LUT_size: LUT max x %i, LUT max y %i\n", LUT_max_x, LUT_max_y);

	edge_LUT = Array_LUT::Zero(LUT_max_y, LUT_max_x);
}

espee_fe::~espee_fe()
{
	//Destructor, add value if necessary
}

//given a pixel location (pix_in), this function will search through a nearby region in a LUT to find possible matching features. It will return the pixel location of the feature (pix_out) and its depth (depth_pointer), as well as a boolean indicating whether a feature was found
bool espee_fe::match_feature(int16_t* pix_in, int16_t* pix_out, float* depth_pointer)
{
	// need to randomize this function so that it is not biased towards a particular direction.
	// Currently it searches starting from the -x -y direction. If multiple features are found at the same distance, it will use the first one, which biases the setup towards -x -y
	*depth_pointer = 0; 

	int16_t min_dist_squared = 30e3;		//very high value
	int16_t dist_squared;

	int16_t x_min, x_max, y_min, y_max;

	x_min = std::max(0			        , pix_in[0]-Edge_range)-pix_in[0];
	x_max = std::min(LUT_max_x, pix_in[0]+Edge_range+1	)-pix_in[0];

	y_min = std::max(0			        , pix_in[1]-Edge_range	)-pix_in[1];
	y_max = std::min(LUT_max_y, pix_in[1]+Edge_range+1	)-pix_in[1];
		
	for(int y_loop = y_min; y_loop< y_max; y_loop++)
		for(int x_loop = x_min; x_loop< x_max; x_loop++)
			if(edge_LUT(pix_in[1]+y_loop,pix_in[0]+x_loop)!=0)
			{
				dist_squared = x_loop*x_loop + y_loop*y_loop;
				if(dist_squared<min_dist_squared) //this will sometimes result in throwing away the only feature found
				{
					min_dist_squared = dist_squared;
					pix_out[0] = pix_in[0]+x_loop;
					pix_out[1] = pix_in[1]+y_loop;
					*depth_pointer = edge_LUT(pix_out[1],pix_out[0]);
				}else
				if((dist_squared==min_dist_squared)& (rand()>RAND_MAX/2))
				{
					// COULD CHOOSE THE CLOSEST ONE INSTEAD OF BEING RANDOM
					pix_out[0] = pix_in[0]+x_loop;
					pix_out[1] = pix_in[1]+y_loop;
					*depth_pointer = edge_LUT(pix_out[1],pix_out[0]);
				}
			}

	if(*depth_pointer==0)
		return false;

	return true;
}

//populate a lookup table of world features projected onto the image plane, and their depth
void espee_fe::remake_LUT(pcl::PointCloud<pcl::PointXYZ>::Ptr features_ptr, const pose& X)
{
	int16_t pixel_loc[2];
	float depth;
	int16_t evt_x, evt_y;
	
	// clear the old table
	for(int y=0; y<LUT_max_y; y++)
		for(int x=0; x<LUT_max_x; x++)
			edge_LUT(y,x) = 0;
			
	Eigen::Vector3f temp3Dpoint;
	for(int feature_num = 0; feature_num < features_ptr->size(); ++feature_num )
	{
		//printf("projecting feature %i. X is %.2f %.2f %.2f %.2f %.2f %.2f %.2f...", feature_num, X[0]);
		temp3Dpoint(0) = features_ptr->points[feature_num].x;
		temp3Dpoint(1) = features_ptr->points[feature_num].y;
		temp3Dpoint(2) = features_ptr->points[feature_num].z; 
		depth = camera_info->project(temp3Dpoint, X.p, X.q, pixel_loc);
		
		//printf("result is at %i %i and depth %f\n", pixel_loc[0], pixel_loc[1], depth);
		evt_x = pixel_loc[0];
		evt_y = pixel_loc[1];
		if ((evt_x>=0) & (evt_y>=0) & (evt_x<LUT_max_x) & (evt_y<LUT_max_y))
			if (edge_LUT(evt_y,evt_x) ==0 )
				edge_LUT(evt_y,evt_x) = depth;
	}
}

void EKF_BE::process_ekf(const Eigen::Vector3f& z, const Eigen::Vector3f& h, float depth, pose& X, int sensor)
{
	Eigen::Matrix<float, EKF_STATE_SIZE, 1> DeltaX_EKF;

	//first assumption not matched event are considered noise so no update of covariance (not true if incomplete map)
	// perhaps prediction should run on every event (even those without matched features) since it causes uncertainty to grow. Might help latch back on to correct features
	predict(); //	P = P + Q; 

	update(z, h, depth, sensor, DeltaX_EKF);

	//update Pose according to EKF computation
	X.p += Eigen::Vector3f(DeltaX_EKF(0,0), DeltaX_EKF(1,0), DeltaX_EKF(2,0));

	Eigen::Vector3f vec = Eigen::Vector3f(DeltaX_EKF(3,0), DeltaX_EKF(4,0), DeltaX_EKF(5,0));
	Eigen::Quaternionf quat_delta;
	vec2quat(vec, quat_delta);
	X.q = quat_delta * X.q;
	
	#ifdef WRITE_EKFSTATE_FILE
		//optionally write outputs to a file
		output_file << X.p.x() << "\t" << X.p.y() << "\t" << X.p.z() << "\t" 
								<< X.q.w() << "\t" << X.q.x() << "\t" << X.q.y() << "\t" << X.q.z() << "\n";
	#endif
}

void EKF_BE::update(const Eigen::Vector3f& z, const Eigen::Vector3f& h, float depth, int n_sensor, Eigen::Ref<Eigen::Matrix<float,EKF_STATE_SIZE,1>> DeltaX_EKF)
{

//	printf("[EKF_BE] start update \n");

	//Extended Kalman Filter intermediate variables
	Eigen::Matrix<float, 2, EKF_STATE_SIZE> H;
	Eigen::Matrix<float, 2, 2> S;
	Eigen::Matrix<float, 2, 2> inv_S;
	Eigen::Matrix<float, EKF_STATE_SIZE, 2> K;
	Eigen::Matrix<float, 2, EKF_STATE_SIZE> HP;

	Eigen::Matrix<float, 2, 1> y;    //vector 2Row, 1 column

	dh_func(h, depth, H); //find the derivative of (h) with respect to (X). H = dh/dX
	
	//Transform H to express it in the main frame
	//H_a = T_dh[n_sensor] * H_new.transpose();
	//H_new = H_a.transpose();
	H = H*T_dh[n_sensor].transpose();

//	printf("[EKF_BE] compute y \n");
	y << z(0)-h(0), 
			 z(1)-h(1);
	
//	printf("[EKF_BE] compute HP \n");
	HP = H*P;

//	printf("[EKF_BE] compute S \n");
	//S = H*PHt+R;
	//  = H*(HP')+R
	S = H * HP.transpose() + R.at(n_sensor);

	//S^-1
	inv_S = S.inverse();

//	printf("[EKF_BE] compute K \n");
	//K = PHt*inv_S;
	//  = (HP)t * inv_S;
	K = HP.transpose() * inv_S;

//	printf("[EKF_BE] compute X \n");
	DeltaX_EKF = K*y;
	

//	printf("[EKF_BE] compute P new \n");
	P = P-K*HP;

	//printf("[EKF_BE] exit update \n");
}


//given a location on the image plane at unit focal length (h) calculate how the location (h) will change with respect to the camera pose. i.e. H = dh/dX
void EKF_BE::dh_func(const Eigen::Vector3f& h, float depth, Eigen::Ref<Eigen::Matrix<float, 2, EKF_STATE_SIZE>> H_in)
{

//H = [-1/depth	, 0			, u/depth, u*v,  -(1+u^2), v;...
//      0		, -1/depth	, v/depth, 1+v^2, -u*v, -u];
	float inverse_depth = 1/depth;
	H_in(0,0) = -inverse_depth;
	H_in(0,1) = 0;
	H_in(0,2) = h(0)*inverse_depth;
	H_in(0,3) = h(0)*h(1);
	H_in(0,4) = -1-h(0)*h(0);
	H_in(0,5) = h(1);

	H_in(1,0) = 0;
	H_in(1,1) = -inverse_depth;
	H_in(1,2) = h(1)*inverse_depth;
	H_in(1,3) = 1+h(1)*h(1);
	H_in(1,4) = -h(0)*h(1);
	H_in(1,5) = -h(0);
}

// P = P+Q. This is the prediction step for the EKF. Q is diagonal, so we just store it as a 1 dimensional vector
void EKF_BE::predict()
{
	P += Q;
}

}
