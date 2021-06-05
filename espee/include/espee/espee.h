#ifndef ESPEE_H
#define ESPEE_H

#include <cstring>

///non standard library
#include "espee/utilities.h"
#include "espee/mapping.h"

#include <Eigen/Dense>

//For debugging purposes:
//#define write_outputs //--- enables an option to write to file. This is checked on every loop iteration, and is therefore slightly detrimental to performance, even when no writing is needed. i.e. for final optimization, comment this out

//Point cloud initialization
#define NUM_INIT_POINTS_DEFAULT 2000 // default number of points per sensor to initialize point cloud with

#define EKF_STATE_SIZE 6

namespace espee_ns {

/* EKF param structure */
struct EKF_param_struct{
	double Ptrans_init_; 	//default 0.001 
	double Prot_init_; 	//default 0.05
	double QtransXY_init_;	//dual camera default val = Ptrans_init/50 = 2e-8;  one camera default val = 5e-9;  
	double QtransZ_init_;	//dual camera default val = Ptrans_init/50 = 2e-8;  one camera default val = 5e-9 or 1e-10;
	double QrotXY_init_;	//dual camera default val = Prot_init/50 = 5e-5;  one camera default val = 1e-4;
	double QrotZ_init_;	//dual camera default val = Prot_init/50 = 5e-5;  one camera default val = 1e-4;
	double R_coeff_;  		//dual camera default val = 200;  one camera default val = 9 or 25; 
};

// ESPEE param structure
struct ESPEE_param_struct{
	int Edge_range_ = 5; 					// how wide a region should be considered when searching for matching features/edges. The region is from -edgeRange to edgeRange (of sidelength 2*edgeRange+1)
	int Remap_Time_Threshold_; 	//recompute the LUT after this many microseconds
	int nb_sensors_;
	std::vector<pose> Xrelative_;  //relative pose from sensor to rig frame (assume cam 0 for now)
	pose Xinit_;  // init pose of the rig
};

// Init structure
struct Init_param_struct{
	int Mode_;  			//0: init param with planar scene and accumulate events, 1: init with PCD file
	int Num_points_;		//average number of points per sensors to initialize with (mode 0)
  std::string path2PCD_; //path to point cloud (mode 1)
	pose Pose_PCDrelative_;  //relative pose of the PCD
};

typedef Eigen::Array<int,Eigen::Dynamic,Eigen::Dynamic> edge_map_type;  //used in init_features
typedef Eigen::Array<float,Eigen::Dynamic,Eigen::Dynamic> Array_LUT;  //type used to store the LUT (depth if a pixel is occupied)

class EKF_BE
{
	private:
		#ifdef WRITE_EKFSTATE_FILE
			std::ofstream output_file;
		#endif

		// The EKF variables 
		Eigen::Matrix<float, EKF_STATE_SIZE, EKF_STATE_SIZE> P; //the covariance of the EKF
		Eigen::Matrix<float, EKF_STATE_SIZE, EKF_STATE_SIZE> Q; 				 //process noise
		std::vector< Eigen::Matrix<float, 2, 2> > R; 				 //measurement noise

		std::vector< Eigen::Matrix<float, EKF_STATE_SIZE, EKF_STATE_SIZE> > T_dh;

		//two main steps of the EKF		
		void predict();
		void update(const Eigen::Vector3f& z, const Eigen::Vector3f& h, float depth, 
								int n_sensor, Eigen::Ref<Eigen::Matrix<float,EKF_STATE_SIZE,1>> DeltaX_EKF);
		
		//calculate the Jacobian H
		void dh_func(const Eigen::Vector3f& h, float depth, Eigen::Ref<Eigen::Matrix<float, 2, EKF_STATE_SIZE>> H_in);

	public:
		void process_ekf(const Eigen::Vector3f& z, const Eigen::Vector3f& h, float depth, pose& X, int sensor);

		EKF_BE(EKF_param_struct ekf_param, ESPEE_param_struct rig_param, const std::vector<sensor_info*>& info_sensors_ptr);
		~EKF_BE();
};

class espee_fe
{
private:
	int Edge_range;

	int32_t LUT_max_x, LUT_max_y; 
	Array_LUT edge_LUT; //a 2d array.  each x by y

public:
	#ifdef WRITE_LUT1
		bool First_LUT=true;
	#endif
	void SaveLUT(int n_sensor);
	bool match_feature(int16_t* pix_in, int16_t* pix_out, float* depth_pointer);
	void remake_LUT(pcl::PointCloud<pcl::PointXYZ>::Ptr features_ptr, const pose& X);	

	sensor_info* camera_info;   // parameters defining the mapping from pixels to image plan co-ordinates

	espee_fe(ESPEE_param_struct espee_param, sensor_info* info_sensors_ptr);
	~espee_fe();
};


class espee_vo
{
	private:
		uint64_t LUT_update_period;   //period of update of the LUT
		uint64_t last_time = 0;		//store the last LUT update time
		int nb_sensors;   //Store the number of sensors used

		void Multithreaded_remake_LUT(pcl::PointCloud<pcl::PointXYZ>::Ptr features_ptr, const pose& X, int n_sensor);

		std::vector<espee_fe> all_fe;   //object doing feature matching for each sensor 

		EKF_BE ekf_computation_obj;
		
		#ifdef WRITE_EVENT_FILE
			std::ofstream event_file;
		#endif
		
		/// Initialization of the map 
		bool PC_initialized = false; 
		int NB_points2get;	//number of points to acquire in the initial map (in mode 0)

    bool init_features_PCD(pcl::PointCloud<pcl::PointXYZ>::Ptr features_ptr, std::string& filepath, 
													 pose& relpose, pose& init);

		//initial assumed depth of the scene from each sensor.
		std::vector<float> init_depth_values;
		bool init_features_planar(pcl::PointCloud<pcl::PointXYZ>::Ptr features_ptr, 
											 event& evt_it);

		///variables and functions for mapping
		int mapping_mode;
		std::function<void(event& evt)> process_Mapping;
		void Mapping_DoNothing(event& evt);
		void Mapping_EMVSonly(event& evt);
		void Mapping_EMVScloseloop(event& evt);

		//Processing events functions
		bool Process_fe(int16_t* pix_in, uint64_t time_evt, int sensor, int16_t* pix_out, float* depth_pointer, bool* update_LUT);
		bool process_event(event& evt_it);

	public:	
		// State of the primary sensor. All other sensors are defined relative to this one
		pose X;
		std::vector<pose> X_relative;	 //an array holding the relative position between each sensor and the primary sensor. X_relative[0] will be all zeros to indicate that sensor 0 has 0 offset from the system state
		
		pcl::PointCloud<pcl::PointXYZ>::Ptr features_EKF_ptr; // Store the 3D features list used by espee 
		std::vector<Mapping_EMVS> Mapping_obj_list;

		espee_vo(EKF_param_struct ekf_param, ESPEE_param_struct espee_param, Init_param_struct init_param,
		         Mapping_param_struct map_param, const std::vector<sensor_info*>& info_sensors_ptr, std::vector<float>& init_depths);
		

		~espee_vo();		//destructor
		
		void process_batch(std::vector<event>::iterator it_begin, std::vector<event>::iterator it_end);
};

}

#endif
