#ifndef MAPPING_H
#define MAPPING_H

///non standard library
#include "espee/utilities.h"
#include "espee/convolution.h"

///ROS library
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>

///for debugging
//#define VERBOSE_MAPPING    //----prints initialization param and function execution
//#define VERBOSE_DSIup  //----prints results for each DSI update
//#define VERBOSE_Extract_features	//----prints results of features extraction
//#define VERBOSE_create_features  //----prints each feature extracted position
//#define SAVE_DSI
//#define SAVE_POINTCLOUD

///parameters for the feature list
#define Discard_Old_Features // if defined Update features list and discard all previous data. A buffer is used otherwise
#define NB_MAX_FEATURES 7500 		//numbers of features max in feature list  //To improve: need a case where this value is reached and check if still working
/**** ***************************** *****/

namespace espee_ns{

typedef Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> RowMatrixXi;

/// Parameters intialization
struct Mapping_param_struct{
	int mode_;  //0= No mapping is performed; 1= EMVS is computed but not used for localization; 2= EMVS is computed and used for localization;

	//depth sampling
	double D_min_mapping_; 		//closest plane distance
	double D_max_mapping_;  		//furthest plane distance
	double dz_min_;   				//closest distance between 2 planes (the 2 closest)
	int Nz_; 									// number of depth planes in DSI
	
	//criteria for creating a new keyframe
	double Percent_Dmean_1_;     		//percentage of the mean depth for the DSI update
	double Percent_Dmean_2_;    		//percentage of the mean depth for the DSI update
	int max_event_per_DSI_;  	//maximum of event for a new DSI to be created

	//Parameters for thresholding confidence map (like EVO algorithm)
	int Dim_Gauss_kernel_; 			//size of the Gaussian kernel for the filtering of Cmap  //init_mapping should be adapted in case of modifications
	double Sigma_Gauss_kernel_; 		//std deviation of the Gaussian filter  
	double Filt_offset_; 				//offset to threshold some data in the comparison   

	//parameters for filtering Zmap
	int MedianFilt_window_size_; 	//size of the window for the median filter applied to the semi-dense depth map

	//parameters for Point Cloud filtering
	double PCl_Radius_thres_; 		//number of neighbors in the area of a features to be taken into account
	int PClF_nb_neighbors_;  	//number of nearest neighbors to take in account for the computation of the mean distance
	
	//parameters for the feature list
	int MinFeatures_to_update_; //minimum of features for one DSI to update the feature list
};

class Mapping_EMVS
{
	public:
		Mapping_EMVS(Mapping_param_struct input_param, sensor_info info, float input_depth);
		~Mapping_EMVS();
		void Process_event(event& evt, const pose& CurrentPose);
		pcl::PointCloud<pcl::PointXYZ>::Ptr features_ptr; //pointer to the list of the features in 3D space (to be used by espee_vo)
		bool new_features_extracted = false;
		
	private:
		Mapping_param_struct param;

		///variables
		// parameters defining the mapping from pixels to image plan co-ordinates
		sensor_info camera_info;
		
		//DSI variables
		int dim_DSI[3];			//3 dimensions of the DSI matrix
		std::vector<float> DSI_z_range;		//z range
		std::vector<float> DSI_invz_range;      //inverse z range
		std::vector<float> DSI_x_range;			//x range
		std::vector<float> DSI_y_range;			//y range

		pose X_DSI;				//pose of the DSI keyframe
		Eigen::Quaternionf Qinv_DSI;				//quaternion inverse of the DSI keyframe

		float mean_depth;			//mean depth

		std::vector<RowMatrixXf> DSI;					//Disparity Sp																																																								ace Image
		bool DSI_initialized;  
		uint Nb_event_DSI = 0;		//number of event used in the current DSI
		
		bool trig_dist1 = false;
		pcl::PointCloud<pcl::PointXYZ>::Ptr features_new_mapping;			//store the extracted 3D point cloud to populate into the list of the 3D space
		
		
		///EVO_mapping param
		Eigen::RowVectorXf GaussFilt_vec;		//separable vector to create the Gaussian Kernel (GK = GaussFilt_vec^t * GaussFilt_vec;)
			
		RowMatrixXi diff_map;				//result of the threshold: 1 if a depth has been found (V2 put the Zmap value to be filtered), -1 otherwise 
		RowMatrixXi Zmap;					//Depth Map	(store indices)				//result of the threshold with the comparison of the Cmap_smooth and offset

		//temp variable for Median filtering
		RowMatrixXf Z_semidense;   		//Depth Map	(store real depth)	
 
		
		///functions
		void unitfp_to_pix(float* uv, int16_t* pixels);
		
		//function for the initialization of the variables according to camera and settings
		void Depth_sampling(std::vector<float>& z_range);
		
		//initialization of the new DSI
		void init_DSI(int16_t* event_uv, const pose& X_curr);
		
		int index_DSI(int x, int y, int z);   //x*height*depth + y*depth + z
		void Update_DSI(int16_t* event_uv, const Eigen::Vector3f& Trans_DSI, const pose& X_curr);
		void Update_DSI_EVO(const Eigen::Vector3f& vec, const Eigen::Vector3f& Trans_DSI, int bound_min, int bound_max);
		
		void median_filtering_Zmap(const RowMatrixXi& In, Eigen::Ref<RowMatrixXf> Out);
		
		void Extract_NewFeatureList_EVO(double time);
		void Depthmap2PointCloud(double time);        
		void Populate_PointCloud();		
		
		///saving into txt file functions 
		void save_DSI_fcn(double time);
		void save_Cmap(double time, const RowMatrixXf& C, const RowMatrixXf& C_smooth);
		void save_diff_map(double time);
		void save_PointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr list_f, double time);
		
};

}

#endif
