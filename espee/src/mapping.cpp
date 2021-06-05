
#include "espee/mapping.h"

namespace espee_ns{

Mapping_EMVS::Mapping_EMVS(Mapping_param_struct input_param, sensor_info info, float input_depth)
: param(input_param), 
  camera_info(info), 
  features_ptr(new pcl::PointCloud<pcl::PointXYZ>), 
  features_new_mapping(new pcl::PointCloud<pcl::PointXYZ>)
{

	///Initialization of the variables used for the mapping part
	#ifdef VERBOSE_MAPPING		
		printf("Enter function Mapping_EMVS::init_mapping \n");
	#endif
	
	//Initialize dim_DSI
	dim_DSI[0]= int(camera_info.x_dim);
	dim_DSI[1]= int(camera_info.y_dim);
	dim_DSI[2]= param.Nz_;
	
	printf("Size DSI = (%d,%d)\n", dim_DSI[0], dim_DSI[1]);
	
	//Initialize z sampling
	Mapping_EMVS::Depth_sampling(DSI_z_range);
	#ifdef VERBOSE_MAPPING	
		printf("depth sampling applied: \n");
	#endif
	for(int i=0;i<param.Nz_;i++){
		DSI_invz_range.push_back(1/DSI_z_range[i]); 
		#ifdef VERBOSE_MAPPING	
			printf("\t DSI_z_range[%d] = %f \n", i, DSI_z_range[i]);
		#endif
	}     
		
	///Initialize x and y sampling
	Eigen::Vector3f point_temp;
	int16_t pix_temp[2]={0,0};

	DSI_x_range.reserve(dim_DSI[0]);
	#ifdef VERBOSE_MAPPING
		printf("DSI_x range :\n");
	#endif
	for(int i=0;i<dim_DSI[0];i++){
		pix_temp[0]=i;
		camera_info.pixel_to_camera(pix_temp, point_temp);
		DSI_x_range.push_back(point_temp(0));
		#ifdef VERBOSE_MAPPING
			printf("\t idx %d: %f\n", i, DSI_x_range[i]);
		#endif	
	}
		
	DSI_y_range.reserve(dim_DSI[1]);
	#ifdef VERBOSE_MAPPING
		printf("DSI_y range :\n");
	#endif
	for(int i=0;i<dim_DSI[1];i++){
		pix_temp[1]=i;
		camera_info.pixel_to_camera(pix_temp, point_temp);
		DSI_y_range.push_back(point_temp(1)); 
		#ifdef VERBOSE_MAPPING
			printf("\t idx %d: %f\n", i, DSI_y_range[i]);
		#endif	
	}	
	
	//initialize the memory space for the DSI
	for(int i=0; i<param.Nz_; ++i)
	{
		DSI.push_back(RowMatrixXf::Zero(dim_DSI[0],dim_DSI[1]));
	}
	
	DSI_initialized=0; //should be set to 1 once the position estimation is initialized
	
	//init the gaussian kernel for the filtering		
	GaussFilt_vec = Eigen::RowVectorXf::Zero(param.Dim_Gauss_kernel_);

	if(param.Dim_Gauss_kernel_%2 ==0){
		printf("[ERROR] The mapping parameter for the dimension of the Gaussian kernel is not odd, default value 5 will be used.");
		param.Dim_Gauss_kernel_ = 5;
	}

	int div_dim = param.Dim_Gauss_kernel_>>1;
	float gain = sqrt(1/(2*PI*pow(param.Sigma_Gauss_kernel_,2)));
	for(int ii=0; ii<=div_dim; ii++)
		GaussFilt_vec(ii) = gain*exp(-pow(ii-2,2)/(2*pow(param.Sigma_Gauss_kernel_,2)));
	for(int ii=div_dim+1; ii<param.Dim_Gauss_kernel_; ii++)
	{
		GaussFilt_vec(ii) = GaussFilt_vec(-ii+2*div_dim);
	}
		 
	
	#ifdef VERBOSE_MAPPING	
		printf("\t The Gaussian kernel vector is: [ %f, %f, %f, %f, %f ] \n", GaussFilt_vec(0), GaussFilt_vec(1), GaussFilt_vec(2), GaussFilt_vec(3), GaussFilt_vec(4));
	#endif
	
	//set init mean depth, initialize its value with input_depth
	mean_depth = input_depth;
	
	#ifdef VERBOSE_MAPPING		
		printf("Exit function Mapping_EMVS::init_mapping \n");
	#endif
}

Mapping_EMVS::~Mapping_EMVS()
{
	///Destructor of the variables used for the mapping part	

	#ifdef VERBOSE_MAPPING		
		printf("deleting object Mapping_EMVS, freeing all allocated memory\n");
	#endif
}

void Mapping_EMVS::Process_event( event& evt , const pose& CurrentPose)
{
	Eigen::Vector3f Translation_DSI;
	float SqrSum=0;	
	bool Extract_f = 0;
	int16_t z_pix[2] = {int16_t (evt.x), int16_t(evt.y)};
	
	++Nb_event_DSI;
	
	
	if(DSI_initialized)   
	{
		
		//transformation of Xpos into world co-ordinates Xpos_cam_world
		Eigen::Vector3f X_cam_world;
		Eigen::Vector3f X_cam_DSI;

		X_cam_world = CurrentPose.q * CurrentPose.p;  //Cam position from cam frame to world frame
		X_cam_DSI = Qinv_DSI * X_cam_world;  //Cam position from world frame to DSI frame


		//check the distant of the DSI frame to current position, only done once every 2047 events to reduce computation load (can be less)
		if((Nb_event_DSI & 0x7FF)==0x7FF)
		{
			Translation_DSI = X_cam_DSI-X_DSI.p;
			SqrSum = Translation_DSI(0)*Translation_DSI(0)
							+Translation_DSI(1)*Translation_DSI(1)
							+Translation_DSI(2)*Translation_DSI(2)/4;

			if(trig_dist1 == false){
				if(SqrSum < pow(param.Percent_Dmean_1_ * mean_depth, 2))
				{
					Extract_f = 0;
				}
				else{
					Extract_f = 1;
					printf("\t First Distance threshold overrun \n");
				}
			}
			else{
				if(SqrSum < pow(param.Percent_Dmean_2_ * mean_depth, 2))
				{
					Extract_f = 0;
				}
				else{
					Extract_f = 1;
					printf("\t Second Distance threshold overrun \n");
				}
			}
		}
		else
		{
			Translation_DSI = X_cam_DSI-X_DSI.p;
		}
			
		if(Extract_f==0  && Nb_event_DSI< param.max_event_per_DSI_) 
		{
			Update_DSI(z_pix, Translation_DSI, CurrentPose);
		}
		else
		{
			int nb_features_mapping;
			#ifdef SAVE_DSI
				printf("Save DSI at time %f\n", evt.ts.toSec());
				printf("\t meand_depth = %f, dist = %f\n", mean_depth, sqrt(SqrSum));
				save_DSI_fcn(evt.ts.toSec());
			#endif
			
			// extract feature and compute mean depth
			#ifdef VERBOSE_Extract_features
			printf("Extract feature and compute mean depth EVO\n");
			#endif
			Extract_NewFeatureList_EVO(evt.ts.toSec());
			
			if(trig_dist1==false)
			{	
				//populate the feature list with the extracted feature as a FIFO  to perform SLAM
				if(features_new_mapping->size() > param.MinFeatures_to_update_)
				{	
					//populate the feature list with the extracted feature as a FIFO to perform SLAM
					Populate_PointCloud();
					new_features_extracted = true;
					
					//reset DSI
					init_DSI(z_pix, CurrentPose);
				}
				else{
					printf("not enough features extracted to populate map, wait for second threshold \n");
					Update_DSI(z_pix, Translation_DSI, CurrentPose);
					trig_dist1 = true;
				}
			}
			else
			{
				//populate the feature list with the extracted feature as a FIFO to perform SLAM
				Populate_PointCloud();
				new_features_extracted = true;
				
				trig_dist1 = false;
				
				//reset DSI
				init_DSI(z_pix, CurrentPose);
			}
			
			Extract_f=0;
		}
	}
	else
	{
		init_DSI(z_pix, CurrentPose); //reset the DSI once the initialization is not done
		DSI_initialized =1; 
	}
	
}

// Projects a point (point) in world co-ordinates to a pixel location (pixels) based on the camera pose (X) 
void Mapping_EMVS::unitfp_to_pix(float* uv, int16_t* pixels)
{
	// map to pixel co-ordinates
	pixels[0] = int16_t( uv[0]*camera_info.fxp + camera_info.cxp);
	pixels[1] = int16_t( uv[1]*camera_info.fyp + camera_info.cyp);
	//pixels[2] = 1;
}

//The function generates the distance of the different planes from the center of the camera
//Depth(x) = -k/(x-offset)+C  with  x E [1:param.Nz_]
void Mapping_EMVS::Depth_sampling(std::vector<float>& z_range)
{
	float diff,G;
	float k, offset, C;
	
	if(param.D_min_mapping_ + param.dz_min_*param.Nz_ - (param.D_max_mapping_ + param.dz_min_) ==0)
		printf("\t Wrong inputs for the Depth_sampling, STOP the program\n,");
	
	diff=param.D_min_mapping_-param.D_max_mapping_;
	G= param.dz_min_ * (param.Nz_-1);
	
	k = param.dz_min_*(pow(param.Nz_, 2) - 3*param.Nz_ + 2)*diff*(diff+param.dz_min_)/(pow(diff + G,2));
	offset = (param.Nz_*diff+2*G)/(diff+G)-1;
	C = (pow(param.D_min_mapping_, 2) + param.D_min_mapping_*(param.dz_min_-param.D_max_mapping_) + param.D_max_mapping_*param.dz_min_*(param.Nz_-2))/(diff+G); 
	 
	for(int i=0;i<param.Nz_;i++)
		z_range.push_back(-k/(i-offset)+C);
			
	printf("SAMPLING Characteristics \n");
	printf("\t Minimum step size: %.3f m\n", z_range[0] );
	printf("\t Maximum step size: %.3f m\n", z_range[param.Nz_-1] );
}

//Initialization of the DSI and the variables for each new keyframe
void Mapping_EMVS::init_DSI(int16_t* event_uv, const pose& X_curr)
{
	float inv_norm;
	int idx_temp;
	
	//reset the DSI
	for(int z=0;z<dim_DSI[2]; ++z)
	{
		DSI[z] = RowMatrixXf::Zero(dim_DSI[0],dim_DSI[1]);
	}

	//reset the number of events used in the DSI
	Nb_event_DSI = 0;
	
	//save the keyframe position and inverse quaternion	
	X_DSI = X_curr;
	
	//get the quaternion inverse 
	Qinv_DSI = X_DSI.q.inverse();
	
  //Update DSI for the first event
	Eigen::Vector3f vec_0 = Eigen::Vector3f::Zero();
  Update_DSI(event_uv, vec_0, X_curr);
		
	
}

void Mapping_EMVS::Update_DSI(int16_t* event_uv, const Eigen::Vector3f& Trans_DSI, const pose& X_curr)
{
	Eigen::Vector3f vec=Eigen::Vector3f(0,0,1);
	Eigen::Vector3f vec_temp;
	
	#ifdef VERBOSE_DSIup   		
		printf("Enter function Mapping_EMVS::Update_DSI \n");
	#endif
	
	//compute A^{-1} * [x;y;1]
	camera_info.pixel_to_camera(event_uv, vec);
	
	//Rotation of the camera compare to DSI position
	vec_temp = X_curr.q * vec;	//ray vector into world frame 
	vec = Qinv_DSI * vec_temp;  //ray vector into DSI frame 

	/*** populate the DSI, another version could store the rays and perform this at feature extraction****/
	Update_DSI_EVO(vec, Trans_DSI, 0, dim_DSI[2]);
	
	#ifdef VERBOSE_DSIup    		
		printf("Exit function Mapping_EMVS::Update_DSI \n\n");
	#endif
	
}

void Mapping_EMVS::Update_DSI_EVO(const Eigen::Vector3f& vec, const Eigen::Vector3f& Trans_DSI, int bound_min, int bound_max)
{
	float uv_idx[2];
	int16_t xy_pix[2];
	float alpha, x_val, y_val; 
	
	
	for(int i=bound_min; i<bound_max; i++)
	{
		//Compute the x,y position of the ray in the z plans of the DSI
		alpha = (DSI_z_range[i] - Trans_DSI(2))/vec(2);
		x_val = alpha * vec(0) + Trans_DSI(0);
		y_val = alpha * vec(1) + Trans_DSI(1);
		
		//Convert into ratio equivalent to tan(theta) 
		uv_idx[0] = x_val * DSI_invz_range[i];       
		uv_idx[1] = y_val * DSI_invz_range[i];
		
		//convert from focal plane to pixel coordinate
		unitfp_to_pix(uv_idx, xy_pix);
		
		///compute the barycentre between points to increase resolution in the DSI (the closest voxels ar all updated)
		//test if in the DSI boundaries     	
		if(DSI_x_range[0]<=uv_idx[0] && DSI_x_range[dim_DSI[0]-1]>uv_idx[0] && DSI_y_range[0]<=uv_idx[1] && DSI_y_range[dim_DSI[1]-1]>uv_idx[1])		
    {
			
			float Rx, Ry;
			if(uv_idx[0]>=DSI_x_range[xy_pix[0]] && uv_idx[1]>=DSI_y_range[xy_pix[1]]){
				Rx =  (uv_idx[0] - DSI_x_range[xy_pix[0]])/(DSI_x_range[xy_pix[0]+1]-DSI_x_range[xy_pix[0]]);
				Ry =  (uv_idx[1] - DSI_y_range[xy_pix[1]])/(DSI_y_range[xy_pix[1]+1]-DSI_y_range[xy_pix[1]]);
					
				DSI[i](xy_pix[0], xy_pix[1]) += (2-Rx-Ry)/4;	//(1-Rx)/4+(1-Ry)/4;
				DSI[i](xy_pix[0]+1, xy_pix[1]) += (Rx+1-Ry)/4;	//(Rx)/4+(1-Ry)/4;
				DSI[i](xy_pix[0], xy_pix[1]+1) += (1-Rx+Ry)/4;	//(1-Rx)/4+(Ry)/4;
				DSI[i](xy_pix[0]+1, xy_pix[1]+1) += (Rx+Ry)/4;	//(Rx)/4+(Ry)/4;
			}
			else if(uv_idx[0]<DSI_x_range[xy_pix[0]] && uv_idx[1]>=DSI_y_range[xy_pix[1]]){
				Rx =  (uv_idx[0] - DSI_x_range[xy_pix[0]-1])/(DSI_x_range[xy_pix[0]]-DSI_x_range[xy_pix[0]-1]);
				Ry =  (uv_idx[1] - DSI_y_range[xy_pix[1]])/(DSI_y_range[xy_pix[1]+1]-DSI_y_range[xy_pix[1]]);
			
				DSI[i](xy_pix[0]-1, xy_pix[1]) += (2-Rx-Ry)/4;	//(1-Rx)/4+(1-Ry)/4;
				DSI[i](xy_pix[0], xy_pix[1]) += (Rx+1-Ry)/4;	//(Rx)/4+(1-Ry)/4;
				DSI[i](xy_pix[0]-1, xy_pix[1]+1) += (1-Rx+Ry)/4;	//(1-Rx)/4+(Ry)/4;
				DSI[i](xy_pix[0], xy_pix[1]+1) += (Rx+Ry)/4;	//(Rx)/4+(Ry)/4;
			}
			else if(uv_idx[0]>=DSI_x_range[xy_pix[0]] && uv_idx[1]<DSI_y_range[xy_pix[1]]){
				//printf("barycenter : val= {%f, %f}, xy_pix{%d,%d}=> range{%f,%f, %f,%f};  \n",uv_idx[0], uv_idx[1], xy_pix[0],xy_pix[1], DSI_x_range[xy_pix[0]], DSI_x_range[xy_pix[0]+1], DSI_y_range[xy_pix[1]], DSI_y_range[xy_pix[1]+1]);
				Rx =  (uv_idx[0] - DSI_x_range[xy_pix[0]])/(DSI_x_range[xy_pix[0]+1]-DSI_x_range[xy_pix[0]]);
				Ry =  (uv_idx[1] - DSI_y_range[xy_pix[1]-1])/(DSI_y_range[xy_pix[1]]-DSI_y_range[xy_pix[1]-1]);
					
				DSI[i](xy_pix[0], xy_pix[1]-1) += (2-Rx-Ry)/4;	//(1-Rx)/4+(1-Ry)/4;
				DSI[i](xy_pix[0]+1, xy_pix[1]-1) += (Rx+1-Ry)/4;	//(Rx)/4+(1-Ry)/4;
				DSI[i](xy_pix[0], xy_pix[1]) += (1-Rx+Ry)/4;	//(1-Rx)/4+(Ry)/4;
				DSI[i](xy_pix[0]+1, xy_pix[1]) += (Rx+Ry)/4;	//(Rx)/4+(Ry)/4;
			}
			else if(uv_idx[0]<DSI_x_range[xy_pix[0]] && uv_idx[1]<DSI_y_range[xy_pix[1]]){
				Rx =  (uv_idx[0] - DSI_x_range[xy_pix[0]-1])/(DSI_x_range[xy_pix[0]]-DSI_x_range[xy_pix[0]-1]);
				Ry =  (uv_idx[1] - DSI_y_range[xy_pix[1]])/(DSI_y_range[xy_pix[1]+1]-DSI_y_range[xy_pix[1]]);
			
				DSI[i](xy_pix[0]-1, xy_pix[1]-1) += (2-Rx-Ry)/4;	//(1-Rx)/4+(1-Ry)/4;
				DSI[i](xy_pix[0], xy_pix[1]-1) += (Rx+1-Ry)/4;	//(Rx)/4+(1-Ry)/4;
				DSI[i](xy_pix[0]-1, xy_pix[1]) += (1-Rx+Ry)/4;	//(1-Rx)/4+(Ry)/4;
				DSI[i](xy_pix[0], xy_pix[1]) += (Rx+Ry)/4;	//(Rx)/4+(Ry)/4;
			}
		}	
			
		#ifdef VERBOSE_DSIup    		
			printf("\t index of the DSI updated: x = %d, y = %d, z=%d\n", xy_pix[0], xy_pix[1], i);
		#endif
		
	}
}

void Mapping_EMVS::median_filtering_Zmap(const RowMatrixXi& In, Eigen::Ref<RowMatrixXf> Out)
{
	std::vector<float> data;
	int nb_data_vector;
	

	#ifdef VERBOSE_MAPPING
		printf("Entering median filtering function\n");
	#endif	
	
	for( int x=0; x<dim_DSI[0]; x++){
		for( int y=0; y<dim_DSI[1]; y++){
			data.clear();
			data.reserve(param.MedianFilt_window_size_*param.MedianFilt_window_size_);
			// Fill in the vector of values with the depths around the interest point
      for (int y_dev = -param.MedianFilt_window_size_/2; y_dev <= param.MedianFilt_window_size_/2; ++y_dev)
				for (int x_dev = -param.MedianFilt_window_size_/2; x_dev <= param.MedianFilt_window_size_/2; ++x_dev)
				{
					if (x + x_dev >= 0 && x + x_dev < dim_DSI[0] && y + y_dev >= 0 && y + y_dev < dim_DSI[1])
					{
						if(In(x+x_dev, y+y_dev)>0)
						{
							data.push_back(DSI_z_range[In(x+x_dev, y+y_dev)-1]);
						}
					}
				}
			nb_data_vector = data.size();
			if(nb_data_vector>=1)
			{
				std::sort(data.begin(), data.end());	//sort the vector 
				
				if(nb_data_vector % 2)
				{
					Out(x,y) = data[nb_data_vector / 2];
				}
				else
				{
					nb_data_vector = nb_data_vector / 2;
					Out(x,y) = (data[nb_data_vector - 1] + data[nb_data_vector]) / 2;
				}
			}
			else
				Out(x,y) = -1;
				
		}
	}
	#ifdef VERBOSE_MAPPING
		printf("exit median filtering function\n");
	#endif
}

void Mapping_EMVS::Extract_NewFeatureList_EVO(double time)
{	
	//Construct a depth and confidence map
	Zmap = RowMatrixXi::Zero(dim_DSI[0],dim_DSI[1]);
	Zmap.array() -= 1;
	RowMatrixXf Cmap = RowMatrixXf::Zero(dim_DSI[0],dim_DSI[1]);				//Confidence Map
	RowMatrixXf Cmap_smooth = RowMatrixXf::Zero(dim_DSI[0],dim_DSI[1]);

	for(int z=0; z<dim_DSI[2]; z++)
	{
		//update Cmap by comparing with the plane at depth z
			//for mapping, init Zmap at -1
		
		for(int i=0; i<dim_DSI[0]; ++i)
			for(int j=0; j<dim_DSI[1]; ++j)
			{
				if(Cmap(i,j)<DSI[z](i,j))
				{
					Cmap(i,j) = DSI[z](i,j);
					Zmap(i,j) = z; 
				}
			}
	}		

	//filter the confidence map
	SeparableConvolution2d(Cmap, GaussFilt_vec, GaussFilt_vec, REFLECT, &Cmap_smooth);
	
	//Save the Cmap and Cmap smoothed
	#ifdef SAVE_DSI
		save_Cmap(time, Cmap_smooth);	
	#endif

	//construct the diff_map matrix with the result of (Cmap-Cmap_smooth+Filt_offset)
	RowMatrixXf diff_comp;
	//diff_map = RowMatrixXi::Zero(dim_DSI[0],dim_DSI[1]);

	diff_comp.array() = (Cmap.array() - Cmap_smooth.array() + (float)param.Filt_offset_);
	diff_map.array() = (diff_comp.array()>0).cast<int>();
	diff_map.array() *= (Zmap.array() + 1);

	//save results
	#ifdef SAVE_DSI
		save_diff_map(time);
	#endif
	
	//apply the Median filter
	Z_semidense = RowMatrixXf::Zero(dim_DSI[0],dim_DSI[1]);
	median_filtering_Zmap(diff_map, Z_semidense);
	Depthmap2PointCloud(time);
	
}

void Mapping_EMVS::Depthmap2PointCloud(double time)
{
	//variables for feature extraction
	float d_temp=0;		//variable to compute the mean depth
	float point_temp[3]={0,0,0};
	Eigen::Vector3f point_world;
	Eigen::Vector3f translation;
	float quat[4]; 
	
	//get the transformation variables from DSI frame to world frame
	translation = X_DSI.q * X_DSI.p;
	
	features_new_mapping->clear();  //erase the old
	features_new_mapping->reserve(NB_MAX_FEATURES);		//TODO check this as the address sanitizer report an error at second func call
	
	#ifdef VERBOSE_Extract_features
		printf("[Mapping_EMVS::Depthmap2PointCloud] \n"); 
	#endif
	 
	for(int i=0; i<(dim_DSI[0]); i++){
		for(int j=0; j<(dim_DSI[1]); j++)
		{	
			if(diff_map(i,j)>0 && (features_new_mapping->size() < NB_MAX_FEATURES))		
			{
				//get the point coordinate in the keyframe
				point_temp[2] = Z_semidense(i,j);

				//barycentric
				point_temp[0] = point_temp[2]*DSI_x_range[i];
				point_temp[1] = point_temp[2]*DSI_y_range[j];
				
				//compute sum of depths		
				d_temp += point_temp[2];
				
				#ifdef VERBOSE_create_features
					printf("\t Features: i=%d , j=%d \n", i,j);
					printf("\t Zmap(i,j) = %d, z = %f m", Zmap(i,j), point_temp[2]);
				#endif

				//convert the feature position into 3D space
				Eigen::Vector3f point3D = Eigen::Vector3f(point_temp[0], point_temp[1], point_temp[2]);
				point_world = X_DSI.q * point3D;  //rotate to express the point position into the world frame
				point_world += translation;  //translation from DSI frame to world frame

				#ifdef VERBOSE_create_features
					printf("\t x= %.3f, y= %.3f, z= %.3f\n", point_world(0),point_world(1),point_world(2));
				#endif
				
				//save the point in the point cloud
				features_new_mapping->push_back (pcl::PointXYZ (point_world(0),point_world(1),point_world(2)));
			}
		}
	}

	///compute mean depth or keep the old one
	if(features_new_mapping->size() > param.MinFeatures_to_update_)     //find a solution if nb_features is under a threshold
		mean_depth = (d_temp/features_new_mapping->size());
	//else we keep the old value
	
	#ifdef VERBOSE_Extract_features
		printf("\t updates features lists at time %f sec\n", time);
		printf("\t Mean depth = %f m \t Nb features mapped = %lu \n", mean_depth, features_new_mapping->size()); 
	#endif
	
	#ifdef SAVE_POINTCLOUD
		save_PointCloud( features_new_mapping, time);
	#endif
	
}

//populate the feature list with the extracted feature as a FIFO to perform SLAM or replace all the old features
void Mapping_EMVS::Populate_PointCloud()
{
	int nb_new_features;
	
	//filter the point cloud 
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;
	// build the filter
	outrem.setInputCloud(features_new_mapping);
	outrem.setRadiusSearch(param.PCl_Radius_thres_);
	outrem.setMinNeighborsInRadius (param.PClF_nb_neighbors_);
	// apply filter
	outrem.filter (*cloud_filtered);  //check better notation
	
	features_new_mapping = std::move(cloud_filtered);	//copy point cloud			 
	
	nb_new_features = features_new_mapping->size();
	
	printf("\t number of inliers after Point cloud filter : %d \n", nb_new_features);	
	
	#ifdef Discard_Old_Features
		///*remove all the data of the feature list and use only the new ones*/
		//float* ptr_list = features_new_mapping;
		//for(int i=0; i<nb_features_mapping; i++, ptr_list += 3){
			//feature_list[i][0] = *(ptr_list);
			//feature_list[i][1] = *(ptr_list+1);
			//feature_list[i][2] = *(ptr_list+2);
		//}	
		//NB_feature_in_list = nb_features_mapping;
		
		features_ptr->clear(); //delete all current point cloud
		features_ptr->reserve(nb_new_features);  //reserve memory space
		*features_ptr = *features_new_mapping; 
		
	#else
		///*remove the oldest data from the list as in FIFO*/	
		if((features_ptr->size() + features_new_mapping->size())<=NB_MAX_FEATURES)			//case all features (old and new) can be kept 
		{
			features_ptr->reserve(features_ptr->size() + features_new_mapping->size());
			*features_ptr += *features_new_mapping;  //TODO check std::copy
		}
		else if(nb_new_features>=NB_MAX_FEATURES)   //case all old features are discarded and replaced by new ones
		{
			features_ptr->clear(); //delete all current point cloud
			features_ptr->reserve(nb_new_features);  //reserve memory space
			*features_ptr = *features_new_mapping;
		}
		else 										//case oldest features are discarded and new features are added to fill up the list
		{									
			
			int diff = nb_new_features + features_ptr->size() - NB_MAX_FEATURES;
			features_ptr->erase(features_ptr->begin(), features_ptr->begin()+diff);  //remove the old data
			*features_ptr += *features_new_mapping;									 // add the new value
		}
	#endif
}	

//saving into txt file functions 
void Mapping_EMVS::save_DSI_fcn(double time)
{
	std::ofstream DSI_outfile;

	DSI_outfile.open("DSI_data" + std::to_string(time) + ".txt", std::ios::out);
	for(int z=0;z<dim_DSI[2];z++)
	{
		DSI_outfile << (DSI[z]) << "\n";
	}

	DSI_outfile.close();
}

void Mapping_EMVS::save_Cmap(double time, const RowMatrixXf& C, const RowMatrixXf& C_smooth)
{
	std::ofstream Cmap_outfile;
	
	printf("Save Cmap at time %f\n", time);
	Cmap_outfile.open("Cmap_" + std::to_string(time) + ".txt", std::ios::out);
	Cmap_outfile << C << " ";

	Cmap_outfile.close();

	printf("Save Cmap smoothed at time %f\n", time);
	Cmap_outfile.open("Cmap_smoothed_" + std::to_string(time) + ".txt", std::ios::out);
	Cmap_outfile << C_smooth << " ";

	Cmap_outfile.close();
	
	
	printf("Save Zmap at time %f\n", time);
	Cmap_outfile.open("Zmap_" + std::to_string(time) + ".txt", std::ios::out);
	Cmap_outfile << (Zmap) << " ";

	Cmap_outfile.close();
}

void Mapping_EMVS::save_diff_map(double time)
{
	std::ofstream outfile;
	
	printf("Save diff at time %f\n", time);
	outfile.open("Diff_" + std::to_string( time) + ".txt", std::ios::out);
	outfile << diff_map << " ";

	outfile.close();
}

void Mapping_EMVS::save_PointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr list_f, double time)
{
	std::ofstream outfile;
	
	printf("Save PointCloud at time %f\n", time);
	outfile.open("features_" + std::to_string( time) + ".txt", std::ios::out);
	
	for(int i=0; i<list_f->points.size() ;i++){
		outfile << list_f->points[i].x << " " << list_f->points[i].y << " " << list_f->points[i].z << std::endl;
	}
	outfile.close();
}
}
