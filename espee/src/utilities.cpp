#include "espee/utilities.h"

namespace espee_ns {
		
bool operator>(const event &first, const event &second) { return first.ts > second.ts; }
bool operator<(const event &first, const event &second) { return second > first; }
bool operator<=(const event &first, const event &second) { return !(first > second); }
bool operator>=(const event &first, const event &second) { return !(first < second); }	


/******* sensor_info function definition ******************************/

// Projects a point (point) in world co-ordinates to a pixel location (pixels) based on the camera pose (X)
float sensor_info::project(const Eigen::Vector3f& point_world, 
													 const Eigen::Vector3f& X, 
													 const Eigen::Quaternionf& quat, 
													 int16_t* pixels)
{
	Eigen::Vector3f point_camera;
	world_to_camera(point_world, X, quat, point_camera);
	camera_to_pixels(point_camera, pixels);

	return point_camera[2];
}

void sensor_info::camera_to_world(const Eigen::Vector3f& point_camera, 
																	const Eigen::Vector3f& position, 
																	Eigen::Quaternionf& quat, 
																	Eigen::Ref<Eigen::Vector3f> point_world)
{
	point_world = quat * (position + point_camera);
}

void sensor_info::world_to_camera(const Eigen::Vector3f& point_world, 
																	const Eigen::Vector3f& position, 
																	const Eigen::Quaternionf& quat,
																	Eigen::Ref<Eigen::Vector3f> point_camera)
{
	point_camera = (quat.inverse() * point_world) - position;
}

void sensor_info::camera_to_pixels(const Eigen::Vector3f& point_camera, int16_t* pixels)
{
	float point[2];
	point[0] = point_camera(0)/point_camera(2);
	point[1] = point_camera(1)/point_camera(2);

	pixels[0] = point[0]*fxp + cxp;
	pixels[1] = point[1]*fyp + cyp;
}

void sensor_info::pixel_to_camera(int16_t* pixels, Eigen::Ref<Eigen::Vector3f> point_camera)
{
	point_camera(0) = (float(pixels[0])-cxp)/fxp;
	point_camera(1) = (float(pixels[1])-cyp)/fyp;
	point_camera(2) = 1;
}
	
/******* quaternion related functions *********************************/
// converts a rotation vector (v) to a quaternion (quat). Takes care of the quaternion normalization!!!
void vec2quat(const Eigen::Vector3f& vec, Eigen::Quaternionf& quat)
{
  //test value approximation of the conversion
  float theta_squared = 0;
  for(int i=0; i<3;i++)
    theta_squared += vec(i)*vec(i);
  
  float theta = sqrt(theta_squared);	
  float C = cos(theta/2);
  float S;
  if(theta == 0)
    S = 1;
  else
    S = sin(theta/2)/theta;
    
  quat = Eigen::Quaternionf(C, S*vec(0), S*vec(1), S*vec(2));
}

//Convert a quaternion into a vector
void quat2vec(const Eigen::Quaternionf& quat, Eigen::Ref<Eigen::Vector3f>& vec)
{
  //logarithmic map

	float inv_mag;
	
	if(quat.w()==1.0)
	{
		vec(0) = 0;
		vec(1) = 0;
		vec(2) = 0;
	}
	else
	{
		inv_mag = 2*acos(quat.w())/sqrt(1-quat.w()*quat.w());
    vec(0) = inv_mag * quat.x();
		vec(1) = inv_mag * quat.y();
		vec(2) = inv_mag * quat.z();
	}
}

void vec2skew(const Eigen::Vector3f& u, Eigen::Ref<Eigen::Matrix<float, 3, 3>> u_skew)
{
  u_skew << 0, -u(2), u(1),
            u(2), 0, -u(0),
            -u(1), u(0), 0;
}

void get_state2_from_state1(const pose& X1, const pose& X_relative, pose& X2)
{
	Eigen::Vector3f vec = X_relative.q.inverse() * X1.p;
	X2.p = vec - X_relative.p;
	X2.q = X1.q * X_relative.q; //combine_quat_rotations(quat_relative, quat_second, &X2[3]);
}

void get_state1_from_state2(const pose& X2, const pose& X_relative, pose& X1)
{
	X1.p = X_relative.q * (X_relative.p + X2.p);	
	X1.q = X2.q * X_relative.q.inverse();
}

}
