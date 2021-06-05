//ALL FILTERING FUNCTIONS GO IN THIS FILE
#include <algorithm>
#include <cstring>

#include "espee/filters.h"


 
using namespace espee_ns;
//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------


/************************************************************************************************************
Input       : (1) The refractory period
Output      : None
Description : Function to initialize the Refractoy filter.
*************************************************************************************************************/
refractory_filter::refractory_filter(int input_RF_period, uint16_t x_dim, uint16_t y_dim): x_dim(x_dim), y_dim(y_dim)
{
	RF_period = ros::Duration(double(input_RF_period)*1e-6);
	RF_last_spike.reserve(y_dim*x_dim);

	for(unsigned int ii=0; ii<y_dim*x_dim; ++ii)
		RF_last_spike.push_back(ros::Time(0));
	
	printf("\t RF_period initialized at %f s\n", RF_period.toSec());
}

/************************************************************************************************************
Input       : (1) Data buffer information. (2) The input data buffer, (2) The output data buffer, (4) Thread ID 
Output      : Returns the Data buffer information packet.
Description : Function to initialize the Refractoy filter.
*************************************************************************************************************/
unsigned long refractory_filter::processData(std::vector<event>& dataIN , unsigned long nb_event )
{	
	for(std::vector<event>::iterator evt_it = dataIN.end()-nb_event; evt_it != dataIN.end(); ++evt_it )
	{ 
		if (evt_it->ts - RF_last_spike[evt_it->y *x_dim + evt_it->x] < RF_period)
		{
			dataIN.erase(evt_it);
			--evt_it;
			--nb_event;
		}else
		{
			RF_last_spike[evt_it->y *x_dim + evt_it->x] = evt_it->ts;
		}
	}
	return nb_event;
}

//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

/************************************************************************************************************
Input       : (1) The refractory period
Output      : None
Description : Function to initialize the Refractoy filter.
*************************************************************************************************************/
refractory_period_polarity::refractory_period_polarity(int input_RFp_period, uint16_t x_dim, uint16_t y_dim): x_dim(x_dim), y_dim(y_dim)
{
	RFp_period = ros::Duration(double(input_RFp_period)*1e-6);
	
	RFp_last_spike.reserve(y_dim*x_dim);
	for(int i=0; i<y_dim*x_dim; ++i)
	{	
		RFp_last_spike.push_back({ros::Time(0), ros::Time(0)});
	}
}


/************************************************************************************************************
Input       : (1) Data buffer information. (2) The input data buffer, (2) The output data buffer, (4) Thread ID 
Output      : Returns the Data buffer information packet.
Description : Function to implement the refractory filter. Filters off events when their intervals are smaller 
              than the refractory period.
*************************************************************************************************************/
//treats positive and negative events with separate polarities
unsigned long refractory_period_polarity::processData(std::vector<event>& dataIN , unsigned long nb_event)
{
	for(std::vector<event>::iterator evt_it = dataIN.end()-nb_event; evt_it != dataIN.end(); ++evt_it )
	{ 
		if (evt_it->ts - RFp_last_spike[evt_it->y *x_dim + evt_it->x][evt_it->polarity] < RFp_period)
		{
			//printf("\t event to erase x= %d, y= %d, ts = %f\n", evt_it->x, evt_it->y, evt_it->ts.toSec());
			////printf("\t event to erase +1 x= %d, y= %d, ts = %f\n", (evt_it+1)->x, (evt_it+1)->y, (evt_it+1)->ts.toSec());
			dataIN.erase(evt_it);
			--evt_it;
			--nb_event;
			//printf("\t event at iterator position x= %d, y= %d, ts = %f\n", evt_it->x, evt_it->y, evt_it->ts.toSec());
		}else
		{
			RFp_last_spike[evt_it->y *x_dim + evt_it->x][evt_it->polarity] = evt_it->ts;
		}
	}
	return nb_event;
}

//---------------------------------------------------------------------------------------------------------------------------------

/************************************************************************************************************
Input       : (1) Data buffer information. (2) The input data buffer, (2) The output data buffer, 
              (4) Thread ID 
Output      : Returns information about the output event buffer 
Description : This filter enables to compensate the lens distortion for monocular camera
************************************************************************************************************/
filter_Undistort::filter_Undistort(sensor_info* info)
{			
	printf("-F_Undistort:\n \t fxp = %f, fyp = %f, cxp = %f, cyp = %f \n \t k1 = %f, k2 = %f, k3 = %f, p1 = %f, p2 = %f\n", 
			 info->fxp, info->fyp, info->cxp, info->cyp, info->k1, info->k2, info->k3, info->p1, info->p2);
	
	//init theoretical intrinsinc camera parameter 
		//Width = DataSource::x_dim;
		//Height = DataSource::y_dim;
		
	
	//init memory space of the LUT 
	init_data = std::max(info->x_dim, info->y_dim);
	map_x = Eigen::Array<uint16_t,Eigen::Dynamic,Eigen::Dynamic>::Zero(info->y_dim, info->x_dim);
	map_y = Eigen::Array<uint16_t,Eigen::Dynamic,Eigen::Dynamic>::Zero(info->y_dim, info->x_dim);

	for(int y = 0; y < info->y_dim; y++)
	{
		for(int x = 0; x < info->x_dim; x++)
			map_x(y,x) = init_data;	
	}

	
	for(int y = 0; y < info->y_dim; y++)
	{
		for(int x = 0; x <info->x_dim; x++)
			map_y(y,x) = init_data;	
	}
	
	
	//init LUT	
	float vec_temp[2];
	float xyp[2];
	float sqr_radius, poly, cross;
	uint16_t x,y;
	
	for (uint16_t v = 0; v < info->y_dim; v++)
    {
        for (uint16_t u = 0; u < info->x_dim; u++)
        {
			vec_temp[0] = (float(u)-info->cxp)/info->fxp;
			vec_temp[1] = (float(v)-info->cyp)/info->fyp;

			sqr_radius = vec_temp[0]*vec_temp[0] + vec_temp[1]*vec_temp[1];
			poly = 1+ info->k1 * sqr_radius + info->k2 * sqr_radius*sqr_radius + info->k3 *sqr_radius*sqr_radius*sqr_radius ;
			cross = 2*vec_temp[0]*vec_temp[1];
			
			xyp[0] = poly * vec_temp[0] + info->p1 * cross + info->p2*(sqr_radius + 2* vec_temp[0]*vec_temp[0]);
			xyp[1] = poly * vec_temp[1] + info->p1*(sqr_radius + 2* vec_temp[1]*vec_temp[1]) + info->p2 * cross;
			
			x = uint16_t(xyp[0] * info->fxp + info->cxp);  //x = round(xyp[0] * fx + cx);
			y = uint16_t(xyp[1] * info->fyp + info->cyp);	 //y = round(xyp[1] * fy + cy);
			
			//correct out of bound data
			if(x<0)
				continue;
			else if(x>=info->x_dim) 
				continue;
				 
			if(y<0)
				continue;
			else if(y>=info->y_dim) 
				continue;	 

			//save results in LUT
			if( map_x(y,x)==info->x_dim && map_x(y,x)==info->x_dim)
			{
				map_x(y,x) = u;
				map_y(y,x) = v;
			}
			else
			{
				//printf("The value of the map has already been updated :\n \t\t old map_x = %d, new map_x = %d \n \t\t old map_y = %d, new map_y = %d \n",
						//map_x(y,x), u, map_y(y,x), v);
				map_x(y,x) = u;
				map_y(y,x) = v;
			}
			
		}
	}
	
}

/************************************************************************************************************
Input       : (1) Data buffer information. (2) The input data buffer, (2) The output data buffer, 
              (4) Thread ID 
Output      : Returns information about the output event buffer 
Description : This filter enables to compensate the lens distortion for monocular camera 
************************************************************************************************************/
unsigned long filter_Undistort::processData(std::vector<event>& dataIN , unsigned long nb_event)
{

	int16_t current_y, current_x;

	for(std::vector<event>::iterator evt_it = dataIN.end()-nb_event; evt_it != dataIN.end(); ++evt_it )
	{
			current_y = evt_it->y;
			current_x = evt_it->x;			
			
			//printf("\t x = %d, y = %d\n", evt_it->x, evt_it->y);			
			evt_it->x = map_x(current_y,current_x);
			evt_it->y = map_y(current_y,current_x);
			
			//check if initialized data, if erase them
			if(evt_it->x == init_data || evt_it->y == init_data)
			{
				dataIN.erase(evt_it);
				--nb_event;
				--evt_it;
			}
	}
	return nb_event;
}

//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------




