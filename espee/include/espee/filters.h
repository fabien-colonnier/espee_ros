//ALL FILTERING FUNCTIONS GO IN THIS FILE
#ifndef FILTERS_H
#define FILTERS_H

#include <fstream>
#include <iostream>

#include <espee/utilities.h>

using namespace espee_ns;

// A refractory period filter, blocks all events within the refractory time of a previous event
// Arguments: 
// "input_RF_period" an uint32_teger specifying the refractory period in microseconds
class refractory_filter{
	private:
		ros::Duration RF_period;
		std::vector<ros::Time> RF_last_spike;
		uint16_t x_dim;
		uint16_t y_dim;
	public:
		refractory_filter(int input_RF_period, uint16_t x_dim, uint16_t y_dim);
		unsigned long processData(std::vector<event>& dataIN , unsigned long nb_event);
};


// A refractory period filter similar to above, but only blocks events of the same polarity within the refractory period
// Arguments: 
// "input_RFp_period" an integer specifying the refractory period in microseconds
class refractory_period_polarity{
	private:
		ros::Duration RFp_period;
		std::vector<std::array<ros::Time,2>> RFp_last_spike;
		uint16_t x_dim;
		uint16_t y_dim;
	public:
		refractory_period_polarity(int input_RFp_period, uint16_t x_dim, uint16_t y_dim);
		unsigned long processData(std::vector<event>& dataIN , unsigned long nb_event);
};

class filter_Undistort{
	private:
		uint16_t init_data;
		Eigen::Array<uint16_t,Eigen::Dynamic,Eigen::Dynamic> map_x;
		Eigen::Array<uint16_t,Eigen::Dynamic,Eigen::Dynamic> map_y;
		
	public:
		filter_Undistort(sensor_info* info);
		unsigned long processData(std::vector<event>& dataIN , unsigned long nb_event);
};

#endif
