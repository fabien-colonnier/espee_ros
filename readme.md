# ESPEE Visual Odometry algorithm


## Install

You need to install ROS (kinetic, melodic and noetic supported) and create a workspace. 
The package is relying on [rpg_dvs_ros](https://github.com/uzh-rpg/rpg_dvs_ros) for realtime test and otherwise only on the dvs_msgs part for using previously recorded rosbag.

Clone the repo into the src folder of your workspace and compile the code as usual. See the example below to run the code. 

## Example of processing

### Slider_depth
The example comes from the [event-camera datasets](http://rpg.ifi.uzh.ch/davis_data.html) made by the RPG team and can be downloaded [here](http://rpg.ifi.uzh.ch/datasets/davis/slider_depth.bag).

Modify the path to the bag in the launch file espee/launch/readbag_sliderExp.launch

Then you can run the example with:
`$ roslaunch espee readbag_sliderExp.launch`

### Dual sensor experiment as in the paper

## About the param.YAML file
The YAML file contains the main parameters of the algorithm.

* EKF_p: contains all the parameters relative to the EKF computation.
* ESPEE_rig_p: contains the parameters relative to the sensor rig, i.e. number of sensors, relative positions and absolute position at start (if none is specified, (0,0,0,1,0,0,0) will be used).
* Init_map_p: indicated the way of initializing the first point clound. Mode 0 accumulate the Num_points and assumes planar scene whereas Mode 1 uses the PCD file (the point cloud can be specified in global coordinate or relative to a pose). This mode has only been tested with one sensor. It might work with 2 sensors if the mapping is not activated.

* ESPEE_FE_p: contains all the parameters related to mapping. If Mapping_mode=0, then no mapping is performed which make the rest of the parameters irrelevant. Mapping_mode=1 means mapping is performed but won't be used for localization. It helps to validate the mapping parameters. Mapping_mode=2 performs mapping and used it for localization.

**NB**: For floating point values, be careful of the notation. Use "5.0" instead of "5". Especially, "5e-8" will not be properly understood.


## Reference to the paper
Not published yet

## Reference to code used in this programme
This code used the 2D separable convolution code written by Chris Sweeny in his project [akaze-eigen](https://github.com/sweeneychris/akaze-eigen) for the mapping algorithm.

## License
The code and models in this repository are licensed under the GNU General Public License Version 3. For commercial use of this code and models, separate commercial licensing is also available. Please contact me (Fabien COLONNIER email: fabien(dot)colonnier(at)gmail(dot)com) or NUS representative Jonathan Tan (email: jonathan_tan(at)nus(dot)edu(dot)sg)
