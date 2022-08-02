#include "ros/ros.h"
#include "ros/package.h"
#include <string>
#include "pgmMapCheck.h"
#include "pgmMapCheck.cpp"

/*
_____________________________________________________________________________________________
		
		MAIN - CODE
_____________________________________________________________________________________________
*/

//INFO: FOR PGM FILES P5 means that these levels are byte codes, P2 is ASCII numbers


int main(int argc, char** argv){

	//initialize node
	ros::init(argc, argv, "unoccupied_base_map_generator");

	//start ROS node handler
	ros::NodeHandle nh;

	//initialize (through ROS) changable parameters
	string base_map_path = "/maps/fhws_first_floor_small.pgm";
	string base_map_ros_pkg = "map_update";
	string base_map_yaml_file_path = "/maps/initial_hector_base_map.yaml";

	int changable_grid_cell_deadzone = 3;
	
	//variables for file handling (SHOULD NOT BE CHANGED!!!)
	string unoccupied_base_map_path = "/unoccupied_base_map";

	//looking for new parameter values (given through launch file e.g)
	std::string nodeName = ros::this_node::getName();		//get the name of this ros node

	//get parameters from rosparam
	nh.param<std::string>(("/base_map_path"), base_map_path, std::string("/maps/fhws_first_floor_small.pgm"));
	nh.param<std::string>(("/base_map_ros_pkg"), base_map_ros_pkg, std::string("map_update"));
	nh.param<std::string>(("/base_map_yaml_file_path"), base_map_yaml_file_path, std::string("/maps/fhws_first_floor_small.yaml"));
	nh.param<int>(("/changable_grid_cell_deadzone"), changable_grid_cell_deadzone, 3);			
	
	//path to base map pgm file
	base_map_ros_pkg = ros::package::getPath(base_map_ros_pkg);			//get the path to the ros package the base map is stored in
	base_map_path = base_map_ros_pkg + base_map_path;					//get (full) path to the base_map / initial map
	base_map_yaml_file_path = base_map_ros_pkg + base_map_yaml_file_path;			//give path to the base_map yaml file for generating a new yaml file
												//in order to create a initial map without occupied cells
	
	std::cout << "BASE MAP PATH: " << base_map_path << std::endl;//delete
	//path to where the unoccupied base map should be stored
	string mapupdate_path = ros::package::getPath("map_update");				//find the corresponding path to this ros package
	unoccupied_base_map_path = mapupdate_path + unoccupied_base_map_path;

	//create an instance (object) of the pgmMapCheck class to load an manipulate/extend 
	//the base map pgm and yaml file
	pgmMapCheck base_map = pgmMapCheck(base_map_path);

	//base_map.write_new_base_occupancy_grid_map(base_map.occupancy_grid_map());

	//write out a base map (pgm and yaml file) for an unoccupied base/inital map
	base_map.write_unoccupied_map(unoccupied_base_map_path, base_map_yaml_file_path, changable_grid_cell_deadzone);

	return 0;

}