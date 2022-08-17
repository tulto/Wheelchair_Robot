#include "ros/ros.h"
#include "ros/package.h"
#include "geometry_msgs/Pose.h"
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <time.h>
#include <filesystem>
#include <dirent.h>
#include <tuple>
#include "pgmMapCheck.h"
#include "pgmMapCheck.cpp"
#include "pgmMapCompare.h"
#include "pgmMapCompare.cpp"
#include "pgmMapJoin.h"
#include "pgmMapJoin.cpp"
#include "pgmMapMerge.h"
#include "pgmMapMerge.cpp"


using std::string;
using std::vector;

/*
_____________________________________________________________________________________________
		
		USED METHODS/FUNCTIONS
_____________________________________________________________________________________________
*/

//Prototyp for functions
bool file_exists(const std::string& name);
string check_and_write_maps_in_directories(string new_map_name, string new_map_path, int max_maps_count);
void delete_directory_content(const std::string& dir_path);
vector<string> search_maps_in_directory(string path_to_map_directory, string map_name_scheme);

/*
_____________________________________________________________________________________________
		
		MAIN - CODE
_____________________________________________________________________________________________
*/

int main(int argc, char** argv){
	
	//initialize node
	ros::init(argc, argv, "map_update_on_destination_drive");

	//start ROS node handler
	ros::NodeHandle nh;
	
	//initialize changeable parameters
	string base_map_path = "/maps/fhws_first_floor_small.pgm";
	string base_map_ros_pkg = "map_update";
	string base_map_yaml_file_path = "/maps/initial_hector_base_map.yaml";
	string costmap_nc_path = "/build_costmaps_on_destination/costmap_no_clearing.pgm";
	string costmap_c_path = "/build_costmaps_on_destination/costmap_clearing.pgm";
	string unoccupied_costmap_nc_path = "/build_costmaps_on_destination/unoccupied_costmap_no_clearing.pgm";
	string unoccupied_costmap_c_path= "/build_costmaps_on_destination/unoccupied_costmap_clearing.pgm";
	string base_map_with_edit_path = "/maps/fhws_first_floor_small_edit.pgm";
	
	//variables for file handling (SHOULD NOT BE CHANGED!!!)
	string new_compared_map_path = "/same_day_maps";
	string new_compared_map_name = "compared_map_same_day";
	string new_joined_map_path = "/different_day_maps";
	string new_joined_map_name = "joined_map_different_day";
	string new_merged_map_path = "/updated_map_after_multiple_days";
	string new_merged_map_name = "updated_initial_map";
	string unoccupied_base_map_name = "base_map_without_occupied_cells";
	string unoccupied_base_map_path = "/unoccupied_base_map";

	
	//parmeters used for comparing the costmaps and generating a resulting costmap for this autonomous drive
	bool overwrite_initial_map = false;
	bool use_and_overwrite_initial_edited_map = false;
	int changable_grid_cell_deadzone = 3;
	int removed_objects_safety_distance = 1;
	int max_maps_used_per_same_day = 5;
	int max_maps_used_per_different_days = 3;
	
	//looking for new values (given through launch files) for different parameters
	//std::string nodeName = ros::this_node::getName();						//get the name of this ros node
	nh.param<int>(("/changable_grid_cell_deadzone"), changable_grid_cell_deadzone, 3);			
	nh.param<int>(("/max_maps_used_per_same_day"), max_maps_used_per_same_day, 5);
	nh.param<int>(("/max_maps_used_per_different_days"), max_maps_used_per_different_days, 3);
	nh.param<int>(("/removed_objects_safety_distance"), removed_objects_safety_distance, 1);
	nh.param<bool>(("/overwrite_initial_map"), overwrite_initial_map, false);
	nh.param<bool>(("/use_and_overwrite_initial_edited_map"), use_and_overwrite_initial_edited_map, false);
	
	nh.param<std::string>(("/base_map_path"), base_map_path, std::string("/maps/fhws_first_floor_small.pgm"));
	nh.param<std::string>(("/base_map_with_edit_path"), base_map_with_edit_path, std::string("/maps/fhws_first_floor_small_edit.pgm"));
	nh.param<std::string>(("/base_map_ros_pkg"), base_map_ros_pkg, std::string("map_update"));
	nh.param<std::string>(("/base_map_yaml_file_path"), base_map_yaml_file_path, std::string("/maps/fhws_first_floor_small.yaml"));


	//paths to pgm files, base_map, no_clearing_map and clearing_map
	base_map_ros_pkg = ros::package::getPath(base_map_ros_pkg);				//get the path to the ros package the base map is stored in
	base_map_path = base_map_ros_pkg + base_map_path;					//get (full) path to the base_map / initial map
	base_map_yaml_file_path = base_map_ros_pkg + base_map_yaml_file_path;			//give path to the base_map yaml file for generating a new yaml file
												//in order to create a initial map without occupied cells
	base_map_with_edit_path = base_map_ros_pkg + base_map_with_edit_path;			//get (full) path to the base/initial map with edit
	std::cout << "BASE MAP PACKAGE FULL PATH: " << base_map_path << std::endl; //dont forget to delete
	std::cout << "BASE MAP PACKAGE FULL PATH: " << base_map_yaml_file_path << std::endl; //dont forget to delete
	
	string mapupdate_path = ros::package::getPath("map_update");				//find the corresponding path to this ros package
	costmap_nc_path = mapupdate_path + costmap_nc_path;					//give path to the pgm file for the map with no clearing enabled
	costmap_c_path = mapupdate_path + costmap_c_path;					//give path to the pgm file for the map with clearing enabled
	unoccupied_costmap_nc_path = mapupdate_path + unoccupied_costmap_nc_path;		//give path to unoccupied base map with clearing
	unoccupied_costmap_c_path = mapupdate_path + unoccupied_costmap_c_path;			//give path to unoccupied base map without clearing
	
	//path where the new compared map should be stored
	new_compared_map_path = (mapupdate_path + new_compared_map_path);
	//path to where the new joined maps should be stored
	new_joined_map_path = (mapupdate_path + new_joined_map_path);
	//path to where the new updated "initial/base" map (merged map) should be stored
	new_merged_map_path = (mapupdate_path + new_merged_map_path);

	//string to get the path to the csv file used for saving the last day
	string csv_file_path = (mapupdate_path + "/csv_files/safe_last_day.csv");

	//get current day
	std::time_t theTime = time(NULL);
	struct tm *today_date = localtime(&theTime);
	int day = today_date->tm_mday;
	int month = today_date->tm_mon + 1; // month in 0 - 11, therefore we add 1 to get a jan-dec 1-12 concept
	int year = today_date->tm_year + 1900; // Year is # years since 1900 therefore +1900

	//following parameters for the last day the program was active (last day is safed in a csv file)
	const string last_day_csv_name = csv_file_path;
	std::cout << last_day_csv_name << " FILESYSTEM" << std::endl;
	int last_day, last_month, last_year;

	//check if a .csv file, which safes the last day the program has been used, is already exisiting otherwise we will generate it
	if(!(file_exists(last_day_csv_name))){
		std::ofstream csv_maker;
		csv_maker.open((last_day_csv_name), std::ios_base::out);

		if(csv_maker.is_open()){
			csv_maker << day << "," << month << "," << year;	//add current day to file

			std::cout << "File to store the last day this program was running successfully generated!" << std::endl;
		}
		else{
			std::cerr << "Could not generate a csv file to safe the last day this program was running!" << std::endl;
		}
	}	
	
	if(file_exists(last_day_csv_name)){
		
		std::cout << "CSV-file for date storing exists!" << std::endl;
		std::ifstream csv_reader;
		csv_reader.open((last_day_csv_name), std::ios_base::in);
		
		if(csv_reader.is_open()){

			string line_in_csv;
			string value_in_csv;
			std::vector<std::string> result;
			string colname;

			
			std::getline(csv_reader, line_in_csv); //getting first line (file shoud only have one line !!!)

			std::stringstream ss(line_in_csv);	//create a string from the read line

			// Extract each column name
			while(std::getline(ss, value_in_csv, ',')){
				
				result.push_back(value_in_csv);
			}
			std::cout << "size of vec: " << result.size() << std::endl;
			
			if(result.size() == 3){
				last_day = stoi(result[0]);
				last_month = stoi(result[1]);
				last_year = stoi(result[2]);
			}
			else{
				std::cerr << "CSV-file has wrong or unacceptable data stored in it! Could not retrieve last day from CSV-file!!" << std::endl;
			}
			
			std::cout << "date: " << last_day << "," << last_month << "," << last_year << std::endl;	//give out last_day
			
			if(last_day != day || last_month != month || last_year != year){
				
				std::ofstream csv_writer;
				csv_writer.open((last_day_csv_name), std::ios_base::out);

				if(csv_writer.is_open()){
					csv_writer << day << "," << month << "," << year;				//add current day to file

					std::cout << "Updated CSV-file with current day!" << std::endl;
				}
			}
			
		}
	}

	std::cout << "base_map: " << file_exists(base_map_path) << "costmap_c: "  << file_exists(costmap_c_path) << "costmap_nc: " << file_exists(costmap_nc_path) << "unoccupied_costmap_c: " << file_exists(unoccupied_costmap_c_path) << "unoccupied_costmap_nc: " << file_exists(unoccupied_costmap_nc_path) << std::endl;

	if(file_exists(base_map_path) && file_exists(costmap_c_path) && file_exists(costmap_nc_path) && file_exists(unoccupied_costmap_c_path) && file_exists(unoccupied_costmap_nc_path)){		//check if all files needed for a map update exist

		//load all needed files as pgmMapCheck object
		pgmMapCheck base_map = pgmMapCheck(base_map_path);
		base_map.set_grid_cell_deadzone(changable_grid_cell_deadzone); 					//set the deadzone for pixels which are not allowed to change around occupied grids for the base/initial map
		pgmMapCheck no_clearing_map = pgmMapCheck(costmap_nc_path);
		pgmMapCheck clearing_map = pgmMapCheck(costmap_c_path);
		pgmMapCheck unoccupied_clearing_map = pgmMapCheck(unoccupied_costmap_c_path);
		pgmMapCheck unoccupied_no_clearing_map = pgmMapCheck(unoccupied_costmap_nc_path);

		std::cout << "map_unchangable_deadzone: " << (int)base_map.get_grid_cell_deadzone() << std::endl;

		if(day == last_day && month == last_month && year == last_year){
			
			pgmMapCompare resulting_map = pgmMapCompare(&base_map, &no_clearing_map, &clearing_map, &unoccupied_clearing_map, &unoccupied_no_clearing_map, removed_objects_safety_distance); //create instance/objekt of class pgmMapCompare to generate compared maps!

			string safe_name_iter_compared_map = check_and_write_maps_in_directories(new_compared_map_name, new_compared_map_path, max_maps_used_per_same_day);
			
			if(safe_name_iter_compared_map != ""){
				resulting_map.write_new_compared_map(new_compared_map_path, safe_name_iter_compared_map);
			}
			
		}
		else{	
			std::cout << "NEW DAY --> Generating a new map for different days and clearing the same_day_maps directory! (Deleting all same_day_maps in this directory)!" << std::endl;
			vector<string> path_to_all_compared_files_same_day{};		//vector to store all made pgmMapCompare maps in one day
			
			path_to_all_compared_files_same_day = search_maps_in_directory(new_compared_map_path, new_compared_map_name);  //search for files in directory with certain name!

			//delete following later only for testing !!!
			std::cout << "PATHS TO SAME DAY MAPS: " << std::endl;
			
			for(int i = 0; i < path_to_all_compared_files_same_day.size(); i++){
				std::cout << path_to_all_compared_files_same_day[i] << std::endl;
			} //end for deleting
			
			if(path_to_all_compared_files_same_day.size() > 1){			//for joining maps we need more than one map

				vector<pgmMapCheck> maps_to_join = {};				//vector to store compared maps in same_day_maps directory as pgmMapCheck objects for later use in joining

				for(int i=0; i<path_to_all_compared_files_same_day.size(); ++i){
					maps_to_join.push_back(pgmMapCheck(path_to_all_compared_files_same_day[i]));
				}

				pgmMapJoin new_joint_map = pgmMapJoin(&base_map, &maps_to_join);	//object for joining maps

				string safe_name_iter_joined_map = check_and_write_maps_in_directories(new_joined_map_name, new_joined_map_path, max_maps_used_per_different_days);	//check different day maps and generate them if they are not already generated
				if(safe_name_iter_joined_map != ""){
					new_joint_map.write_new_joined_map(new_joined_map_path, safe_name_iter_joined_map);
				}
			}
			else{
				std::cerr << "Not enough maps for joining maps received! Please make sure we have at least 2 (compared) maps in the " << new_compared_map_path << " directory!" << std::endl;
			}

			//because we got a different day the counting or saving of new same_day_maps will have to start at zero
			//therefore we delete the contents of new_compared_map_path (same_day_maps directory)
			delete_directory_content(new_compared_map_path);

			//now we safe the new made map (first map of the day because prior to this we deleted all contents of same_day_maps)
			pgmMapCompare resulting_map = pgmMapCompare(&base_map, &no_clearing_map, &clearing_map, &unoccupied_clearing_map, &unoccupied_no_clearing_map, removed_objects_safety_distance); //create instance/objekt of class pgmMapCompare to generate compared maps!

			string safe_name_iter_compared_map = check_and_write_maps_in_directories(new_compared_map_name, new_compared_map_path, max_maps_used_per_same_day);
			
			if(safe_name_iter_compared_map != ""){
				resulting_map.write_new_compared_map(new_compared_map_path, safe_name_iter_compared_map);
			}

			//further we check if we already have enough joined maps from different days to store a merged map!

			vector<string> path_to_all_joined_files_different_day{};		//vector to store all made pgmMapJoin maps in different days
			
			path_to_all_joined_files_different_day = search_maps_in_directory(new_joined_map_path, new_joined_map_name);  	//search for files in directory with certain name!
			
			if(path_to_all_joined_files_different_day.size() == max_maps_used_per_different_days){				//check if we have as much joined maps as max_maps_used_per_different_days (so how many different updated maps we have) and then merge them into a new "initial/base" map!
				
				vector<pgmMapCheck> maps_to_merge = {};
				
				for(int i=0; i<path_to_all_joined_files_different_day.size(); ++i){
					maps_to_merge.push_back(pgmMapCheck(path_to_all_joined_files_different_day[i]));
				}

				pgmMapMerge new_updated_map = pgmMapMerge(&base_map, &maps_to_merge, base_map_yaml_file_path);

				new_updated_map.write_new_updated_map_pgm_and_yaml(new_merged_map_path, new_merged_map_name);

				if(overwrite_initial_map){
					base_map.write_new_base_occupancy_grid_map(new_updated_map.merge_result_map());
				}

				if(use_and_overwrite_initial_edited_map){
					pgmMapCheck base_edit_map = pgmMapCheck(base_map_with_edit_path);
					base_edit_map.write_new_base_occupancy_grid_map(new_updated_map.merge_result_with_edit(&base_edit_map));
				}

				delete_directory_content(new_joined_map_path);
			}
			
		}
		
	}
	else{
		std::cerr << "Please check your path to the initial/base map file! Base map file and/or costmaps (C,NC,UC,UNC) do not exist!" << std::endl;
	}

	return 0;
}


/*
_____________________________________________________________________________________________
		
		IMPLEMENTATION OF USED METHODS/FUNCTIONS
_____________________________________________________________________________________________
*/

//file_exists function
bool file_exists(const std::string& name) {
    std::ifstream f(name.c_str());
    return f.good();
}

//check_and_write_maps_in_directories function
string check_and_write_maps_in_directories(string new_map_name, string new_map_path, int max_maps_count){
	
	for(int safed_maps = 1; safed_maps <= max_maps_count; safed_maps++){
		
		string map_safe_name = (new_map_name + "_" + std::to_string(safed_maps));		//variable to store new map with index of which map it is (_1 is the first map _2 second and so on)
		string map_safe_search_path = (new_map_path + "/" + map_safe_name + ".pgm");		//path to corresondping map files in order to check if they already exist or not
		
		if(!file_exists(map_safe_search_path)){
			return map_safe_name;
		}
		if(safed_maps == max_maps_count){
			std::cout << "Maximum for maps in this directory: \"" << new_map_path << "\" is reached!" << std::endl;
			return "";									//if the maximum for used maps is reached we return a emtpy string
		}
	}
	std::cerr << "Checking maps in directory failed! Please check if the given directory exists and is right!" << std::endl;
	return "";
}

//delete_directory_content function
void delete_directory_content(const std::string& dir_path)
{
    for (const auto& entry : std::filesystem::directory_iterator(dir_path)){
	std::filesystem::remove_all(entry.path());
    }
        std::cout << "Deleted contents of \" " << dir_path << "\" directory!" << std::endl;

}

//search_maps_in_directory function
vector<string> search_maps_in_directory(string path_to_map_directory, string map_name_scheme){
	
	vector<string> path_to_found_maps{};

	for(auto& search_path: std::filesystem::directory_iterator(path_to_map_directory)){
		
		std::string file_name = search_path.path().filename();

		if(file_name.find(map_name_scheme) == 0){
			path_to_found_maps.push_back(path_to_map_directory + "/" + file_name);								//check same day maps and generate them if they are not aready generated
		}
	}

	return path_to_found_maps;
}