#ifndef pgmMapCheck_H
#define pgmMapCheck_H

#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#define pgm_header_size_const 4				//pgm files have a header constisting of 4 lines, which we want to extract

//class for reading in and checking pgm occupancy grid files
//INFO: In pgm files stored with map_server node the grid values are 254=free, 205=unknown and 0=occupied

using std::string;
using std::vector;


//class definition
class pgmMapCheck
{
	private:
	//general parameters
	string map_file_path;				//path to the occupancy grid map
	bool map_load_successful = false;		//variable for checking if loading of map was successful
	uint8_t grid_cell_deadzone = 3;			//variable for generating a deadzone around occupied cells 
							//in the initial map (onlx x and y diagonal is always 1 cell)

	//pgm file / map file parameters
	string map_header_data[pgm_header_size_const];	//pgm file header data
	vector<vector<uint8_t>> map_occupancy_matrix;	//matrix with the occupancy values (reperesented like pgm picture in x and y)
	vector<vector<bool>> map_grid_changable_matrix;	//matrix which has boolean values for representing if changing
							//a certain grid (in x and y) is allowed, only needed for the base (initial made) map
							//true = changing is allowed, false = changing is not allowed
	uint16_t map_dim_x = 0, map_dim_y = 0;		//variables to store the map dimensions extracted for pgm header
	float map_res_m;				//store extraxted map resolution from file

	//private functions
	void extract_map_data();			//extracting pgm file (map) data and saving them into the class parameters


	public:
	//declaring constructor and destructor (no default constructor without parameters)
	pgmMapCheck(string path_to_map_file);
	~pgmMapCheck();

	//public functions for interaction with pgmMapCheck class
	uint16_t get_map_dim_in_pixel_x();
	uint16_t get_map_dim_in_pixel_y();
	void set_grid_cell_deadzone(uint8_t cell_deadzone);				//set currently used grid cell deadzone
	uint8_t get_grid_cell_deadzone();						//get currently used grid cell deadzone
	vector<vector<bool>> check_changable_grid_cells();				//matrix which has boolean values for representing if changing
												//a certain grid (in x and y) is allowed, only needed for the base (initial made) map
												//true = changing is allowed, false = changing is not allowed
	vector<vector<uint8_t>> occupancy_grid_map();						//receiving the occupancy grid map data 
	bool write_new_base_occupancy_grid_map(vector<vector<uint8_t>> compared_map);	//writing the new, compared map to the pgm file (only needed for base/map)
	void show_not_changable_cells(string map_destination_and_name);			//only used for checking
	void write_unoccupied_map(string map_destination_folder, string path_to_base_map_yaml_file, uint8_t cell_deadzone = 3);	//write a version of the loaded map without occupied cells (only needed for base/initial map)
	


};


#endif