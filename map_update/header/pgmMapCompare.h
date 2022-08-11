#ifndef pgmMapCompare_H
#define pgmMapCompare_H

#include "pgmMapCheck.h"
#include <array>
#include <list>

using std::string;
using std::vector;
using std::array;
using std::list;

//class for comparing different pgm map files generated simultaneously and generating
//a new updated map of the environment
//INFO: In pgm files stored with map_server node the grid values are 254=free, 205=unknown and 0=occupied

class pgmMapCompare
{
	private:
	//general parameters of the class
	vector<vector<bool>> changable_grid_matrix;			//matrix showing which grids are changable
	vector<vector<uint8_t>> initial_map;				//loaded base/initial map
	vector<vector<uint8_t>> no_clearing_map;			//loaded map with no_clearing allowed
	vector<vector<uint8_t>> clearing_map;				//map with clearing allowed
	vector<vector<uint8_t>> unoccupied_cost_base_map;		//base/initial map without occupied cells that has been changed by the costmap
	vector<vector<uint8_t>> resulting_map;				//resulting map after comparison
	
	//define private functions
	bool check_compatible_map_sizes();				//check if all loaded maps have the same x and y size (otherwise there is an error)
	bool check_unoccupied_map_sizes(vector<vector<uint8_t>> unoccupied_c, vector<vector<bool>> unoccupied_nc);				//chceck if the unoccupied maps have the same size in x and y 
	void extract_coordinates_from_maps();				//get coordinates of occupied cells from map with clearing enabled
	void find_new_object_grid_cell_groups(array<uint16_t, 2> coord);//find group of grid cells which belong to one object and mark/input them into the resulting map
	void find_removed_objects_grid_cell_groups(array<uint16_t, 2> coord); 	//find group of grid cells which belong to one object and mark/input them into the resulting map
	vector<vector<uint8_t>> concatenate_unoccupied_maps(vector<vector<uint8_t>> unoccupied_c, vector<vector<bool>> unoccupied_nc);	//function to summarize the unoccupied costmaps with clearing enabled and disabled with a safety zone around the unoccupied_no_clearing map(--> putting all black cells in the unoccupied clearing grid). Needed for right removal of removed objects
	
	public:
	//define constructor and deconstructor
	pgmMapCompare(pgmMapCheck *init_map, pgmMapCheck *no_clearing, pgmMapCheck *clearing,  pgmMapCheck *unoccupied_clearing, pgmMapCheck *unoccupied_no_clearing, uint8_t remove_object_safe_dist = 1); //remove_object_safe_dist used as a safety so we have a layer around not cleared objects in order to not falsely remove a pixel from a wall (make occupied wall pixel to free pixel)
	~pgmMapCompare();

	//public functions
	vector<vector<uint8_t>> comparison_result_map();
	bool write_new_compared_map(string path_to_new_map_file_loc, string name_of_new_map);




};

#endif