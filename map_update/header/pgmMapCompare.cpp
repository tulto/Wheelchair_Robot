//implementation of the pgmMapComparison class
#include "pgmMapCompare.h"

//implementing class constructor
pgmMapCompare::pgmMapCompare(pgmMapCheck *init_map, pgmMapCheck *no_clearing, pgmMapCheck *clearing, pgmMapCheck *unoccupied_clearing, pgmMapCheck *unoccupied_no_clearing, uint8_t remove_object_safe_dist)
{
	changable_grid_matrix = init_map->check_changable_grid_cells();
	initial_map = init_map->occupancy_grid_map();
	no_clearing_map = no_clearing->occupancy_grid_map();
	clearing_map = clearing->occupancy_grid_map();
	unoccupied_no_clearing->set_grid_cell_deadzone(remove_object_safe_dist);										//set the remove object safety distance to the unoccupied_no_clearing map for better safety towards deleting wrong cells (e.g. walls)	
	unoccupied_cost_base_map = concatenate_unoccupied_maps(unoccupied_clearing->occupancy_grid_map(), unoccupied_no_clearing->check_changable_grid_cells());
	resulting_map = init_map->occupancy_grid_map();
	extract_coordinates_from_maps();
}

//implementing destructor
pgmMapCompare::~pgmMapCompare()
{

}

//implementing map size check
bool pgmMapCompare::check_compatible_map_sizes()
{
	if(initial_map.size() == clearing_map.size() && initial_map.size() == no_clearing_map.size() && initial_map.size() == unoccupied_cost_base_map.size() && initial_map.size() == changable_grid_matrix.size()){
		if(initial_map.size() > 0){
			if(initial_map.at(0).size() == clearing_map.at(0).size() && initial_map.at(0).size() == no_clearing_map.at(0).size() && initial_map.at(0).size() == unoccupied_cost_base_map.at(0).size() && initial_map.at(0).size() == changable_grid_matrix.at(0).size()){
				
				return true;
			}
			else{
				std::cerr << "Sizes of the given maps are note the same/compatible. Make sure all the given maps originate from one base/initial map and have the same x and y size!" << std::endl;
				return false;
			}
		}
		else{	
			std::cout << "Map sizes are zero 0. Map is completly empty (PGM file is empty)" << std::endl;
			return true;
		}
	}
	else{
		std::cerr << "Sizes of the given maps are note the same/compatible. Make sure all the given maps originate from one base/initial map and have the same x and y size!" << std::endl;
		return false;
	}
}

//implementing coodinate extract function
void pgmMapCompare::extract_coordinates_from_maps()
{	
	if(check_compatible_map_sizes()){

		array<uint16_t, 2> coordinate = {0,0};					//coordinates in y and x [y, y]

		for(int i=0; i < clearing_map.size(); i++){
			for(int a=0; a < clearing_map.at(i).size(); a++){
				
				if(initial_map.at(i)[a] == 0){

					if(!(changable_grid_matrix.at(i)[a])){						//just used as a safety feature because normally all occupied cells shouldnt be changable 

						if(unoccupied_cost_base_map.at(i)[a] == 254){			//check if the initial map has an occupied cell where the unoccupied 								
							
							if(resulting_map[i][a] == 0){					//also check if the resulting map for this pixel might already been changed
								coordinate = {(uint16_t)i, (uint16_t)a};			//and updated grid map (after costmap update) has a free cell
								find_removed_objects_grid_cell_groups(coordinate);
							}
						}
					}
				}
				
				if(clearing_map.at(i)[a] == 0){
					
					if(changable_grid_matrix.at(i)[a]){			//only (possible) new object coordinates should be saved therefore
												//we check if the inital map already had an occupied cell on the same coordinate
												//or nearby (because of the measurement error of the lidar) which we can get from the changable_grid_matrix 
						
						if(no_clearing_map.at(i)[a] == 0){		//also check if the no_clearing allowed map has an occupied cell here (just for safety)
							
							if(resulting_map[i][a] != 0){		//checking if the now used occupied cell has already been put into the resulting map
								coordinate = {(uint16_t)i, (uint16_t)a};
								find_new_object_grid_cell_groups(coordinate);
							}
						}
					}
				}
			}
		}
	}
	else{
		std::cerr << "Base/Initial Map, Clearing Map and No Clearing Map are not compatible. Error: Different Map sizes!! \n Could not extract coordinates from maps!" << std::endl;
	}
	
}

//implementing function to find groups of cells which belong to a new object
//this function is a recursice function
void pgmMapCompare::find_new_object_grid_cell_groups(array<uint16_t, 2> coord)
{
	if(changable_grid_matrix.at(coord[0])[coord[1]]){					//check if the given coordinate is changable

		resulting_map[coord[0]][coord[1]] = no_clearing_map.at(coord[0])[coord[1]];	//change given cell to the cell value of the no_clearing_map
		
		bool left=false, right=false, upper=false, lower=false;				//boolean values to search for not matching occupied cells in the resulting and no_clearing_map
		
		array<uint16_t, 2> coordinate; 							//variable to store the coordinate of an "unknown" grid cell for the next rekursive call of this function
		
		//following we checkt that the index we want to call is not out of bounds
		if((coord[0]-1) >= 0){
			upper = true;
		}
		if((coord[1]-1) >= 0){
			left = true;
		}
		if((coord[0]+1) < resulting_map.size()){
			lower = true;
		}
		if((coord[1]+1) < resulting_map[coord[0]].size()){
			right = true;
		}

		//now we search for changable and not equal occupied cells in the resluting and no clearing map 
		//here we search in a certain pattern which is: 	6 | 7 | 8
		//							5 | 0 | 1
		//							4 | 3 | 2
		if(right){

			if(changable_grid_matrix.at(coord[0])[coord[1]+1]){
				if(!((resulting_map[coord[0]][coord[1]+1]) == (no_clearing_map.at(coord[0])[coord[1]+1]))){
					coordinate = {coord[0], (uint16_t)(coord[1]+1)}; 
					find_new_object_grid_cell_groups(coordinate);
				}
			}
			
			if(lower){
				if(changable_grid_matrix.at(coord[0]+1)[coord[1]+1]){
					if(!((resulting_map[coord[0]+1][coord[1]+1]) == (no_clearing_map.at(coord[0]+1)[coord[1]+1]))){
						coordinate = {(uint16_t)(coord[0]+1), (uint16_t)(coord[1]+1)};
						find_new_object_grid_cell_groups(coordinate);
					}
				}
			}
		}

		if(lower){

			if(changable_grid_matrix.at(coord[0]+1)[coord[1]]){
				if(!((resulting_map[coord[0]+1][coord[1]]) == (no_clearing_map.at(coord[0]+1)[coord[1]]))){
					coordinate = {(uint16_t)(coord[0]+1), (coord[1])};
					find_new_object_grid_cell_groups(coordinate);
				}
			}

			if(left){
				if(changable_grid_matrix.at(coord[0]+1)[coord[1]-1]){
					if(!((resulting_map[coord[0]+1][coord[1]-1]) == (no_clearing_map.at(coord[0]+1)[coord[1]-1]))){
						coordinate = {(uint16_t)(coord[0]+1), (uint16_t)(coord[1]-1)};
						find_new_object_grid_cell_groups(coordinate);
					}
				}
			}
		}

		if(left){

			if(changable_grid_matrix.at(coord[0])[coord[1]-1]){
				if(!((resulting_map[coord[0]][coord[1]-1]) == (no_clearing_map.at(coord[0])[coord[1]-1]))){
					coordinate = {coord[0], (uint16_t)(coord[1]-1)}; 
					find_new_object_grid_cell_groups(coordinate);
				}
			}

			if(upper){
				if(changable_grid_matrix.at(coord[0]-1)[coord[1]-1]){
					if(!((resulting_map[coord[0]-1][coord[1]-1]) == (no_clearing_map.at(coord[0]-1)[coord[1]-1]))){
						coordinate = {(uint16_t)(coord[0]-1), (uint16_t)(coord[1]-1)};
						find_new_object_grid_cell_groups(coordinate);
					}
				}
			}
		}

		if(upper){

			if(changable_grid_matrix.at(coord[0]-1)[coord[1]]){
				if(!((resulting_map[coord[0]-1][coord[1]]) == (no_clearing_map.at(coord[0]-1)[coord[1]]))){
					coordinate = {(uint16_t)(coord[0]-1), (coord[1])};
					find_new_object_grid_cell_groups(coordinate);
				}
			}

			if(right){
				if(changable_grid_matrix.at(coord[0]-1)[coord[1]+1]){
					if(!((resulting_map[coord[0]-1][coord[1]+1]) == (no_clearing_map.at(coord[0]-1)[coord[1]+1]))){
						coordinate = {(uint16_t)(coord[0]-1), (uint16_t)(coord[1]+1)};
						find_new_object_grid_cell_groups(coordinate);
					}
				}
			}
		}


	}
	
}

//implementing function to find groups of cells which belong to a (re-)moved object
//this function is a recursice function
void pgmMapCompare::find_removed_objects_grid_cell_groups(array<uint16_t, 2> coord)
{
	if(!(changable_grid_matrix.at(coord[0])[coord[1]])){							//check if given coordinate is normally not changable because (before) occupied cells should not be changable (plus an unchangable radius)

		resulting_map[coord[0]][coord[1]] = unoccupied_cost_base_map.at(coord[0])[coord[1]];	//change given cell to the cell value of the unoccupied_cost_base_map
		//above is a mistake for the removing object class (will work on adding objects but not on removing objects)
		bool left=false, right=false, upper=false, lower=false;						//boolean values to search for not matching occupied cells in the resulting and unoccupied_cost_base_map
		
		array<uint16_t, 2> coordinate; 									//variable to store the coordinate of an "unknown" grid cell for the next rekursive call of this function
		
		//following we checkt that the index we want to call is not out of bounds
		if((coord[0]-1) >= 0){
			upper = true;
		}
		if((coord[1]-1) >= 0){
			left = true;
		}
		if((coord[0]+1) < resulting_map.size()){
			lower = true;
		}
		if((coord[1]+1) < resulting_map[coord[0]].size()){
			right = true;
		}

		//now we search for not changable and not equal occupied cells in the resluting and unoccupied map 
		//here we search in a certain pattern which is: 	6 | 7 | 8
		//							5 | 0 | 1
		//							4 | 3 | 2
		if(right){

			if(!(changable_grid_matrix.at(coord[0])[coord[1]+1])){
				if((resulting_map[coord[0]][coord[1]+1] == 0) && (unoccupied_cost_base_map.at(coord[0])[coord[1]+1] == 254)){
					coordinate = {coord[0], (uint16_t)(coord[1]+1)}; 
					find_removed_objects_grid_cell_groups(coordinate);
				}
			}
			
			if(lower){
				if(!(changable_grid_matrix.at(coord[0]+1)[coord[1]+1])){
					if((resulting_map[coord[0]+1][coord[1]+1] == 0) && (unoccupied_cost_base_map.at(coord[0]+1)[coord[1]+1] == 254)){
						coordinate = {(uint16_t)(coord[0]+1), (uint16_t)(coord[1]+1)};
						find_removed_objects_grid_cell_groups(coordinate);
					}
				}
			}
		}

		if(lower){

			if(!(changable_grid_matrix.at(coord[0]+1)[coord[1]])){
				if((resulting_map[coord[0]+1][coord[1]] == 0) && (unoccupied_cost_base_map.at(coord[0]+1)[coord[1]] == 254)){
					coordinate = {(uint16_t)(coord[0]+1), (coord[1])};
					find_removed_objects_grid_cell_groups(coordinate);
				}
			}

			if(left){
				if(!(changable_grid_matrix.at(coord[0]+1)[coord[1]-1])){
					if((resulting_map[coord[0]+1][coord[1]-1] == 0) && (unoccupied_cost_base_map.at(coord[0]+1)[coord[1]-1] == 254)){
						coordinate = {(uint16_t)(coord[0]+1), (uint16_t)(coord[1]-1)};
						find_removed_objects_grid_cell_groups(coordinate);
					}
				}
			}
		}

		if(left){

			if(!(changable_grid_matrix.at(coord[0])[coord[1]-1])){
				if((resulting_map[coord[0]][coord[1]-1] == 0) && (unoccupied_cost_base_map.at(coord[0])[coord[1]-1] == 254)){
					coordinate = {coord[0], (uint16_t)(coord[1]-1)}; 
					find_removed_objects_grid_cell_groups(coordinate);
				}
			}

			if(upper){
				if(!(changable_grid_matrix.at(coord[0]-1)[coord[1]-1])){
					if((resulting_map[coord[0]-1][coord[1]-1] == 0) && (unoccupied_cost_base_map.at(coord[0]-1)[coord[1]-1] == 254)){
						coordinate = {(uint16_t)(coord[0]-1), (uint16_t)(coord[1]-1)};
						find_removed_objects_grid_cell_groups(coordinate);
					}
				}
			}
		}

		if(upper){

			if(!(changable_grid_matrix.at(coord[0]-1)[coord[1]])){
				if((resulting_map[coord[0]-1][coord[1]] == 0) && (unoccupied_cost_base_map.at(coord[0]-1)[coord[1]] == 254)){
					coordinate = {(uint16_t)(coord[0]-1), (coord[1])};
					find_removed_objects_grid_cell_groups(coordinate);
				}
			}

			if(right){
				if(!(changable_grid_matrix.at(coord[0]-1)[coord[1]+1])){
					if((resulting_map[coord[0]-1][coord[1]+1] == 0) && (unoccupied_cost_base_map.at(coord[0]-1)[coord[1]+1] == 254)){
						coordinate = {(uint16_t)(coord[0]-1), (uint16_t)(coord[1]+1)};
						find_removed_objects_grid_cell_groups(coordinate);
					}
				}
			}
		}


	}
}

//implementing function to check if the whole area around a certain grid cell is known
//known means unchangable, free or already "checked"/changed in the resulting map
/*bool pgmMapCompare::grid_cell_neighbourhood_completely_known(array<uint16_t, 2> coord)
{
}
*/

//implement function to return new initial/base map
vector<vector<uint8_t>> pgmMapCompare::comparison_result_map()
{
	return resulting_map;
}

//implementing function to make sure the unoccupied grid maps (which are passed to the class constructor) have the same size
bool pgmMapCompare::check_unoccupied_map_sizes(vector<vector<uint8_t>> unoccupied_c, vector<vector<bool>> unoccupied_nc)
{
	if(unoccupied_c.size() == unoccupied_nc.size()){
		if(unoccupied_c.size() > 0){
			if(unoccupied_c.at(0).size() == unoccupied_nc.at(0).size()){
				
				return true;
			}
			else{
				std::cerr << "Sizes of the given unoccupied maps are note the same/compatible. Make sure all the given maps originate from one base/initial map and have the same x and y size!" << std::endl;
				return false;
			}
		}
		else{	
			std::cout << "Map sizes are zero 0. Map is completly empty (PGM file is empty)" << std::endl;
			return true;
		}
	}
	else{
		std::cerr << "Sizes of the given unoccupied maps are note the same/compatible. Make sure all the given maps originate from one base/initial map and have the same x and y size!" << std::endl;
		return false;
	}
	
}

//implement function to concatenate two unoccupied costmaps (with and without clearing enabled)
vector<vector<uint8_t>> pgmMapCompare::concatenate_unoccupied_maps(vector<vector<uint8_t>> unoccupied_c, vector<vector<bool>> unoccupied_nc)
{
	if(check_unoccupied_map_sizes(unoccupied_c, unoccupied_nc)){
		for(int i = 0; i < unoccupied_c.size(); i++){
			for(int a = 0; a < unoccupied_c.at(i).size(); a++){
				if(unoccupied_nc[i][a] == false){
					unoccupied_c[i][a] = 0;			//mark cell as occupied
				}
			}
		}
		return unoccupied_c;
	}
	else{
		std::cerr << "Given unoccupied maps have not the same size!!! Returned matrix is not right (Size x=1, y=1 and pgm value 100)! Please make sure all the given costmaps originate from the same base/initial map!" << std::endl;
		vector<vector<uint8_t>> false_map(1, vector<uint8_t> (1, 100));
		return false_map;
	}
}

//implement function which will be used to generate a new map as a .pgm and .yaml file that is created by the comparison of the maps
bool pgmMapCompare::write_new_compared_map(string path_to_new_map_file_loc, string name_of_new_map){

	std::ofstream map_write_all;											//object for writing the new yaml file from the compared maps
	
	map_write_all.open((path_to_new_map_file_loc +  "/" + name_of_new_map + ".pgm"), std::ios_base::out);	//open new file for writing new pgm file for compared map	
	
	if(map_write_all.is_open()){
				
		if(check_compatible_map_sizes()){

			//write the initial map header data received upon creating this object
			map_write_all << "P5" << std::endl;
			map_write_all << "# CREATOR: pgmMapCompare class" << std::endl;
			map_write_all << resulting_map.at(0).size() << " " << resulting_map.size() << std::endl;
			map_write_all << 255 << std::endl;

			//write the received map grid values into new file (need to be char!)
			for(int i=0; i < resulting_map.size(); i++){
				for(int a=0; a < resulting_map[i].size(); a++){
					
					map_write_all << resulting_map[i][a];
				}
			}

			std::cout << "Generated new map file (File name: " << name_of_new_map << ".pgm)" << std::endl;
		}
		map_write_all.close();

	}
	else{
		std::cerr << "Failed to open new file for generating " << name_of_new_map << ".pgm" << std::endl;
		return false;
	}

	return true;
}

