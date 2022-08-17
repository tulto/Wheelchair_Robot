//implementation of the pgmMapJoin class

#include "pgmMapJoin.h"

//implementing class constructor
pgmMapJoin::pgmMapJoin(pgmMapCheck *passed_initial_map, vector<pgmMapCheck> *passed_maps)
{
	count_maps_to_join = passed_maps->size();

	if(check_count()){

		for(int i = 0; i < count_maps_to_join; i++){
			maps_to_join.push_back(passed_maps->at(i).occupancy_grid_map());
		}
		
		initial_map = passed_initial_map->occupancy_grid_map();
		resulting_map = passed_initial_map->occupancy_grid_map();
		join_maps_to_resulting_map_for_the_day();
	}
}

//implementing destructor
pgmMapJoin::~pgmMapJoin()
{

}


bool pgmMapJoin::check_count()
{
	if(count_maps_to_join > 1){
		return true;
	}
	else{
		return false;
	}
}


//implementing map size check
bool pgmMapJoin::check_compatible_map_sizes()
{
	for(int i = 0; i < count_maps_to_join; i++){
		
		if(initial_map.size() == maps_to_join.at(i).size()){
			if(initial_map.size() > 0){
				if(initial_map.at(0).size() == maps_to_join.at(i).at(0).size()){
					continue;
				}
				else{
					std::cerr << "Sizes of the given maps are note the same/compatible. Make sure all the given maps originate from one base/initial map and have the same x and y size!" << std::endl;
					return false;
				}
			}
			else{	
				std::cout << "Map sizes are zero 0. Map is completely empty (PGM file is empty)" << std::endl;
			}
		}
		else{
			std::cerr << "Sizes of the given maps are note the same/compatible. Make sure all the given maps originate from one base/initial map and have the same x and y size!" << std::endl;
			return false;
		}
	}
	
	return true;
}


//implementing the function which joins all maps to one resulting map
void pgmMapJoin::join_maps_to_resulting_map_for_the_day()
{
	if(check_compatible_map_sizes()){
		
		for(int i=0; i < initial_map.size(); i++){				//go through all rows
			for(int a = 0; a < initial_map.at(i).size(); a++){			//go through all coloumns
				
				for(int map = 0; map < count_maps_to_join; map++){	//compare with all passed maps to join
											
					if(initial_map.at(i)[a] != maps_to_join.at(map).at(i)[a]){
						
						if(resulting_map[i][a] != maps_to_join.at(map).at(i)[a]){
							
							resulting_map[i][a] = maps_to_join.at(map).at(i)[a];
							
						}
						
					}
					
				}
			}
		}
	}
}

//implement function to give out the generated joined map from different drives
vector<vector<uint8_t>> pgmMapJoin::join_result_map()
{
	return resulting_map;
}

//return the count of used maps to join

uint8_t pgmMapJoin::get_count_maps_to_join()
{
	return count_maps_to_join;
}

bool pgmMapJoin::write_new_joined_map(string path_to_new_map_file_loc, string name_of_new_map){

	std::ofstream map_write_all;											//object for writing the new yaml file from the compared maps
	
	map_write_all.open((path_to_new_map_file_loc +  "/" + name_of_new_map + ".pgm"), std::ios_base::out);	//open new file for writing new pgm file for compared map	
	
	if(map_write_all.is_open()){
				
		if(check_compatible_map_sizes()){

			//write the initial map header data received upon creating this object
			map_write_all << "P5" << std::endl;
			map_write_all << "# CREATOR: pgmMapJoin class" << std::endl;
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