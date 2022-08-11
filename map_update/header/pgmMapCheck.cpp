//implementation of the pgmMapCheck class
#include "pgmMapCheck.h"

//implementing class constructor
pgmMapCheck::pgmMapCheck(string path_to_map_file)
	:map_file_path(path_to_map_file)
{
	extract_map_data();
}

//implementing class destructor
pgmMapCheck::~pgmMapCheck()
{
}

//implementing private functions
void pgmMapCheck::extract_map_data()
{
	std::ifstream map_read;					//ifstream object for reading in data (map)
	map_read.open(map_file_path, std::ios::in);		//open pgm file

	if(map_read.is_open()){ 				//check if open was succesfull
		
		std::getline(map_read, map_header_data[0]);	//get mode (for maps occupancy grid maps P5)

		if(map_header_data[0].compare("P5") != 0){ 	//check if pgm has right format
			std::cerr << "Expected PGM Version P5. Map cannot be received. \nVersion error" << std::endl;
		}
		else{
			std::getline(map_read, map_header_data[1]);	//get creator name and resolution of map
			std::getline(map_read, map_header_data[2]);	//get dimensions of the map
			std::getline(map_read, map_header_data[3]);	//get the grey value range used (normal 255)

			string occupancy_grid_values_string;
			std::getline(map_read, occupancy_grid_values_string);	//get all the pixel values coded in bytes for pgm type P5

			//create stringstream object in order to get resolution of the map
			std::stringstream resolution = std::stringstream(map_header_data[1]);
			//create stringstream object in order to get resolution of the map
			std::stringstream dimensions = std::stringstream(map_header_data[2]);

			//save resolution of the map
			string temp_unit_of_map_resolution;
			string map_res_helper = "";
			while(resolution >> temp_unit_of_map_resolution){
				std::stringstream(map_res_helper) >> map_res_m;
				map_res_helper = temp_unit_of_map_resolution;
			}
			
			std::cout << "Map resolution in m: " << (map_res_m) << std::endl; //entfernen nur temp
			//save dimensions of the map in x and y
			dimensions >> map_dim_x >> map_dim_y;
			std::cout << "map dimension x: " << map_dim_x << ", map dimension y: " << map_dim_y << " | " << std::endl; //entfernen

			//create temporary vector for filling in matrix
			vector<uint8_t> map_occupancy_row;

			//get all the grid values as matrix(254 = free, 0 = occupied, 205 = unknown)
			for(int i=0; i < occupancy_grid_values_string.length(); i++){
				if((i%map_dim_x == 0) && i != 0){ 				//adding rows to matrix
					map_occupancy_matrix.push_back(map_occupancy_row);
					map_occupancy_row.clear();
				}
				map_occupancy_row.push_back((unsigned char)(occupancy_grid_values_string[i]));
			}

			//adding last row to matrix
			map_occupancy_matrix.push_back(map_occupancy_row);
			map_occupancy_row.clear();

			//following only for testing
			//std::cout << map_occupancy_row.size() << std::endl;			//only relevevant for testing
			int zaehler_b= 0;
			int zaehler_g=0;
			int zaehler_w=0;
			
			//following check size again for safety and error search
			std::cout << "map_size_x: " << map_occupancy_matrix[0].size() << std::endl;
			std::cout << "map_size_y: " << map_occupancy_matrix.size() << std::endl;

			for(int i = 0; i< map_occupancy_matrix.size(); i++){
				for(int g = 0; g< map_occupancy_matrix[i].size(); g++){
				if(map_occupancy_matrix[i][g] == 0){
					zaehler_b++;
				}
				if(map_occupancy_matrix[i][g] == 205){
					zaehler_g++;
				}
				if(map_occupancy_matrix[i][g] == 254){
					zaehler_w++;
				}
				}
			}
			std::cout << "Count of occupied cells in the loaded map: " << zaehler_b << std::endl;
			std::cout << "Count of unknown cells in the loaded map: " << zaehler_g << std::endl;
			std::cout << "Count of free (unoccupied) cells in the loaded map: " << zaehler_w << std::endl;
			std::cout << "Total count of cells in the loaded map: " << zaehler_b+zaehler_g+zaehler_w << "\n" << std::endl;
			
			//map load was successful
			map_load_successful = true;

		}
	}
	else{
		map_load_successful = false;
		std::cerr << "Couldn't open file at: " << map_file_path << std::endl;
	}

}

//return x dimension
uint16_t pgmMapCheck::get_map_dim_in_pixel_x()
{
	return map_dim_x;
}

//retunr y dimension
uint16_t pgmMapCheck::get_map_dim_in_pixel_y()
{
	return map_dim_y;
}

//return loaded occupancy grid values
vector<vector<uint8_t>> pgmMapCheck::occupancy_grid_map()
{
	return map_occupancy_matrix;
}

//set the deadzone for the gridmap to use
void pgmMapCheck::set_grid_cell_deadzone(uint8_t cell_deadzone){
	grid_cell_deadzone = cell_deadzone;
}

//return grid_cell_deadzone
uint8_t pgmMapCheck::get_grid_cell_deadzone()
{
	return grid_cell_deadzone;
}

//write out new  map pgm file
//normally only needed for the base map / inital map file
bool pgmMapCheck::write_new_base_occupancy_grid_map(vector<vector<uint8_t>> compared_map)
{	
	std::ofstream map_write;

	if(map_load_successful){
		//load old occupancy grid map and wipe the whole file
		map_write.open(map_file_path, std::ios::trunc);

		//write the initial map header data received upon creating this object
		for(int i = 0; i< pgm_header_size_const; i++){
			map_write << map_header_data[i] << std::endl;
		}

		//write the received map grid values into new file (need to be char!)
		for(int i=0; i < compared_map.size(); i++){
			for(int a=0; a < compared_map[i].size(); a++){
				map_write << char(compared_map[i][a]);
			}
		}

		map_write.close();

		return true;
	}
	else{
		std::cerr << "Not able to execute function because initial loading of the map was not successful!" << std::endl;
		return false;
	}
}

//return matrix that shows if a pixel in the pgm map file is allowed to be changed
//this feature should only be used on the base_map/the initial map file!!!
vector<vector<bool>> pgmMapCheck::check_changable_grid_cells()
{
	//get all the grid values and give a value if the corresponding grid cell 
	//is allowed to be changed (254 = free, 0 = occupied, 205 = unknown) 
	//occupied cells in the inital map get a "not changable" radius in order to not get thicker walls over the time
	//INFO: not changable only applies to a cell being changed from free to occupied, occupied to free has to be always changable
	
	vector<vector<bool>> changable_grid_cell_matrix(map_dim_y, vector<bool> (map_dim_x, true)); 	//only needed temporary

	if(map_load_successful){
			
		//iterate through map as matrix and change the "allowed" matrix to the corresponding values
		
		for(int i=0; i < map_occupancy_matrix.size(); i++){
			for(int a=0; a < map_occupancy_matrix[i].size(); a++){
				//search for occupied cells
				if(map_occupancy_matrix[i][a] == 0){						//check for occupied cells
					//change values inside changable_grid_cell_matrix accordingly
					if(changable_grid_cell_matrix[i][a] == true){
						changable_grid_cell_matrix[i][a] = false;
					}
					//boolean values in order to check if indices are accessable
					bool left_index_callable = false;
					bool right_index_callable = false;
					bool upper_index_callable = false;
					bool lower_index_callable = false;

					for(int deadzone = 1; deadzone <= grid_cell_deadzone; deadzone++){
						if(i - deadzone >= 0){
							if(changable_grid_cell_matrix[i-deadzone][a] == true){
								changable_grid_cell_matrix[i-deadzone][a] = false;
							}
							if(deadzone == 1){
								upper_index_callable=true;
							}
						}
						else{
							break;
						}
					}

					for(int deadzone = 1; deadzone <= grid_cell_deadzone; deadzone++){
						if(a - deadzone >= 0){
							if(changable_grid_cell_matrix[i][a-deadzone] == true){
								changable_grid_cell_matrix[i][a-deadzone] = false;
							}
							if(deadzone == 1){
								left_index_callable=true;
							}
						}
						else{
							break;
						}
					}

					for(int deadzone = 1; deadzone <= grid_cell_deadzone; deadzone++){
						if(i + deadzone < map_occupancy_matrix.size()){
							if(changable_grid_cell_matrix[i+deadzone][a] == true){
								changable_grid_cell_matrix[i+deadzone][a] = false;
							}
							if(deadzone == 1){
								lower_index_callable=true;
							}
						}
						else{
							break;
						}
					}

					for(int deadzone = 1; deadzone <= grid_cell_deadzone; deadzone++){
						if(a + deadzone < map_occupancy_matrix[i].size()){
							if(changable_grid_cell_matrix[i][a+deadzone] == true){
								changable_grid_cell_matrix[i][a+deadzone] = false;
							}
							if(deadzone == 1){
								right_index_callable=true;
							}
						}
						else{
							break;
						}
					}

					if(upper_index_callable && right_index_callable){
						if(changable_grid_cell_matrix[i-1][a+1] == true){
							changable_grid_cell_matrix[i-1][a+1] = false;
						}
					}
					if(upper_index_callable && left_index_callable){
						if(changable_grid_cell_matrix[i-1][a-1] == true){
							changable_grid_cell_matrix[i-1][a-1] = false;
						}
					}
					if(lower_index_callable && right_index_callable){
						if(changable_grid_cell_matrix[i+1][a+1] == true){
							changable_grid_cell_matrix[i+1][a+1] = false;
						}
					}
					if(lower_index_callable && left_index_callable){
						if(changable_grid_cell_matrix[i+1][a-1] == true){
							changable_grid_cell_matrix[i+1][a-1] = false;
						}
					}

				}

			}
		}

		map_grid_changable_matrix = changable_grid_cell_matrix;
		return map_grid_changable_matrix;
	}
	else{
		std::cerr << "Not able to execute function because initial loading of the map was not successful!\n Be careful returned value is most certainly not right!!!" << std::endl;
		return changable_grid_cell_matrix;
	}

}

//show the changable gridcells in a pgm file
void pgmMapCheck::show_not_changable_cells(string map_destination_and_name)
{
	std::ofstream map_write;

	map_write.open(map_destination_and_name, std::ios_base::out);
	
	if(map_write.is_open()){
		//write the initial map header data received upon creating this object
		for(int i = 0; i< pgm_header_size_const; i++){
			map_write << map_header_data[i] << std::endl;
		}

		//write the received map grid values into new file (need to be char!)
		for(int i=0; i < map_grid_changable_matrix.size(); i++){
			for(int a=0; a < map_grid_changable_matrix[i].size(); a++){
				if(map_grid_changable_matrix[i][a] == true){
					map_write << char(map_occupancy_matrix[i][a]);
				}
				else{
					map_write << char(120);
				}
			}
		}

		std::cout << "Changable gridcells pgm generated!" << std::endl;

		map_write.close();
	}
	else{
		std::cerr << "Failed to open new file for showing changable gridcells!" << std::endl;
	}
}

//generate a (initial) map without occupied cells for checking if objects have been (re-)moved
void pgmMapCheck::write_unoccupied_map(string map_destination_folder, string path_to_base_map_yaml_file, uint8_t cell_deadzone)
{
	if(map_grid_changable_matrix.size() == 0){				//checking if unchangable matrix has already been generated
		set_grid_cell_deadzone(cell_deadzone);				//set deadzone to use
		check_changable_grid_cells();					//if not generate the unchangable map matrix
		std::cout << "No changable grid cell matrix, therefore generating one" << std::endl;
	}
	
	std::ifstream map_description_load; 					//loader for yaml file corresponding to base/initial map pgm file
	map_description_load.open(path_to_base_map_yaml_file, std::ios::in);	//load corresponding yaml file

	string base_map_yaml_file_data[6];					//array of strings to load YAML-file data

	if(map_description_load.is_open()){
		for(int i = 0; i<6; i++){
			std::getline(map_description_load, base_map_yaml_file_data[i]);					//get the data from the original base (initial) map file
		}
		string check_for_space = "";
		while(check_for_space != " "){
			check_for_space = base_map_yaml_file_data[0][base_map_yaml_file_data[0].length()-1];
			base_map_yaml_file_data[0].pop_back();
		};
		base_map_yaml_file_data[0] = base_map_yaml_file_data[0] + " base_map_without_occupied_cells.pgm";	//created pgm file will always be called base_map_without_occupied_cells.pgm

	}
	else{
		std::cerr << "Could not open corresponding YAML file to the base (initial) map file at: " << path_to_base_map_yaml_file << " for reading!" << std::endl;
	}

	std::ofstream map_write_all;											//object for writing the new yaml file for the base_image_wihtout_occupied_cells.pgm file
	map_write_all.open((map_destination_folder +  "/base_map_without_occupied_cells.yaml"), std::ios_base::out);	//open new file for writing base_map_without_occupied_cells.yaml
	
	if(map_write_all.is_open()){
		//write the initial map header data received upon creating this object
		for(int i = 0; i<6; i++){
			map_write_all << base_map_yaml_file_data[i] << std::endl;
		}

		std::cout << "YAML file for base (initial) map without occupied cells generated! (File name: base_map_without_occupied_cells.yaml)" << std::endl;

		map_write_all.close();
	}
	else{
		std::cerr << "Failed to open new file for generating base_map_without_occupied_cells.yaml! file" << std::endl;
	}

	map_write_all.open((map_destination_folder +  "/base_map_without_occupied_cells.pgm"), std::ios_base::out);	//open new file for writing base_map_without_occupied_cells.pgm
	
	if(map_write_all.is_open()){
		//write the initial map header data received upon creating this object
		for(int i = 0; i< pgm_header_size_const; i++){
			map_write_all << map_header_data[i] << std::endl;
		}

		//write the received map grid values into new file (need to be char!)
		for(int i=0; i < map_occupancy_matrix.size(); i++){
			for(int a=0; a < map_occupancy_matrix[i].size(); a++){
				
				if(map_grid_changable_matrix[i][a]){
					map_write_all << char(map_occupancy_matrix[i][a]);
				}
				else{
					map_write_all << char(205);
				}
			}
		}

		std::cout << "Generated base (initial) map without occupied cells for searching for (re-)moved objects! (File name: base_map_without_occupied_cells.pgm)" << std::endl;

		map_write_all.close();
	}
	else{
		std::cerr << "Failed to open new file for generating base_map_without_occupied_cells.pgm! file" << std::endl;
	}

}
