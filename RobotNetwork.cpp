#include "RobotNetwork.h"

RobotNetwork::RobotNetwork():
	world_grid(GRID_SIZE),
	num_robots(1),
	steps_to_success(0)
{
	Robot* bot = new Robot(0,0,0,GRID_SIZE);
	robots.push_back(bot);
	world_grid.changeCellValue(0,0,1);
	robots[0]->robot_grid.changeCellValue(0,0,1);
	pulseSensor(0,0,SENSOR_RANGE, 0);
	std::cout << "Network Initialized\n";
}

RobotNetwork::RobotNetwork(int n):
	world_grid(GRID_SIZE),
	num_robots(n),
	steps_to_success(0)
{
	
	for(int i = 0; i < num_robots; i++){
		Robot* bot = new Robot(i, 0, 0, GRID_SIZE);
		robots.push_back(bot);
		world_grid.changeCellValue(i,0,1);
		robots[i]->robot_grid.changeCellValue(i,0,1);
		pulseSensor(robots[i]->x, robots[i]->y, SENSOR_RANGE, i);

	}
	std::cout << "Network Initialized\n";
}

RobotNetwork::~RobotNetwork(){
	for(int i = 0; i < num_robots; i++){
		delete robots[i];
	}
	std::cout << "Network Destroyed\n";
}

// Able to work for multiple robots
void RobotNetwork::exploreGrid_Random(){
	srand(time(0));
	while(!(world_grid.gridExplored())){
		for(int i = 0; i < num_robots; i++){
			robots[i]->state = rand() % 4 + 1;
			if(robots[i]->state == 1){
				robots[i]->x++;	// Move Right
			}
			else if(robots[i]->state == 2){
				robots[i]->x--;	// Move Left
			}
			else if(robots[i]->state == 3){
				robots[i]->y++;	// Move Down
			}
			else if(robots[i]->state == 4){
				robots[i]->y--;	// Move Up
			}

			if(updateMap(i)){
				steps_to_success++;
			}
		}
	}

	std::cout << "It took RANDOM algorithm " << steps_to_success << " steps to cover the space.\n";
}

// Check if robot r's move is valid and update map accordingly
bool RobotNetwork::updateMap(int r){
	// Check if the robot is going to collide with another robot
	for(int i = 0; i < num_robots; i++){
		if(r != i && robots[i]->x == robots[r]->x && robots[i]->y == robots[r]->y){
			if(robots[r]->state == 1){
				robots[r]->x--;
			}
			else if(robots[r]->state == 2){
				robots[r]->x++;
			}
			else if(robots[r]->state == 3){
				robots[r]->y--;
			}
			else if(robots[r]->state == 4){
				robots[r]->y++;
			}
			return false;
		}
	}
	// Check that the robot is within the bounds of the map
	if(robots[r]->x == GRID_SIZE){
		robots[r]->x--;
		return false;
	}
	if(robots[r]->x == -1){
		robots[r]->x++;
		return false;
	}
	if(robots[r]->y == GRID_SIZE){
		robots[r]->y--;
		return false;
	}
	if(robots[r]->y == -1){
		robots[r]->y++;
		return false;
	}

	// Update maps and have robot takes sensor measurements
	world_grid.changeCellValue(robots[r]->x, robots[r]->y, 1);
	robots[r]->robot_grid.changeCellValue(robots[r]->x, robots[r]->y, 1);
	pulseSensor(robots[r]->x, robots[r]->y, SENSOR_RANGE, r);
	robots[r]->robot_grid.findFrontiers();
	world_grid.findFrontiers();

	std::cout << "World Map:\n";
	world_grid.printGrid();
	std::cout << "Robot " << r << " Map:\n";
	robots[r]->robot_grid.printGrid();

	return true;
}

// Flood fill to simulate sensor measurements
void RobotNetwork::pulseSensor(int x, int y, int d, int r){
	if(d == 0) return;
	if(x == GRID_SIZE || x == -1 || y == GRID_SIZE || y == -1) return;

	world_grid.changeCellValue(x, y, 1);
	robots[r]->robot_grid.changeCellValue(x, y, 1);
	pulseSensor(x + 1, y, d - 1, r);
	pulseSensor(x - 1, y, d - 1, r);
	pulseSensor(x, y + 1, d - 1, r);
	pulseSensor(x, y - 1, d - 1, r);
}