// This class is the main container in the system.  It maintains all of the robots
// in the system and the absolute map.  It is also where the random movement algorithm
// is implemented and where object and edge of map avoidance algorithms are implemented.

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
		robots[i]->robot_grid.findFrontiers();
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

void RobotNetwork::exploreGrid_Bidding(){
	int moves = 0;
	std::vector< std::pair< std::pair<int, int>, double > >::iterator it;
	std::pair< std::pair<int, int>, double> goal = std::make_pair(std::make_pair(0,0), 0.0);
	std::vector< std::pair< std::pair<int, int>, double> > goals;

	for(int i = 0; i < num_robots; i++){
		updateMap(i);
	}

	while(!(world_grid.gridExplored())){
		goals.clear();
		// Bidding Session
			// Update all bids
		for(int i = 0; i < num_robots; i++){
			robots[i]->updateBids();
		}
			// Get every robot's max bid
		for(int i = 0; i < num_robots; i++){
			std::cout << "Checking Robot " << i + 1 << " Bids\n";
			for(it = robots[i]->bids.begin(); it != robots[i]->bids.end(); it++){
				std::cout << "(" << it->first.first << "," << it->first.second << ") " << it->second << "\n";
				if(goal.second < it->second){
					goal.first.first = it->first.first;
					goal.first.second = it->first.second;
					goal = std::make_pair(std::make_pair(it->first.first, it->first.second), it->second);
				}
			}
			goals.push_back(goal);
			goal = std::make_pair(std::make_pair(0,0), 0.0);
		}

		// Moving Session
		for(int i = 0; i < num_robots; i++){
			moves = moveTo(goals[i].first.first, goals[i].first.second, i);
			if(updateMap(i)){
				steps_to_success += moves;
			}
			if(world_grid.gridExplored()) break;
		}

		// Sensing Session within the map update
	}

	std::cout << "It took the Bidding Algorithm " << steps_to_success << " steps to cover the space.\n";
}

// Check if robot r's move is valid and update map accordingly
bool RobotNetwork::updateMap(int r){
	// Check if the robot is going to collide with another robot
	for(int i = 0; i < num_robots; i++){
		if(r != i && robots[i]->x == robots[r]->x && robots[i]->y == robots[r]->y){
			std::cout << "Error: Robot " << r + 1 << " will hit Robot " << i + 1 << "\n";
			robots[r]->x = robots[r]->prev_x;
			robots[r]->y = robots[r]->prev_y;
			return false;
		}
	}
	// Check that the robot is within the bounds of the map
	if(robots[r]->x == GRID_SIZE || robots[r]->x == -1 || robots[r]->y == GRID_SIZE || robots[r]->y == -1){
		std::cout << "Error: Robot " << r + 1 << " will be Out-Of-Bounds\n";
		robots[r]->x = robots[r]->prev_x;
		robots[r]->y = robots[r]->prev_y;
		return false;
	}

	// Update maps and have robot takes sensor measurements
	world_grid.changeCellValue(robots[r]->x, robots[r]->y, 3);
	world_grid.changeCellValue(robots[r]->prev_x, robots[r]->prev_y, 1);
	robots[r]->robot_grid.changeCellValue(robots[r]->x, robots[r]->y, 3);
	robots[r]->robot_grid.changeCellValue(robots[r]->prev_x, robots[r]->prev_y, 1);
	pulseSensor(robots[r]->x, robots[r]->y, SENSOR_RANGE, r);
	robots[r]->robot_grid.findFrontiers();
	world_grid.findFrontiers();
	shareMap(r);

	// Print results
	std::cout << "World Map:\n";
	world_grid.printGrid();
	std::cout << " Robot " << r + 1 << " (" << robots[r]->x << "," << robots[r]->y << ") Map:\n";
	robots[r]->robot_grid.printGrid();

	return true;
}

// Flood fill to simulate sensor measurements
void RobotNetwork::pulseSensor(int x, int y, int d, int r){
	if(d == 0) return;
	if(x == GRID_SIZE || x == -1 || y == GRID_SIZE || y == -1) return;

	if(world_grid.getCellValue(x, y) != 3){
		world_grid.changeCellValue(x, y, 1);
	}
	if(robots[r]->robot_grid.getCellValue(x, y) != 3){
		robots[r]->robot_grid.changeCellValue(x, y, 1);
	}
	pulseSensor(x + 1, y, d - 1, r);
	pulseSensor(x - 1, y, d - 1, r);
	pulseSensor(x, y + 1, d - 1, r);
	pulseSensor(x, y - 1, d - 1, r);
}

// Robot r will get map data from everyone in its range
void RobotNetwork::shareMap(int r){
	for(int i = 0; i < num_robots; i++){
		if(withinRange(r,i)){
			robots[i]->combineMaps(robots[r]->robot_grid);
			robots[r]->combineMaps(robots[i]->robot_grid);
		}
	}
}

// check if Robot a and Robot b are within range of one another
bool RobotNetwork::withinRange(int a, int b){
	if(a == b) return false;
	if(getDistance(robots[a]->x, robots[a]->y, robots[b]->x, robots[b]->y) > COMM_RANGE) return false;
	return true;
}

// Move Robot r to (x,y) and calculate moves to that position
int RobotNetwork::moveTo(int x, int y, int r){
	int moves = abs(robots[r]->x - x) + abs(robots[r]->y - y);
	robots[r]->prev_x = robots[r]->x;
	robots[r]->prev_y = robots[r]->y;
	robots[r]->x = x;
	robots[r]->y = y;
	return moves;
}