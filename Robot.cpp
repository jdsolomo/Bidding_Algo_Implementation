// This class will mainly be a structure to hold the robot's coordinates and
// and occupancy grid so that the algorithm can act in a distributed fashion
// instead of using knowledge of the absolute map in the RobotNetwork class

#include "Robot.h"

Robot::Robot(int x, int y, int state, int sz):
	x(x),
	y(y),
	prev_x(x),
	prev_y(y),
	goal_x(0),
	goal_y(0),
	state(state),
	size(sz),
	robot_grid(sz)
{}

Robot::Robot():
	robot_grid(1)
{}

Robot::Robot(const Robot& bot):
	x(bot.x),
	y(bot.y),
	prev_x(bot.prev_x),
	prev_y(bot.prev_y),
	goal_x(bot.goal_x),
	goal_y(bot.goal_y),
	state(bot.state),
	robot_grid(bot.size)
{}

void Robot::combineMaps(OccGrid& other_grid){
	for(int i = 0; i < size; i++){
		for(int j = 0; j < size; j++){
			if(robot_grid.getCellValue(i, j) == 3) robot_grid.changeCellValue(i, j, 1);
			if(robot_grid.getCellValue(i, j) != other_grid.getCellValue(i, j)){
				if(other_grid.getCellValue(i, j) == 3){
					robot_grid.changeCellValue(i, j, 3);
				}
				else if(other_grid.getCellValue(i, j) == 1 && robot_grid.getCellValue(i, j) != 3){
					robot_grid.changeCellValue(i, j, 1);
				}
				else if(other_grid.getCellValue(i, j) == 2 && robot_grid.getCellValue(i, j) == 0){
					robot_grid.changeCellValue(i, j, 2);
				}
			}
		}
	}
}

// Gets a list of bids on all the frontier cell and descendingly organizes them
void Robot::updateBids(const std::vector< Robot* >& robots){
	bids.clear();
	double bid = 0.0;
	for(int i = 0; i < size; i++){
		for(int j = 0; j < size; j++){
			if(robot_grid.getCellValue(i, j) == 2){
				bid = getBid(i, j, robots);
				bids.push_back(std::make_pair(std::make_pair(i, j), bid));
			}
		}
	}
	// sort bids vector
	sort(bids.begin(), bids.end(), comparePair);
}

// Gets a single bid on frontier cell (x,y)
double Robot::getBid(int x, int y, const std::vector< Robot* >& robots){
	int information_gain = getInfoGain(x, y, robots);
	// Assume no obstacles for now
	double shortest_distance = getDistance(x, y, this->x, this->y);
	double nearness = getLambda(x, y, robots);

	double bid = OMEGA1 * information_gain - OMEGA2 * shortest_distance + OMEGA3 * nearness;
	return bid;
}

// Gets the potential information gained by traveling to a frontier cell (x,y)
int Robot::getInfoGain(int x, int y, const std::vector< Robot* >& robots){
	int gain = 0;
	for(int i = 0; i < size; i++){
		for(int j = 0; j < size; j++){
			// Counts unknown cells that would be within SENSOR_RANGE at the frontier cell and would not be within SENSOR_RANGE
			// of any other robot's goal cell
			if(ceil(getDistance(x, y, i, j)) <= SENSOR_RANGE && robot_grid.getCellValue(i, j) == 0 && !(otherRobotsNearby(i, j, robots))){
				gain++;
			}
		}
	}
	return gain;
}

// Check if a cell (x,y) would be within SENSOR_RANGE of another robot's goal cell (goal_x, goal_y)
bool Robot::otherRobotsNearby(int x, int y, const std::vector< Robot* >& robots){
	int len = robots.size();
	for(int i = 0; i < len; i++){
		// Check if an unknown cell would be within SENSOR_RANGE of another robot's goal cell
		if((robots[i]->x != this->x || robots[i]->y != this->y) && ceil(getDistance(x, y, robots[i]->goal_x, robots[i]->goal_y)) <= SENSOR_RANGE){
			return true;
		}
	}

	return false;
}

// Finds the "nearness" measure of a robot if it were to travel to a cell (x,y)
double Robot::getLambda(int x, int y, const std::vector< Robot* >& robots){
	double lambda = 0.0;
	int count = 0;
	int len = robots.size();
	for(int i = 0; i < len; i++){
		if((robots[i]->x != this->x || robots[i]->y != this->y) && ceil(getDistance(x, y, robots[i]->goal_x, robots[i]->goal_y)) <= COMM_RANGE){
			lambda += pow(ALPHA, count) * exp(-1 * getDistance(robots[i]->goal_x, robots[i]->goal_y, x, y) / COMM_RANGE);
			count++;
		}
	}
	return lambda;
}

// Gets the nth max bid
std::pair< std::pair<int, int>, double > Robot::getMaxBid(int n){
	if(n > bids.size() - 1) return std::make_pair(std::make_pair(prev_x, prev_y), 0.0);
	return bids[n];
}