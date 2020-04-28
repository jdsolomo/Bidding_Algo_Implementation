// This class will mainly be a structure to hold the robot's coordinates and
// and occupancy grid so that the algorithm can act in a distributed fashion
// instead of using knowledge of the absolute map in the RobotNetwork class

#include "Robot.h"

Robot::Robot(int x, int y, int state, int sz):
	x(x),
	y(y),
	prev_x(x),
	prev_y(y),
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

void Robot::updateBids(){
	bids.clear();
	double bid = 0.0;
	for(int i = 0; i < size; i++){
		for(int j = 0; j < size; j++){
			if(robot_grid.getCellValue(i, j) == 2){
				bid = getBid(i, j);
				bids.push_back(std::make_pair(std::make_pair(i, j), bid));
			}
		}
	}
}

double Robot::getBid(int x, int y){
	int information_gain = getInfoGain(x, y, SENSOR_RANGE);
	// Assume no obstacles for now
	double shortest_distance = getDistance(x, y, this->x, this->y);
	double nearness = getLambda(x, y);

	double bid = OMEGA1 * information_gain + OMEGA2 * shortest_distance + OMEGA3 * nearness;
	return bid;
}

int Robot::getInfoGain(int x, int y, int d){
	if(x == size || x == -1 || y == size || y == -1) return 0;
	if(d < 0) return 0;
	if(!otherRobotsNearby(x, y) && robot_grid.getCellValue(x, y) == 0){
		return getInfoGain(x+1, y, d-1) + getInfoGain(x-1, y, d-1) + getInfoGain(x, y+1, d-1) + getInfoGain(x, y-1, d-1) + 1;
	}
	else{
		return getInfoGain(x+1, y, d-1) + getInfoGain(x-1, y, d-1) + getInfoGain(x, y+1, d-1) + getInfoGain(x, y-1, d-1);

	}

}

bool Robot::otherRobotsNearby(int x, int y){
	for(int i = 0; i < size; i++){
		for(int j = 0; j < size; j++){
			if(robot_grid.getCellValue(i, j) == 3 && getDistance(x, y, i, j) < SENSOR_RANGE){
				return true;
			}
		}
	}

	return false;
}

double Robot::getLambda(int x, int y){
	double lambda = 0.0;
	int count = 0;
	for(int i = 0; i < size; i++){
		for(int j = 0; j < size; j++){
			// Assume all robots are within comm range for now
			if(robot_grid.getCellValue(i, j) == 3){
				lambda += pow(ALPHA, count) * exp(-1 * getDistance(x, y, i, j) / COMM_RANGE);
				count++;
			}
		}
	}
	return lambda;
}