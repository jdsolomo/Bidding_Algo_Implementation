// This class will mainly be a structure to hold the robot's coordinates and
// and occupancy grid so that the algorithm can act in a distributed fashion
// instead of using knowledge of the absolute map in the RobotNetwork class

#include "Robot.h"

Robot::Robot(int x, int y, int state, int sz):
	x(x),
	y(y),
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
	state(bot.state),
	robot_grid(bot.size)
{}

void Robot::combineMaps(OccGrid& other_grid){
	for(int i = 0; i < size; i++){
		for(int j = 0; j < size; j++){
			if(robot_grid.getCellValue(i, j) != other_grid.getCellValue(i, j)){
				if(other_grid.getCellValue(i, j) == 1){
					robot_grid.changeCellValue(i, j, 1);
				}
				else if(other_grid.getCellValue(i, j) == 2 && robot_grid.getCellValue(i, j) == 0){
					robot_grid.changeCellValue(i, j, 2);
				}
			}
		}
	}
}