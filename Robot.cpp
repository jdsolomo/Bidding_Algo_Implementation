#include "Robot.h"

Robot::Robot(int x, int y, int state, int sz):
	x(x),
	y(y),
	state(state),
	robot_grid(sz)
{}

Robot::Robot():
	robot_grid(1)
{}