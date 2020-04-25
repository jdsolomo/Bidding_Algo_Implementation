#ifndef ROBOT_H
#define ROBOT_H
#include "OccGrid.h"

class Robot {
public:
	int x, y, state, num_robots;
	OccGrid robot_grid;
	Robot();
	Robot(int x, int y, int state, int sz);
};

#endif