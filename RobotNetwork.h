#ifndef ROBOT_NETWORK_H
#define ROBOT_NETWORK_H
#include "OccGrid.h"
#include "Robot.h"
#include <vector>
#include <algorithm>
#include <utility>
#include <stdlib.h>
#include <time.h>

#define GRID_SIZE 5
#define SENSOR_RANGE 3
#define COMM_RANGE 4

class RobotNetwork: public Robot {

private:
	OccGrid world_grid;
	int num_robots;
	long steps_to_success;
	// Vector to hold robots' coordinates and state
	std::vector< Robot* > robots;
	bool updateMap(int r);
	void pulseSensor(int x, int y, int d, int r);
	void shareMap(int r);
	bool withinRange(int a, int b);

public:
	RobotNetwork();
	RobotNetwork(int n);
	~RobotNetwork();
	void exploreGrid_Random();
};

#endif