#ifndef ROBOT_NETWORK_H
#define ROBOT_NETWORK_H
#include "Misc.h"
#include "OccGrid.h"
#include "Robot.h"
#include <vector>
#include <algorithm>
#include <utility>
#include <stdlib.h>
#include <time.h>
#include <unistd.h>

#define GRID_SIZE 20

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
	int moveTo(int x, int y, int r);
	bool checkMove(int r);
	void takeScan(int r);

public:
	RobotNetwork();
	RobotNetwork(int n);
	~RobotNetwork();
	void exploreGrid_Random();
	void exploreGrid_BiddingLite();
	void exploreGrid_Bidding();
};

#endif