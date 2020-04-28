#ifndef ROBOT_H
#define ROBOT_H
#include "Misc.h"
#include "OccGrid.h"
#include <vector>

#define OMEGA1 0.5
#define OMEGA2 0.5
#define OMEGA3 0.5
#define SENSOR_RANGE 3
#define COMM_RANGE 10.0
#define ALPHA 0.50

class Robot {
private:
	double getBid(int x, int y);
	int getInfoGain(int x, int y, int d);
	bool otherRobotsNearby(int x, int y);
public:
	int x, y, prev_x, prev_y, state, size, num_robots;
	OccGrid robot_grid;
	std::vector< std::pair< std::pair<int, int>, double > > bids;
	Robot();
	Robot(int x, int y, int state, int sz);
	Robot(const Robot& bot);
	void combineMaps(OccGrid& other_grid);
	void updateBids();
	double getLambda(int x, int y);
};

#endif