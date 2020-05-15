#ifndef ROBOT_H
#define ROBOT_H
#include "Misc.h"
#include "OccGrid.h"
#include <vector>

#define OMEGA1 1
#define OMEGA2 1
#define OMEGA3 0
#define SENSOR_RANGE 2
#define COMM_RANGE 29
#define ALPHA 0.50


class Robot {
private:
	double getBid(int x, int y, const std::vector< Robot* >& robots);
	int getInfoGain(int x, int y, const std::vector< Robot* >& robots);
	bool otherRobotsNearby(int x, int y, const std::vector< Robot* >& robots);
public:
	int x, y, prev_x, prev_y, goal_x, goal_y, state, size, num_robots;
	OccGrid robot_grid;
	std::vector< std::pair< std::pair<int, int>, double > > bids;
	Robot();
	Robot(int x, int y, int state, int sz);
	Robot(const Robot& bot);
	void combineMaps(OccGrid& other_grid);
	void updateBids(const std::vector< Robot* >& robots);
	double getLambda(int x, int y, const std::vector< Robot* >& robots);
	std::pair< std::pair<int, int>, double > getMaxBid(int n);
};

#endif