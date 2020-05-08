#include "Misc.h"

double getDistance(int x1, int y1, int x2, int y2){
	return sqrt(pow(x1 - x2, 2) + pow(y1 - y2, 2));
}

bool comparePair(std::pair< std::pair<int, int>, double> p1, std::pair< std::pair<int, int>, double> p2){
	return (p1.second > p2.second);
}