#ifndef MISC_H
#define MISC_H
#include <iostream>
#include <math.h>
#include <algorithm>

// Get a distance between two points
double getDistance(int x1, int y1, int x2, int y2);
// Help the generic sort function for vectors
bool comparePair(std::pair< std::pair<int, int>, double> p1, std::pair< std::pair<int, int>, double> p2);

#endif