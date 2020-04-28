#ifndef OCCGRID_H
#define OCCGRID_H
#include "Misc.h"
#include <iostream>

class OccGrid {

private:
	int size;
	int** grid;
	bool checkFrontier(int x, int y);

public:
	OccGrid();
	~OccGrid();
	OccGrid(int sz);
	OccGrid(const OccGrid& occ);
	void printGrid();
	void changeCellValue(int x, int y, int val);
	bool gridExplored();
	int getCellValue(int x, int y);
	void findFrontiers();
};

#endif