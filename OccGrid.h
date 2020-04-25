#ifndef OCCGRID_H
#define OCCGRID_H
#include <iostream>

class OccGrid {

private:
	int size;
	int** grid;

public:
	OccGrid();
	~OccGrid();
	OccGrid(int sz);
	void printGrid();
	void changeCellValue(int x, int y, int val);
	bool gridExplored();
	int getCellValue(int x, int y);
	void setSize(int sz);
};

#endif