#include "OccGrid.h"

// Create an occupancy grid of size 1 filled with zeros
OccGrid::OccGrid():
	size(1)
{
	grid = new int*[size];
	for(int i = 0; i < size; i++){
		grid[i] = new int[size];
	}
	for(int i = 0; i < size; i++){
		for(int j = 0; j < size; j++){
			grid[i][j] = 0;
		}
	}
}

// Create an occupancy grid of size sz filled with zeros
OccGrid::OccGrid(int sz):
	size(sz)
{
	grid = new int*[size];
	for(int i = 0; i < size; i++){
		grid[i] = new int[size];
	}
	for(int i = 0; i < size; i++){
		for(int j = 0; j < size; j++){
			grid[i][j] = 0;
		}
	}
}

OccGrid::OccGrid(const OccGrid& occ):
	size(occ.size)
{
	grid = new int*[size];
	for(int i = 0; i < size; i++){
		grid[i] = new int[size];
	}
	for(int i = 0; i < size; i++){
		for(int j = 0; j < size; j++){
			grid[i][j] = occ.grid[i][j];
		}
	}
}

// Deallocate occupancy grid
OccGrid::~OccGrid(){
	for(int i = 0; i < size; i++){
		delete grid[i];
	}
	delete [] grid;
}

// Print the current occupancy grid to the screen
void OccGrid::printGrid(){
	for(int j = 0; j < size; j++){
		std::cout << "- - ";
	}
	std::cout << "\n";
	for(int i = 0; i < size; i++){
		for(int j = 0; j < size; j++){
			std::cout << grid[i][j] << " | ";
		}
		std::cout << "\n";
		for(int j = 0; j < size; j++){
			std::cout << "- - ";
		}
		std::cout << "\n";
	}
}

// Change the value of a cell to val
void OccGrid::changeCellValue(int x, int y, int val){
	grid[x][y] = val;
}

// Check if the graph has been completely explored
bool OccGrid::gridExplored(){
	for(int i = 0; i < size; i++){
		for(int j = 0; j < size; j++){
			if(grid[i][j] == 0){
				return false;
			}
		}
	}
	return true;
}

int OccGrid::getCellValue(int x, int y){
	return grid[x][y];
}