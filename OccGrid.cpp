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
			if(grid[i][j] != 1){
				return false;
			}
		}
	}
	return true;
}

// Get the value of cell x,y
int OccGrid::getCellValue(int x, int y){
	return grid[x][y];
}

// Find all frontiers in the grid
void OccGrid::findFrontiers(){
	for(int i = 0; i < size; i++){
		for(int j = 0; j < size; j++){
			if(grid[i][j] == 0 && checkFrontier(i,j)){
				grid[i][j] = 2;
			}
		}
	}
}

// Check if the cell at x,y is a frontier
bool OccGrid::checkFrontier(int x, int y){
	if(x == size - 1){
		if(y == size - 1){
			if(grid[x-1][y] == 1 || grid[x][y-1] == 1) return true;
			else return false;
		}
		else if(y == 0){
			if(grid[x-1][y] == 1 || grid[x][y+1] == 1) return true;
			else return false;
		}
		else{
			if(grid[x-1][y] == 1 || grid[x][y-1] == 1 || grid[x][y+1] == 1) return true;
			else return false;
		}
	}
	else if(x == 0){
		if(y == size - 1){
			if(grid[x+1][y] == 1 || grid[x][y-1] == 1) return true;
			else return false;
		}
		else if(y == 0){
			if(grid[x+1][y] == 1 || grid[x][y+1] == 1) return true;
			else return false;
		}
		else{
			if(grid[x+1][y] == 1 || grid[x][y-1] == 1 || grid[x][y+1] == 1) return true;
			else return false;
		}
	}
	else{
		if(grid[x+1][y] == 1 || grid[x-1][y] == 1 || grid[x][y+1] == 1 || grid[x][y-1] == 1) return true;
		else return false;
	}
}