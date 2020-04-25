#include "OccGrid.h"
#include <iostream>

int main(){

	OccGrid my_grid(5);
	my_grid.printGrid();
	my_grid.changeCellValue(1,2,5);
	my_grid.printGrid();
	my_grid.setSize(10);
	my_grid.printGrid();

	return 0;
}