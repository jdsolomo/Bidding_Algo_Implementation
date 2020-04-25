#include "RobotNetwork.h"
#include <iostream>

int main(){
	RobotNetwork my_robots(2);
	std::cout << "OKIE DOKIE\n";
	my_robots.exploreGrid_Random();
}