#include "Polygon.hpp"

Polygon::Polygon(){}

double Polygon::getXVertex(int number){
	return _x_coordinates.at(number);
}
double Polygon::getYVertex(int number){
	return _y_coordinates.at(number);
}

void Polygon::addVertex(double x, double y){
	_x_coordinates.push_back(x);
	_y_coordinates.push_back(y);
}

int Polygon::getSize(){
	return _x_coordinates.size();
}