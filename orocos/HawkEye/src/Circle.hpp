#ifndef CIRCLE_H
#define CIRCLE_H

// #define CIRCLE_DEBUGFLAG

#ifdef CIRCLE_DEBUGFLAG //print statements on/off
	#define CIRCLE_DEBUG_PRINT(x)	std::cout << x << std::endl;
#else
	#define CIRCLE_DEBUG_PRINT(x)	//std::cout << x << std::endl;
#endif

#include <vector>
#include "Obstacle.hpp"

class Circle : public Obstacle
{
private:
	int _radius;

public:
	Circle();
	Circle(int radius); //specific constructor
	virtual ~Circle(){} //virtual destructor, to be able to delete instances of Circle

	void setRadius(int radius);

	int getRadius();

	static void obj2vec(Circle obstacle, std::vector<double> *obstacleVector);
	static void vec2obj(std::vector<double> obstacleVector, Circle *obstacle);

};

#endif