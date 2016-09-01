#ifndef RECTANGLE_H
#define RECTANGLE_H

// #define RECTANGLE_TESTFLAG

#ifdef RECTANGLE_DEBUGFLAG //print statements on/off
	#define RECTANGLE_DEBUG_PRINT(x)	std::cout << x << std::endl;
#else
	#define RECTANGLE_DEBUG_PRINT(x)	//std::cout << x << std::endl;
#endif

#include <vector>
#include "Obstacle.hpp"

class Rectangle : public Obstacle
{
private:
	int _width;
	int _length;
	double _theta;
	double _omega;

public:
	Rectangle();
	Rectangle(int width, int length, double theta, double omega); //specific constructor
	virtual ~Rectangle(){} //virtual destructor, to be able to delete instances of Rectangle

	void setWidth(int width);
	void setLength(int length);
	void setTheta(double theta);
	void setOmega(double omega);

	int getWidth();
	int getLength();
	double getTheta();
	double getOmega();

	static void obj2vec(Rectangle obstacle, std::vector<double> *obstacleVector);
	static void vec2obj(std::vector<double> obstacleVector, Rectangle *obstacle);

};

#endif