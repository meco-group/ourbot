#ifndef OBSTACLE_H
#define OBSTACLE_H

// #define OBSTACLE_TESTFLAG

#ifdef OBSTACLE_DEBUGFLAG //print statements on/off
	#define OBSTACLE_DEBUG_PRINT(x)	std::cout << x << std::endl;
#else
	#define OBSTACLE_DEBUG_PRINT(x)	//std::cout << x << std::endl;
#endif

#include <vector>
#include <iostream>

typedef enum shape_t{ //define enum to hold possible shapes
    CIRCLE = 1,
    RECTANGLE = 2
} shape_t;

class Obstacle
{
private:
	std::vector<int>    _pos;
	std::vector<double> _vel;
	const shape_t _shape;
	double _area;

public:
	
	Obstacle(int x, int y, double vx, double vy, shape_t shape, double area); //specific constructor
	virtual ~Obstacle(){} //virtual destructor to be able to delete inherited classes

	void setPos(int x, int y);
	void setVel(double vx, double vy);
	// void setShape(shape_t shape);
	void setArea(double area);

	std::vector<int> getPos();
	std::vector<double> getVel();
	shape_t getShape();
	double getArea();

protected:
	static void obj2vec(Obstacle obstacle, std::vector<double> *obstacleVector);
	static void vec2obj(std::vector<double> obstacleVector, Obstacle *obstacle);

};

#endif