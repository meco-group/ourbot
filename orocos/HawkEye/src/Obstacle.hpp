#ifndef OBSTACLE_H
#define OBSTACLE_H

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
	int _area;

public:
	
	Obstacle(int x, int y, double vx, double vy, shape_t shape, int area); //specific constructor

	void setPos(int x, int y);
	void setVel(double vx, double vy);
	// void setShape(shape_t shape);
	void setArea(int area);

	std::vector<int> getPos();
	std::vector<double> getVel();
	shape_t getShape();
	int getArea();

protected:
	static void obj2vec(Obstacle obstacle, std::vector<double> *obstacleVector);
	static void vec2obj(std::vector<double> obstacleVector, Obstacle *obstacle);

};

#endif