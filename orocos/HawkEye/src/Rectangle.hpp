#ifndef RECTANGLE_H
#define RECTANGLE_H

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

	void setWidth(int width);
	void setLength(int length);
	void setTheta(double theta);
	void setOmega(double omega);

	int getWidth();
	int getLength();
	double getTheta();
	double getOmega();

	static void obj2vec(Rectangle obstacle, std::vector<double> *obstacleVector);
	static void vec2obj(std::vector<double> obstacleVector, Obstacle *obstacle);

};

#endif