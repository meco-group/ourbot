#include "Rectangle.hpp"

Rectangle::Rectangle() :
	Obstacle(0, 0, 0.0, 0.0, RECTANGLE, 0),
    _width(0),
    _length(0),
    _theta(0),
    _omega(0)
{

}

Rectangle::Rectangle(int width, int length, double theta, double omega) :
	Obstacle(0, 0, 0.0, 0.0, RECTANGLE, 0),
    _width(width),
    _length(length),
    _theta(theta),
    _omega(omega)
{

}

void Rectangle::setWidth(int width)
{
	_width = width;
}
void Rectangle::setLength(int length)
{
	_length = length;
}
void Rectangle::setTheta(double theta)
{
    _theta = theta;
}
void Rectangle::setOmega(double omega)
{
    _omega = omega;
}

int Rectangle::getWidth()
{
	return _width;
}
int Rectangle::getLength()
{
	return _length;
}
double Rectangle::getTheta()
{
    return _theta;
}
double Rectangle::getOmega()
{
    return _omega;
}

void Rectangle::obj2vec(Rectangle obstacle, std::vector<double> *obstacleVector)
{
	
	Obstacle::obj2vec(obstacle, obstacleVector);

	obstacleVector->push_back(obstacle.getWidth());
	obstacleVector->push_back(obstacle.getLength());
	obstacleVector->push_back(obstacle.getTheta());
	obstacleVector->push_back(obstacle.getOmega());

}

void Rectangle::vec2obj(std::vector<double> obstacleVector, Obstacle *obstacle)
{
	
	obstacle = new Rectangle();

	Obstacle::vec2obj(obstacleVector, obstacle);
	
	obstacle->setWidth(int(obstacleVector[6]));
	obstacle->setLength(int(obstacleVector[7]));
	obstacle->setTheta(int(obstacleVector[8]));
	obstacle->setOmega(int(obstacleVector[9]));

}