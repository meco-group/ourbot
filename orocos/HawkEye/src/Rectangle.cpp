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
	std::cout<<"in Rectangle obj2vec"<<std::endl;
	Obstacle::obj2vec(obstacle, obstacleVector);
	std::cout<<"in Rectangle obj2vec, pushing back width, length,..."<<std::endl;
	std::cout<<"Obstacle width: "<<obstacle.getWidth()<<std::endl;
	std::cout<<"Obstacle length: "<<obstacle.getLength()<<std::endl;
	std::cout<<"Obstacle theta: "<<obstacle.getTheta()<<std::endl;
	std::cout<<"Obstacle omega: "<<obstacle.getOmega()<<std::endl;
	(*obstacleVector)[6] = obstacle.getWidth();
	(*obstacleVector)[7] = obstacle.getLength();
	(*obstacleVector)[8] = obstacle.getTheta();
	(*obstacleVector)[9] = obstacle.getOmega();
}

void Rectangle::vec2obj(std::vector<double> obstacleVector, Rectangle *obstacle)
{
	
	// obstacle = new Rectangle();
	std::cout<<"in Rectangle vec2obj"<<std::endl;
	Obstacle::vec2obj(obstacleVector, obstacle);
	std::cout<<"in Rectangle vec2obj, setting width, length,..."<<std::endl;
	std::cout<<"rectangle width: "<<obstacleVector[6]<<std::endl;
	obstacle->setWidth(int(obstacleVector[6]));
	obstacle->setLength(int(obstacleVector[7]));
	obstacle->setTheta(int(obstacleVector[8]));
	obstacle->setOmega(int(obstacleVector[9]));

}