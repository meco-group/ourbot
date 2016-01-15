#include "Circle.hpp"

Circle::Circle() :
	Obstacle(0, 0, 0.0, 0.0, CIRCLE, 0.0),
    _radius(0)
{

}

Circle::Circle(int radius) :
	Obstacle(0, 0, 0.0, 0.0, CIRCLE, 0.0),
    _radius(radius)
{

}

void Circle::setRadius(int radius)
{
	_radius = radius;
}

int Circle::getRadius()
{
	return _radius;
}

void Circle::obj2vec(Circle obstacle, std::vector<double> *obstacleVector) 
{
	std::cout<<"in Circle obj2vec"<<std::endl;
	Obstacle::obj2vec(obstacle, obstacleVector);
	std::cout<<"in Circle obj2vec, pushing back radius"<<std::endl;
	std::cout<<"circle radius: "<<obstacle.getRadius()<<std::endl;
	(*obstacleVector)[6] = obstacle.getRadius();
}

void Circle::vec2obj(std::vector<double> obstacleVector, Circle *obstacle)
{

	// obstacle = new Circle();

	std::cout<<"in Circle vec2obj"<<std::endl;
	Obstacle::vec2obj(obstacleVector, obstacle);
	std::cout<<"in Circle vec2obj, setting radius"<<std::endl;
	std::cout<<"circle radius: "<<int(obstacleVector[6])<<std::endl;
	obstacle->setRadius(int(obstacleVector[6]));

}