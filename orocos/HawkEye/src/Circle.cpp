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
	CIRCLE_DEBUG_PRINT("in Circle obj2vec")
	Obstacle::obj2vec(obstacle, obstacleVector);
	CIRCLE_DEBUG_PRINT("in Circle obj2vec, pushing back radius")
	CIRCLE_DEBUG_PRINT("circle radius: "<<obstacle.getRadius())
	(*obstacleVector)[6] = obstacle.getRadius();
}

void Circle::vec2obj(std::vector<double> obstacleVector, Circle *obstacle)
{

	// obstacle = new Circle();
	CIRCLE_DEBUG_PRINT("in Circle vec2obj")
	Obstacle::vec2obj(obstacleVector, obstacle);
	CIRCLE_DEBUG_PRINT("in Circle vec2obj, setting radius")
	CIRCLE_DEBUG_PRINT("circle radius: "<<int(obstacleVector[6]))
	obstacle->setRadius(int(obstacleVector[6]));

}