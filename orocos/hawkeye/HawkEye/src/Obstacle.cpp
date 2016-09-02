#include "Obstacle.hpp"

Obstacle::Obstacle(int x, int y, double vx, double vy, shape_t shape, double area) :
    _pos({x,y}),
    _vel({vx,vy}),
    _shape(shape),
    _area(area)
{

}

//Setters
void Obstacle::setPos(int x , int y)
{
    _pos[0] = x;
    _pos[1] = y;
}
void Obstacle::setVel(double vx , double vy)
{
    _vel[0] = vx;
    _vel[1] = vy;
}
// void Obstacle::setShape(shape_t shape)
// {
//     _shape = shape;
// }
void Obstacle::setArea(double area)
{
    _area = area;
}

//Getters
std::vector<int> Obstacle::getPos()
{
    return _pos;
}
std::vector<double> Obstacle::getVel()
{
    return _vel;
}
shape_t Obstacle::getShape()
{
    return _shape;
}
double Obstacle::getArea()
{
    return _area;
}

//Conversion functions
void Obstacle::obj2vec(Obstacle obstacle, std::vector<double> *obstacleVector)
{   
    //shape,x,y,vx,vy,area
    OBSTACLE_DEBUG_PRINT("in Obstacle obj2vec")
    OBSTACLE_DEBUG_PRINT("obstacle shape: "<<double(obstacle.getShape()))
    (*obstacleVector)[0] = double(obstacle.getShape());

    OBSTACLE_DEBUG_PRINT("Getting position")
    std::vector<int> posVector = obstacle.getPos();
    OBSTACLE_DEBUG_PRINT("obstacle pos 0: "<<double(posVector[0]))
    (*obstacleVector)[1] = double(posVector[0]);
    (*obstacleVector)[2] = double(posVector[1]);

    std::vector<double> velVector = obstacle.getVel();
    OBSTACLE_DEBUG_PRINT("obstacle vel 0: "<<double(velVector[0]))    
    (*obstacleVector)[3] = velVector[0];
    (*obstacleVector)[4] = velVector[1];    

    OBSTACLE_DEBUG_PRINT("obstacle area: "<<double(obstacle.getArea()))
    (*obstacleVector)[5] = double(obstacle.getArea());
}

void Obstacle::vec2obj(std::vector<double> obstacleVector, Obstacle *obstacle)
{
    OBSTACLE_DEBUG_PRINT("in Obstacle vec2obj: setting Pos, Vel, Area")

    obstacle->setPos(int(obstacleVector[1]), int(obstacleVector[2]));
    obstacle->setVel(obstacleVector[3], obstacleVector[4]);
    obstacle->setArea(obstacleVector[5]);
}