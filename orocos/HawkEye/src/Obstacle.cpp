#include "Obstacle.hpp"

Obstacle::Obstacle(int x, int y, double vx, double vy, shape_t shape, int area) :
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
void Obstacle::setArea(int area)
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
int Obstacle::getArea()
{
    return _area;
}

//Conversion functions
void Obstacle::obj2vec(Obstacle obstacle, std::vector<double> *obstacleVector)
{   
    //shape,x,y,vx,vy,area

    (*obstacleVector)[0] = double(obstacle.getShape());

    std::vector<int> posVector = obstacle.getPos();
    (*obstacleVector)[1] = double(posVector[0]);
    (*obstacleVector)[2] = double(posVector[1]);

    std::vector<double> velVector = obstacle.getVel();
    (*obstacleVector)[3] = velVector[0];
    (*obstacleVector)[4] = velVector[1];    

    (*obstacleVector)[5] = double(obstacle.getArea());
}

void Obstacle::vec2obj(std::vector<double> obstacleVector, Obstacle *obstacle)
{
    
    obstacle->setPos(int(obstacleVector[1]), int(obstacleVector[2]));
    obstacle->setVel(obstacleVector[3], obstacleVector[4]);
    obstacle->setArea(int(obstacleVector[5]));
}