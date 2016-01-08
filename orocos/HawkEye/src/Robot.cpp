#include "Robot.hpp"

Robot::Robot() :
    _pos({0,0}),
    _vel({0,0}),
    _theta(0),
    _omega(0),
    _width(0),
    _length(0),
    _radius(0)
{
     
}

Robot::Robot(int x, int y, double vx, double vy, double theta, double omega, int width, int length, int radius) :
    _pos({x,y}),
    _vel({vx,vy}),
    _theta(theta),
    _omega(omega),
    _width(width),
    _length(length),
    _radius(radius)
{
     
}

//Setters
void Robot::setPos(int x , int y)
{
    _pos[0] = x;
    _pos[1] = y;
}
void Robot::setVel(double vx , double vy)
{
    _vel[0] = vx;
    _vel[1] = vy;
}
void Robot::setTheta(double theta)
{
    _theta = theta;
}
void Robot::setOmega(double omega)
{
    _omega = omega;
}
void Robot::setWidth(int width)
{
    _width = width;
}
void Robot::setLength(int length)
{
    _length = length;
}
void Robot::setRadius(int radius)
{
    _radius = radius;
}

//Getters
std::vector<int> Robot::getPos()
{
    return _pos;
}
std::vector<double> Robot::getVel()
{
    return _vel;
}
double Robot::getTheta()
{
    return _theta;
}
double Robot::getOmega()
{
    return _omega;
}
int Robot::getWidth()
{
    return _width;
}
int Robot::getLength()
{
    return _length;
}
int Robot::getRadius()
{
    return _radius;
}

//Conversion functions
void Robot::obj2vec(Robot robot, std::vector<double> *robotVector)
{
    std::vector<int> posVector = robot.getPos();
    robotVector->push_back(double(posVector[0]));//x,y,vx,vy,theta,omega,width,length,radius
    robotVector->push_back(double(posVector[1]));

    std::vector<double> velVector = robot.getVel();
    robotVector->push_back(velVector[0]);
    robotVector->push_back(velVector[1]);

    robotVector->push_back(robot.getTheta());
    robotVector->push_back(robot.getOmega());

    robotVector->push_back(double(robot.getWidth()));
    robotVector->push_back(double(robot.getLength()));
    robotVector->push_back(double(robot.getRadius()));

}

void Robot::vec2obj(std::vector<double> robotVector, Robot *robot)
{
    robot->setPos(int(robotVector[0]), int(robotVector[1]));
    robot->setVel(robotVector[2], robotVector[3]);
    robot->setTheta(robotVector[4]);
    robot->setOmega(robotVector[5]);    
    robot->setWidth(int(robotVector[6]));
    robot->setLength(int(robotVector[7]));
    robot->setRadius(int(robotVector[8]));

}

