#include "Robot.hpp"

Robot::Robot() :
    _pos({0,0}),
    _vel({0,0}),
    _theta(0),
    _omega(0),
    _width(0),
    _length(0),
    _radius(0),
    _roi{0},
    _bottomMarkers{0},
    _topMarker{0},
    _marker{}
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
void Robot::setRoi(int x, int y){
    _roi[0] = x;
    _roi[1] = y;    
}
void Robot::setMarker(double *bottomMarkers, double *topMarker){
    std::cout<<"In setMarkers"<<std::endl;
    for (int k = 0 ; k<BOTTOMMARKERSLENGTH ; k++){
      _bottomMarkers[k] = bottomMarkers[k];
      _marker.bottomMarkers[k] = _bottomMarkers[k];
    }
    std::cout<<"set bottom markers"<<std::endl;
    for (int k = 0 ; k<TOPMARKERLENGTH ; k++){
        _topMarker[k] = topMarker[k];
        _marker.topMarker[k] = _topMarker[k];
    }        
    std::cout<<"set top marker"<<std::endl;    
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
int* Robot::getRoi()
{
    return _roi;
}
Robot::marker Robot::getMarker()
{
    return _marker;
}
double* Robot::getBottomMarkers()
{
    return _marker.bottomMarkers;
}
double* Robot::getTopMarker()
{
    return _marker.topMarker;
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

