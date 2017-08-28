#include "Circle.hpp"
//Michiel
Circle::Circle(double x, double y, double radius){
  _x = x;
  _y = y;
  _radius = radius;
}

double Circle::getX(){
  return _x;
}

double Circle::getY(){
  return _y;
}

double Circle::getRadius(){
  return _radius;
}