#ifndef CIRCLE
#define CIRCLE
//Michiel
class Circle{
  private:
    double _x;
    double _y;
    double _radius;

  public:
  	Circle(double, double, double);
    double getX();
    double getY();
    double getRadius();
};

#endif