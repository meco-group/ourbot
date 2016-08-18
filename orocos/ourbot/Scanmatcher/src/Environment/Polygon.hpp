#ifndef POLYGON
#define POLYGON

#include <vector>

class Polygon{
  private:
  	std::vector<double> _x_coordinates;
  	std::vector<double> _y_coordinates;

  public:
  	Polygon();
  	double getXVertex(int number);
  	double getYVertex(int number);
  	void addVertex(double x, double y);
  	int getSize();
};

#endif