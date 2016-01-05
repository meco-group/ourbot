#ifndef ROBOT_H
#define ROBOT_H

#include <vector>
 
class Robot
{
private:
    std::vector<int>    _pos;
    std::vector<double> _vel;
    double _theta;
    double _omega;
	int _width;
	int _length;
	int _radius;
 
public:
	Robot();
    Robot(int x, int y, double vx, double vy, double theta, double omega, int width, int length, int radius); //specific constructor
 
    void setPos(int x , int y);
	void setVel(double vx , double vy);
	void setTheta(double theta);
	void setOmega(double omega);
	void setWidth(int width);
	void setLength(int length);
	void setRadius(int radius);

	std::vector<int> getPos();
 	std::vector<double> getVel();
	double getTheta();
	double getOmega();
	int getWidth();
	int getLength();
	int getRadius();

	static void obj2vec(Robot robot, std::vector<double> *robotVector);
	static void vec2obj(std::vector<double> robotVector, Robot *robot);
};
 
#endif