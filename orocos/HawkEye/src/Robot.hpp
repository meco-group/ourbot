#ifndef ROBOT_H
#define ROBOT_H

// #define ROBOT_DEBUGFLAG

#ifdef ROBOT_DEBUGFLAG //print statements on/off
	#define ROBOT_DEBUG_PRINT(x)	std::cout << x << std::endl;
#else
	#define ROBOT_DEBUG_PRINT(x)	//std::cout << x << std::endl;
#endif

#define BOTTOMMARKERSLENGTH 7
#define TOPMARKERLENGTH 5

#include <vector>
#include <iostream>

class Robot
{

public:
	struct marker {  //define here, otherwise can't make a private _markers
  		double bottomMarkers[BOTTOMMARKERSLENGTH];
		double topMarker[TOPMARKERLENGTH];
	};	

private:
    std::vector<int>    _pos;
    std::vector<double> _vel;
    double _theta;
    double _omega;
	int _width;
	int _length;
	int _radius;
	int _roi[2];
	marker _marker;
	double _bottomMarkers[BOTTOMMARKERSLENGTH];
	double _topMarker[TOPMARKERLENGTH];
 
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
	void setRoi(int x, int y);
	void setMarker(double *bottomMarkers, double *topMarker);

	std::vector<int> getPos();
 	std::vector<double> getVel();
	double getTheta();
	double getOmega();
	int getWidth();
	int getLength();
	int getRadius();
	marker getMarker();
	int* getRoi();
	double* getBottomMarkers();
	double* getTopMarker();

	static void obj2vec(Robot robot, std::vector<double> *robotVector);
	static void vec2obj(std::vector<double> robotVector, Robot *robot);
};
 
#endif