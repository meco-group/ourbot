#ifndef MOTIONPLANNING
#define MOTIONPLANNING

#include <casadi/casadi.hpp>
#include <math.h>
#include <memory.h>
#include <vector>
#include <iostream>

#define inf std::numeric_limits<double>::infinity()
#define N_OBS 0
#define G_0_TF {{0.0,0.25,0.583333333333,0.166666666667,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0},{0.0,0.0,0.666666666667,0.333333333333,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0},{0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0},{0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0},{0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0},{0.0,0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0},{0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0},{0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0},{0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0},{0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0},{0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,-0.5,1.5,0.0},{0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.5,-3.5,4.0},{0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,-0.166666666667,2.41666666667,-9.25,8.0}}
#define N_Y 2
#define N_IN 2
#define Y_DEGREE 3
#define TOL 0.001
#define DERT_DEF {{{-30.0,30.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0},{0.0,-15.0,15.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0},{0.0,0.0,-10.0,10.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0},{0.0,0.0,0.0,-10.0,10.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0},{0.0,0.0,0.0,0.0,-10.0,10.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0},{0.0,0.0,0.0,0.0,0.0,-10.0,10.0,0.0,0.0,0.0,0.0,0.0,0.0},{0.0,0.0,0.0,0.0,0.0,0.0,-10.0,10.0,0.0,0.0,0.0,0.0,0.0},{0.0,0.0,0.0,0.0,0.0,0.0,0.0,-10.0,10.0,0.0,0.0,0.0,0.0},{0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,-10.0,10.0,0.0,0.0,0.0},{0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,-10.0,10.0,0.0,0.0},{0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,-15.0,15.0,0.0},{0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,-30.0,30.0}},{{600.0,-900.0,300.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0},{0.0,150.0,-250.0,100.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0},{0.0,0.0,100.0,-200.0,100.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0},{0.0,0.0,0.0,100.0,-200.0,100.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0},{0.0,0.0,0.0,0.0,100.0,-200.0,100.0,0.0,0.0,0.0,0.0,0.0,0.0},{0.0,0.0,0.0,0.0,0.0,100.0,-200.0,100.0,0.0,0.0,0.0,0.0,0.0},{0.0,0.0,0.0,0.0,0.0,0.0,100.0,-200.0,100.0,0.0,0.0,0.0,0.0},{0.0,0.0,0.0,0.0,0.0,0.0,0.0,100.0,-200.0,100.0,0.0,0.0,0.0},{0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,100.0,-200.0,100.0,0.0,0.0},{0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,100.0,-250.0,150.0,0.0},{0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,300.0,-900.0,600.0}}}
#define N_VAR 52
#define LBG_DEF {-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,0.0,0.0,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0}
#define ORDER 1
#define LINEAR_SOLVER "ma57"
#define G_0_KNOTS {0.0,0.0,0.0,0.0,0.1,0.2,0.3,0.4,0.5,0.6,0.7,0.8,0.9,1.0,1.0,1.0,1.0}
#define N_DER 2
#define G_0_LENGTH 13
#define UBG_DEF {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0}
#define N_DIM 2
#define N_PAR 14
#define N_CON 206
#define G_0_DEGREE 3
#define Y_KNOTS {0.0,0.0,0.0,0.0,0.1,0.2,0.3,0.4,0.5,0.6,0.7,0.8,0.9,1.0,1.0,1.0,1.0}
#define N_ST 2
#define Y_LENGTH 13
#define Y_TF {{0.0,0.25,0.583333333333,0.166666666667,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0},{0.0,0.0,0.666666666667,0.333333333333,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0},{0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0},{0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0},{0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0},{0.0,0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0},{0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0},{0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0},{0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0},{0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0},{0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,-0.5,1.5,0.0},{0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.5,-3.5,4.0},{0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,-0.166666666667,2.41666666667,-9.25,8.0}}


namespace mp{

typedef struct obstacle {
    double position[N_DIM];
    double velocity[N_DIM];
    double acceleration[N_DIM];
} __attribute__((packed)) obstacle_t;

typedef struct spline {
	std::vector<double> knots;
	int degree;
	std::vector<std::vector<double>> transform;
}spline_t;


class MotionPlanning{
    private:
        double current_time=0.0;
        double horizon_time;
        double update_time;
        double sample_time;
        casadi::NlpSolver problem;
        std::map<std::string, casadi::DMatrix> args, sol;
        std::vector<double> parameters;
        std::vector<double> variables;
        std::vector<double> lbg;
        std::vector<double> ubg;
        std::vector<std::vector<double>> y_coeffs;
        std::vector<double> time;
        std::vector<std::vector<double>> input_trajectory;
        std::vector<std::vector<double>> state_trajectory;
        std::vector<std::vector<double>> y0;
        std::vector<std::vector<double>> yT;
        std::vector<std::vector<double>> ypred;
        std::map<std::string, spline_t> splines;
        std::string solver_output;

        bool ideal_update;
        const int n_y = N_Y;
        const int n_var = N_VAR;
        const int n_par = N_PAR;
        const int n_con = N_CON;
        const int n_der = N_DER;
        const int order = ORDER;
        const int y_degree = Y_DEGREE;
        const std::vector<std::vector<std::vector<double>>> derivative_T = DERT_DEF;

        void generateProblem();
        void initSplines();
        bool solve(std::vector<obstacle_t>&);
        void setParameters(std::vector<obstacle_t>&);
        void initVariables();
        void updateBounds(double);
        void interpreteVariables();
        void predict(std::vector<double>&, std::vector<std::vector<double>>&, std::vector<std::vector<double>>&, double);
        void integrate(std::vector<double>&, std::vector<std::vector<double>>&, std::vector<double>&, int);
        void sampleSplines(std::vector<std::vector<double>>&, std::vector<std::vector<double>>&, std::vector<std::vector<double>>&);
        double evalSpline(double, std::vector<double>&, std::vector<double>&, int);
        void transformSplines(double);
        void updateModel(std::vector<double>&, std::vector<double>&, std::vector<double>&);
        void getY(std::vector<double>&, std::vector<double>&, std::vector<std::vector<double>>&);
        void getState(std::vector<std::vector<double>>&, std::vector<double>&);
        void getInput(std::vector<std::vector<double>>&, std::vector<double>&);

    public:
        const int n_dim = N_DIM;
        const int n_obs = N_OBS;
        const int n_in = N_IN;
        const int n_st = N_ST;

        MotionPlanning(double updateTime, double sampleTime, double horizonTime);
        bool update(std::vector<double>&, std::vector<double>&, std::vector<double>&, std::vector<std::vector<double>>&, std::vector<std::vector<double>>&, std::vector<obstacle_t>&);
        void setIdealUpdate(bool);
    };
}

#endif
