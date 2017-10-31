// This file is part of OMG-tools.

// OMG-tools -- Optimal Motion Generation-tools
// Copyright (C) 2016 Ruben Van Parys & Tim Mercy, KU Leuven.
// All rights reserved.

// OMG-tools is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation; either
// version 3 of the License, or (at your option) any later version.
// This software is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
// Lesser General Public License for more details.

// You should have received a copy of the GNU Lesser General Public
// License along with this program; if not, write to the Free Software
// Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA

#ifndef POINT2POINT_P2P
#define POINT2POINT_P2P

#include "Vehicle_p2p.hpp"
#include <casadi/casadi.hpp>
#include <math.h>
#include <memory.h>
#include <iostream>

namespace p2p{

const double inf = std::numeric_limits<double>::infinity();
const int N_PAR = 8;
const std::vector<std::vector<double>> G1_TF = {{0.0,0.25,0.583333333333,0.166666666667,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0},{0.0,0.0,0.666666666667,0.333333333333,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0},{0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0},{0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0},{0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0},{0.0,0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0},{0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0},{0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0},{0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0},{0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0},{0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,-0.5,1.5,0.0},{0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.5,-3.5,4.0},{0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,-0.166666666667,2.41666666667,-9.25,8.0}};
const std::string LINEAR_SOLVER = "ma57";
const int N_DIM = 2;
const double TOL = 0.001;
const int N_VAR = 52;
const std::vector<std::vector<double>> SPLINES0_TF = {{0.0,0.25,0.583333333333,0.166666666667,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0},{0.0,0.0,0.666666666667,0.333333333333,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0},{0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0},{0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0},{0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0},{0.0,0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0},{0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0},{0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0},{0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0},{0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0},{0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,-0.5,1.5,0.0},{0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.5,-3.5,4.0},{0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,-0.166666666667,2.41666666667,-9.25,8.0}};
const int N_CON = 154;
const bool FREET = false;
const std::vector<double> LBG_DEF = {-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,0.0,0.0,0.0,0.0,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,0.0,0.0,0.0,0.0,0.0,0.0};
const std::string VEHICLELBL = "vehicle0";
const std::vector<std::vector<double>> G0_TF = {{0.0,0.25,0.583333333333,0.166666666667,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0},{0.0,0.0,0.666666666667,0.333333333333,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0},{0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0},{0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0},{0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0},{0.0,0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0},{0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0},{0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0},{0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0},{0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0},{0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,-0.5,1.5,0.0},{0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.5,-3.5,4.0},{0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,-0.166666666667,2.41666666667,-9.25,8.0}};
const std::string P2PLBL = "p2p0";
const std::vector<double> UBG_DEF = {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};
const int N_OBS = 0;


typedef struct obstacle {
    std::vector<double> position;
    std::vector<double> velocity;
    std::vector<double> acceleration;
    std::vector<double> checkpoints;
    std::vector<double> radii;
    //To keep datatype compatible, put x coefficients first and then y coefficients in the same vector
    std::vector<double> traj_coeffs;
    bool avoid;
} obstacle_t;

class Point2Point{
    private:
        casadi::Function problem;
        std::vector<double> spline_coeffs_vec;
        bool solve(double, std::vector<obstacle_t>&);
        bool _recover;
        void generateSubstituteFunctions();
        void initSplines();

    protected:
        Vehicle* vehicle;
        double current_time=0.0;
        double current_time_prev=0.0;
        double horizon_time;
        double update_time;
        double sample_time;
        int trajectory_length;
        std::map<std::string, casadi::Function> substitutes;
        std::map<std::string, casadi::DM> args, sol;
        std::vector<double> parameters;
        std::vector<double> variables;

        std::vector<double> lbg;
        std::vector<double> ubg;
        std::vector<double> time;
        std::vector<std::vector<double>> state_trajectory;
        std::vector<std::vector<double>> input_trajectory;
        std::map<std::string, std::vector<std::vector<double>>> splines_tf;
        std::string solver_output;

        const int n_var = N_VAR;
        const int n_par = N_PAR;
        const int n_con = N_CON;
        const int freeT = FREET;

        void setParameters(std::vector<obstacle_t>&);
        void initVariables();
        void updateBounds(double, std::vector<obstacle_t>&);
        void retrieveTrajectories(std::vector<std::vector<double>>&);
        void getParameterVector(std::vector<double>&, std::map<std::string, std::map<std::string, std::vector<double>>>&);
        void getVariableVector(std::vector<double>&, std::map<std::string, std::map<std::string, std::vector<double>>>&);
        void getVariableDict(std::vector<double>&, std::map<std::string, std::map<std::string, std::vector<double>>>&);
        void transformSplines(double, double);

        virtual void generateProblem();
        virtual void fillParameterDict(std::vector<obstacle_t>&, std::map<std::string, std::map<std::string, std::vector<double>>>&);
        virtual void extractData();
        virtual void initialize();

    public:
        const int n_dim = N_DIM;
        const int n_obs = N_OBS;
        Point2Point(Vehicle* vehicle, double update_time, double sample_time, double horizon_time);
        Point2Point(Vehicle* vehicle, double update_time, double sample_time, double horizon_time, int trajectory_length);
        Point2Point(Vehicle* vehicle, double update_time, double sample_time, double horizon_time, int trajectory_length, bool initialize);
        virtual void reset();
        virtual void resetTime();
        virtual void recover();
        bool update(std::vector<double>&, std::vector<double>&, std::vector<std::vector<double>>&, std::vector<std::vector<double>>&, std::vector<obstacle_t>&);
        bool update(std::vector<double>&, std::vector<double>&, std::vector<std::vector<double>>&, std::vector<std::vector<double>>&, std::vector<obstacle_t>&, int);
        void getCoefficients(std::vector<double>& coeffs);
        int getLenBasis();
    };
}

#endif
