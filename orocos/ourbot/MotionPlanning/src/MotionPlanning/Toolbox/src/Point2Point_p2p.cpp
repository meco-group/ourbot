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

#include "Point2Point_p2p.hpp"
#ifdef DEBUG
#include <ctime>
#endif
#include <unistd.h>

using namespace std;
using namespace casadi;

namespace p2p{

Point2Point::Point2Point(Vehicle* vehicle,
    double update_time, double sample_time, double horizon_time, int trajectory_length, bool initialize):
parameters(N_PAR), variables(N_VAR), lbg(LBG_DEF), ubg(UBG_DEF), _recover(false) {
    if (trajectory_length > int(horizon_time/sample_time)){
        cerr << "trajectory_length > (horizon_time/sample_time)!" << endl;
    }
    if (trajectory_length > int(update_time/sample_time)){
        time.resize(trajectory_length+1);
        state_trajectory.resize(trajectory_length+1, vector<double>(vehicle->getNState()));
        input_trajectory.resize(trajectory_length+1, vector<double>(vehicle->getNInput()));
    }
    else {
        time.resize(int(update_time/sample_time)+1);
        state_trajectory.resize(int(update_time/sample_time)+1, vector<double>(vehicle->getNState()));
        input_trajectory.resize(int(update_time/sample_time)+1, vector<double>(vehicle->getNInput()));
    }
    this->vehicle = vehicle;
    this->update_time = update_time;
    this->sample_time = sample_time;
    this->horizon_time = horizon_time;
    this->trajectory_length = trajectory_length;
    for (int k=0; k<time.size(); k++){
        time[k] = k*sample_time;
    }
    if (initialize){
        this->initialize();
    }
}

Point2Point::Point2Point(Vehicle* vehicle,
    double update_time, double sample_time, double horizon_time):
Point2Point(vehicle, update_time, sample_time, horizon_time, int(update_time/sample_time), true){
}

Point2Point::Point2Point(Vehicle* vehicle,
    double update_time, double sample_time, double horizon_time, int trajectory_length):
Point2Point(vehicle, update_time, sample_time, horizon_time, trajectory_length, true){
}

void Point2Point::initialize(){
    generateProblem();
    generateSubstituteFunctions();
    args["p"] = parameters;
    args["x0"] = variables;
    args["lbg"] = lbg;
    args["ubg"] = ubg;
    initSplines();
}

void Point2Point::generateProblem(){
    string obj_path = CASADIOBJ;
    // set options
    Dict options;
    options["ipopt.print_level"] = 0;
    options["print_time"] = 0;
    options["ipopt.tol"] = TOL;
    options["ipopt.linear_solver"] = LINEAR_SOLVER;
    options["ipopt.warm_start_init_point"] = "yes";
    // create nlp solver
    this->problem = nlpsol("problem", "ipopt", obj_path+"/nlp.so", options);
}

void Point2Point::generateSubstituteFunctions(){
	string obj_path = CASADIOBJ;

}

void Point2Point::initSplines(){
	splines_tf["splines0"] = SPLINES0_TF;
	splines_tf["eps_00"] = EPS_00_TF;
	splines_tf["eps_01"] = EPS_01_TF;
	splines_tf["g0"] = G0_TF;
	splines_tf["g1"] = G1_TF;
	splines_tf["a_vehicle0_00"] = A_VEHICLE0_00_TF;
	splines_tf["b_vehicle0_00"] = B_VEHICLE0_00_TF;
	splines_tf["a_vehicle0_01"] = A_VEHICLE0_01_TF;
	splines_tf["b_vehicle0_01"] = B_VEHICLE0_01_TF;

}

void Point2Point::reset(){
    for (int k=0; k<input_trajectory.size(); k++){
        for (int j=0; j<input_trajectory[0].size(); j++){
            input_trajectory[k][j] = 0.0;
        }
    }
}

void Point2Point::resetTime(){
    current_time = 0.0;
    current_time_prev = 0.0;
}

void Point2Point::recover(){
    // this method should be called (by user) when encountered infeasibilities
    _recover = true;
}

bool Point2Point::update(vector<double>& condition0, vector<double>& conditionT,
    vector<vector<double>>& state_trajectory, vector<vector<double>>& input_trajectory, vector<obstacle_t>& obstacles){
    update(condition0, conditionT, state_trajectory, input_trajectory, obstacles, 0);
}

bool Point2Point::update(vector<double>& condition0, vector<double>& conditionT,
    vector<vector<double>>& state_trajectory, vector<vector<double>>& input_trajectory,
    vector<obstacle_t>& obstacles, int predict_shift){
    #ifdef DEBUG
    double tmeas;
    clock_t begin;
    clock_t end;
    #endif
    // correct current_time with predict_shift:
    current_time += predict_shift*sample_time;
    // transform splines: good init guess for this update
    #ifdef DEBUG
    begin = clock();
    #endif
    transformSplines(current_time, current_time_prev);
    #ifdef DEBUG
    end = clock();
    tmeas = double(end-begin)/CLOCKS_PER_SEC;
    cout << "time in transformSplines: " << tmeas << "s" << endl;
    #endif
    // set target condition
    #ifdef DEBUG
    begin = clock();
    #endif
    vehicle->setTerminalConditions(conditionT);
    #ifdef DEBUG
    end = clock();
    tmeas = double(end-begin)/CLOCKS_PER_SEC;
    cout << "time in setTerminalConditions: " << tmeas << "s" << endl;
    #endif
    // predict initial state and input for problem
    #ifdef DEBUG
    begin = clock();
    #endif
    if (fabs(current_time)<=1.e-6){
        vehicle->setInitialConditions(condition0);
    } else{
        vehicle->predict(condition0, this->state_trajectory, this->input_trajectory, update_time, sample_time, predict_shift);
    }
    #ifdef DEBUG
    end = clock();
    tmeas = double(end-begin)/CLOCKS_PER_SEC;
    cout << "time in predict: " << tmeas << "s" << endl;
    #endif
    // solve problem
    #ifdef DEBUG
    begin = clock();
    #endif
    bool check = solve(current_time, obstacles);
    #ifdef DEBUG
    end = clock();
    tmeas = double(end-begin)/CLOCKS_PER_SEC;
    cout << "time in solve: " << tmeas << "s" << endl;
    #endif
    if (!check){
        current_time_prev = current_time; // prevent to transform again after infeasible!
        return false; // user should retry
    }
    // retrieve splines
    #ifdef DEBUG
    begin = clock();
    #endif
    extractData();
    #ifdef DEBUG
    end = clock();
    tmeas = double(end-begin)/CLOCKS_PER_SEC;
    cout << "time in extractData: " << tmeas << "s" << endl;
    #endif
    // write output state and input trajectory
    for (int k=0; k<trajectory_length; k++){
        for (int j=0; j<state_trajectory[0].size(); j++){
            state_trajectory[k][j] = this->state_trajectory[k][j];
        }
        for (int j=0; j<input_trajectory[0].size(); j++){
            input_trajectory[k][j] = this->input_trajectory[k][j];
        }
    }
    // update current time
    current_time_prev = current_time;
    current_time += update_time;
    return true;
}

bool Point2Point::solve(double current_time, vector<obstacle_t>& obstacles){
    // init variables if first time
    if(fabs(current_time)<=1.e-6 || _recover){
        initVariables();
        _recover = false;
    }
    updateBounds(current_time, obstacles);
    setParameters(obstacles);
    args["p"] = parameters;
    args["x0"] = variables;
    args["lbg"] = lbg;
    args["ubg"] = ubg;
    sol = problem(args);
    solver_output = string(problem.stats().at("return_status"));
    if (solver_output.compare("Solve_Succeeded") != 0){
        cout << solver_output << endl;
        return false;
    } else{
        vector<double> var(sol.at("x"));
        for (int k=0; k<n_var; k++){
            variables[k] = var[k];
        }
        return true;
    }
}

void Point2Point::getCoefficients(std::vector<double>& coeffs){
    coeffs = std::vector<double>(spline_coeffs_vec);
}

int Point2Point::getLenBasis(){
    return vehicle->getLenBasis();
}

void Point2Point::initVariables(){
    map<string, map<string, vector<double>>> var_dict;
    int n_spl = vehicle->getNSplines();
    int len_basis = vehicle->getLenBasis();
    vector<vector<double>> init_var_veh (n_spl, vector<double>(len_basis));
    vehicle->getInitSplineValue(init_var_veh);
    vector<double> init_var_veh_vec(n_spl*len_basis);
    for (int k=0; k<n_spl; k++){
        for (int j=0; j<len_basis; j++){
            init_var_veh_vec[k*len_basis+j] = init_var_veh[k][j];
        }
    }
    var_dict[VEHICLELBL]["splines0"] = init_var_veh_vec;
    getVariableVector(variables, var_dict);
}

void Point2Point::setParameters(vector<obstacle_t>& obstacles){
    map<string, map<string, vector<double>>> par_dict;
    fillParameterDict(obstacles, par_dict);
    getParameterVector(parameters, par_dict);
}

void Point2Point::fillParameterDict(vector<obstacle_t>& obstacles, map<string, map<string, vector<double>>>& par_dict){
    map<string, vector<double>> par_dict_veh;
    vehicle->setParameters(par_dict_veh);
    par_dict[VEHICLELBL] = par_dict_veh;
    if (!freeT){
        par_dict[P2PLBL]["t"] = {fmod(round(current_time*1000.)/1000., horizon_time/(vehicle->getKnotIntervals()))};
        par_dict[P2PLBL]["T"] = {horizon_time};
    } else{
        par_dict[P2PLBL]["t"] = {0.0};
    }

	std::vector<double> pos0(2), vel0(2), acc0(2);
	std::vector<double> posT(2), velT(2), accT(2);
	pos0 = obstacles[0].position;
	vel0 = obstacles[0].velocity;
	acc0 = obstacles[0].acceleration;
	// prediction over update_time
	for (int j=0; j<2; j++){
		posT[j] = pos0[j] + update_time*vel0[j] + 0.5*pow(update_time,2)*acc0[j];
		velT[j] = vel0[j] + update_time*acc0[j];
		accT[j] = acc0[j];
	}
	par_dict["obstacle0"]["x"] = posT;
	par_dict["obstacle0"]["v"] = velT;
	par_dict["obstacle0"]["a"] = accT;
	par_dict["obstacle0"]["checkpoints"] = obstacles[0].checkpoints;
	par_dict["obstacle0"]["rad"] = obstacles[0].radii;


	pos0 = obstacles[1].position;
	vel0 = obstacles[1].velocity;
	acc0 = obstacles[1].acceleration;
	// prediction over update_time
	for (int j=0; j<2; j++){
		posT[j] = pos0[j] + update_time*vel0[j] + 0.5*pow(update_time,2)*acc0[j];
		velT[j] = vel0[j] + update_time*acc0[j];
		accT[j] = acc0[j];
	}
	par_dict["obstacle1"]["x"] = posT;
	par_dict["obstacle1"]["v"] = velT;
	par_dict["obstacle1"]["a"] = accT;
	par_dict["obstacle1"]["checkpoints"] = obstacles[1].checkpoints;
	par_dict["obstacle1"]["rad"] = obstacles[1].radii;


}

void Point2Point::extractData(){
    map<string, map<string, vector<double>>> var_dict;
    getVariableDict(variables, var_dict);
    spline_coeffs_vec = std::vector<double>(var_dict[VEHICLELBL]["splines0"]);
    vehicle->setKnotHorizon(horizon_time);
    if (freeT){
        horizon_time = var_dict[P2PLBL]["T"][0];
    }
    vehicle->setKnotHorizon(horizon_time);
    int n_spl = vehicle->getNSplines();
    int len_basis = vehicle->getLenBasis();
    vector<vector<double>> spline_coeffs(n_spl, vector<double>(len_basis));
    for (int k=0; k<n_spl; k++){
        for (int j=0; j<len_basis; j++){
            spline_coeffs[k][j] = spline_coeffs_vec[k*len_basis+j];
        }
    }
    retrieveTrajectories(spline_coeffs);
}

void Point2Point::retrieveTrajectories(vector<vector<double>>& spline_coeffs){
    vector<double> time(this->time);
    if (!freeT){
        for (int k=0; k<time.size(); k++){
            time[k] += fmod(round(current_time*1000.)/1000., horizon_time/vehicle->getKnotIntervals());
        }
    }
    vehicle->splines2State(spline_coeffs, time, state_trajectory);
    vehicle->splines2Input(spline_coeffs, time, input_trajectory);
}

void Point2Point::getParameterVector(vector<double>& par_vect, map<string, map<string, vector<double>>>& par_dict){
	for (int i=0; i<2; i++){
		par_vect[0+i] = par_dict["vehicle0"]["state0"][i];
	}
	for (int i=0; i<2; i++){
		par_vect[2+i] = par_dict["vehicle0"]["input0"][i];
	}
	for (int i=0; i<2; i++){
		par_vect[4+i] = par_dict["vehicle0"]["poseT"][i];
	}
	for (int i=0; i<2; i++){
		par_vect[6+i] = par_dict["obstacle0"]["x"][i];
	}
	for (int i=0; i<2; i++){
		par_vect[8+i] = par_dict["obstacle0"]["v"][i];
	}
	for (int i=0; i<2; i++){
		par_vect[10+i] = par_dict["obstacle0"]["a"][i];
	}
	for (int i=0; i<8; i++){
		par_vect[12+i] = par_dict["obstacle0"]["checkpoints"][i];
	}
	for (int i=0; i<4; i++){
		par_vect[20+i] = par_dict["obstacle0"]["rad"][i];
	}
	for (int i=0; i<2; i++){
		par_vect[24+i] = par_dict["obstacle1"]["x"][i];
	}
	for (int i=0; i<2; i++){
		par_vect[26+i] = par_dict["obstacle1"]["v"][i];
	}
	for (int i=0; i<2; i++){
		par_vect[28+i] = par_dict["obstacle1"]["a"][i];
	}
	for (int i=0; i<8; i++){
		par_vect[30+i] = par_dict["obstacle1"]["checkpoints"][i];
	}
	for (int i=0; i<4; i++){
		par_vect[38+i] = par_dict["obstacle1"]["rad"][i];
	}
	par_vect[42] = par_dict["p2p0"]["T"][0];
	par_vect[43] = par_dict["p2p0"]["t"][0];

}

void Point2Point::getVariableVector(vector<double>& var_vect, map<string, map<string, vector<double>>>& var_dict){
	if (var_dict.find("vehicle0") != var_dict.end()){
		if (var_dict["vehicle0"].find("splines0") != var_dict["vehicle0"].end()){
			for (int i=0; i<26; i++){
				var_vect[0+i] = var_dict["vehicle0"]["splines0"][i];
			}
		}
		if (var_dict["vehicle0"].find("eps_00") != var_dict["vehicle0"].end()){
			for (int i=0; i<13; i++){
				var_vect[26+i] = var_dict["vehicle0"]["eps_00"][i];
			}
		}
		if (var_dict["vehicle0"].find("eps_01") != var_dict["vehicle0"].end()){
			for (int i=0; i<13; i++){
				var_vect[39+i] = var_dict["vehicle0"]["eps_01"][i];
			}
		}
	}
	if (var_dict.find("obstacle0") != var_dict.end()){
	}
	if (var_dict.find("obstacle1") != var_dict.end()){
	}
	if (var_dict.find("p2p0") != var_dict.end()){
		if (var_dict["p2p0"].find("g0") != var_dict["p2p0"].end()){
			for (int i=0; i<13; i++){
				var_vect[52+i] = var_dict["p2p0"]["g0"][i];
			}
		}
		if (var_dict["p2p0"].find("g1") != var_dict["p2p0"].end()){
			for (int i=0; i<13; i++){
				var_vect[65+i] = var_dict["p2p0"]["g1"][i];
			}
		}
	}
	if (var_dict.find("environment0") != var_dict.end()){
		if (var_dict["environment0"].find("a_vehicle0_00") != var_dict["environment0"].end()){
			for (int i=0; i<22; i++){
				var_vect[78+i] = var_dict["environment0"]["a_vehicle0_00"][i];
			}
		}
		if (var_dict["environment0"].find("b_vehicle0_00") != var_dict["environment0"].end()){
			for (int i=0; i<11; i++){
				var_vect[100+i] = var_dict["environment0"]["b_vehicle0_00"][i];
			}
		}
		if (var_dict["environment0"].find("a_vehicle0_01") != var_dict["environment0"].end()){
			for (int i=0; i<22; i++){
				var_vect[111+i] = var_dict["environment0"]["a_vehicle0_01"][i];
			}
		}
		if (var_dict["environment0"].find("b_vehicle0_01") != var_dict["environment0"].end()){
			for (int i=0; i<11; i++){
				var_vect[133+i] = var_dict["environment0"]["b_vehicle0_01"][i];
			}
		}
	}

}

void Point2Point::getVariableDict(vector<double>& var_vect, map<string, map<string, vector<double>>>& var_dict){
	vector<double> vec;	vec.resize(26);
	for (int i=0; i<26; i++){
		vec[i] = var_vect[0+i];
	}
	var_dict["vehicle0"]["splines0"] = vec;
	vec.resize(13);
	for (int i=0; i<13; i++){
		vec[i] = var_vect[26+i];
	}
	var_dict["vehicle0"]["eps_00"] = vec;
	vec.resize(13);
	for (int i=0; i<13; i++){
		vec[i] = var_vect[39+i];
	}
	var_dict["vehicle0"]["eps_01"] = vec;
	vec.resize(13);
	for (int i=0; i<13; i++){
		vec[i] = var_vect[52+i];
	}
	var_dict["p2p0"]["g0"] = vec;
	vec.resize(13);
	for (int i=0; i<13; i++){
		vec[i] = var_vect[65+i];
	}
	var_dict["p2p0"]["g1"] = vec;
	vec.resize(22);
	for (int i=0; i<22; i++){
		vec[i] = var_vect[78+i];
	}
	var_dict["environment0"]["a_vehicle0_00"] = vec;
	vec.resize(11);
	for (int i=0; i<11; i++){
		vec[i] = var_vect[100+i];
	}
	var_dict["environment0"]["b_vehicle0_00"] = vec;
	vec.resize(22);
	for (int i=0; i<22; i++){
		vec[i] = var_vect[111+i];
	}
	var_dict["environment0"]["a_vehicle0_01"] = vec;
	vec.resize(11);
	for (int i=0; i<11; i++){
		vec[i] = var_vect[133+i];
	}
	var_dict["environment0"]["b_vehicle0_01"] = vec;

}

void Point2Point::updateBounds(double current_time, vector<obstacle_t>& obstacles){
	if(!obstacles[0].avoid){
		lbg[472] = -inf;
		ubg[472] = +inf;
		lbg[473] = -inf;
		ubg[473] = +inf;
		lbg[474] = -inf;
		ubg[474] = +inf;
		lbg[475] = -inf;
		ubg[475] = +inf;
		lbg[476] = -inf;
		ubg[476] = +inf;
		lbg[477] = -inf;
		ubg[477] = +inf;
		lbg[478] = -inf;
		ubg[478] = +inf;
		lbg[479] = -inf;
		ubg[479] = +inf;
		lbg[480] = -inf;
		ubg[480] = +inf;
		lbg[481] = -inf;
		ubg[481] = +inf;
		lbg[482] = -inf;
		ubg[482] = +inf;
		lbg[483] = -inf;
		ubg[483] = +inf;
		lbg[484] = -inf;
		ubg[484] = +inf;
		lbg[485] = -inf;
		ubg[485] = +inf;
		lbg[486] = -inf;
		ubg[486] = +inf;
		lbg[487] = -inf;
		ubg[487] = +inf;
		lbg[488] = -inf;
		ubg[488] = +inf;
		lbg[489] = -inf;
		ubg[489] = +inf;
		lbg[490] = -inf;
		ubg[490] = +inf;
		lbg[491] = -inf;
		ubg[491] = +inf;
		lbg[492] = -inf;
		ubg[492] = +inf;
		lbg[493] = -inf;
		ubg[493] = +inf;
		lbg[494] = -inf;
		ubg[494] = +inf;
		lbg[495] = -inf;
		ubg[495] = +inf;
		lbg[496] = -inf;
		ubg[496] = +inf;
		lbg[497] = -inf;
		ubg[497] = +inf;
		lbg[498] = -inf;
		ubg[498] = +inf;
		lbg[499] = -inf;
		ubg[499] = +inf;
		lbg[500] = -inf;
		ubg[500] = +inf;
		lbg[501] = -inf;
		ubg[501] = +inf;
		lbg[502] = -inf;
		ubg[502] = +inf;
		lbg[503] = -inf;
		ubg[503] = +inf;
		lbg[504] = -inf;
		ubg[504] = +inf;
		lbg[505] = -inf;
		ubg[505] = +inf;
		lbg[506] = -inf;
		ubg[506] = +inf;
		lbg[507] = -inf;
		ubg[507] = +inf;
		lbg[508] = -inf;
		ubg[508] = +inf;
		lbg[509] = -inf;
		ubg[509] = +inf;
		lbg[510] = -inf;
		ubg[510] = +inf;
		lbg[511] = -inf;
		ubg[511] = +inf;
		lbg[512] = -inf;
		ubg[512] = +inf;
		lbg[513] = -inf;
		ubg[513] = +inf;
		lbg[514] = -inf;
		ubg[514] = +inf;
		lbg[515] = -inf;
		ubg[515] = +inf;
		lbg[516] = -inf;
		ubg[516] = +inf;
		lbg[517] = -inf;
		ubg[517] = +inf;
		lbg[518] = -inf;
		ubg[518] = +inf;
		lbg[519] = -inf;
		ubg[519] = +inf;
		lbg[520] = -inf;
		ubg[520] = +inf;
		lbg[521] = -inf;
		ubg[521] = +inf;
		lbg[522] = -inf;
		ubg[522] = +inf;
		lbg[523] = -inf;
		ubg[523] = +inf;
		lbg[524] = -inf;
		ubg[524] = +inf;
		lbg[525] = -inf;
		ubg[525] = +inf;
		lbg[526] = -inf;
		ubg[526] = +inf;
		lbg[527] = -inf;
		ubg[527] = +inf;
		lbg[528] = -inf;
		ubg[528] = +inf;
		lbg[529] = -inf;
		ubg[529] = +inf;
		lbg[530] = -inf;
		ubg[530] = +inf;
		lbg[531] = -inf;
		ubg[531] = +inf;
		lbg[532] = -inf;
		ubg[532] = +inf;
		lbg[533] = -inf;
		ubg[533] = +inf;
		lbg[534] = -inf;
		ubg[534] = +inf;
		lbg[535] = -inf;
		ubg[535] = +inf;
		lbg[536] = -inf;
		ubg[536] = +inf;
		lbg[537] = -inf;
		ubg[537] = +inf;
		lbg[538] = -inf;
		ubg[538] = +inf;
		lbg[539] = -inf;
		ubg[539] = +inf;
		lbg[540] = -inf;
		ubg[540] = +inf;
		lbg[541] = -inf;
		ubg[541] = +inf;
		lbg[542] = -inf;
		ubg[542] = +inf;
		lbg[543] = -inf;
		ubg[543] = +inf;
		lbg[544] = -inf;
		ubg[544] = +inf;
		lbg[545] = -inf;
		ubg[545] = +inf;
		lbg[546] = -inf;
		ubg[546] = +inf;
		lbg[547] = -inf;
		ubg[547] = +inf;
		lbg[548] = -inf;
		ubg[548] = +inf;
		lbg[549] = -inf;
		ubg[549] = +inf;
		lbg[550] = -inf;
		ubg[550] = +inf;
		lbg[551] = -inf;
		ubg[551] = +inf;
		lbg[552] = -inf;
		ubg[552] = +inf;
		lbg[553] = -inf;
		ubg[553] = +inf;
		lbg[554] = -inf;
		ubg[554] = +inf;
		lbg[555] = -inf;
		ubg[555] = +inf;
		lbg[556] = -inf;
		ubg[556] = +inf;
		lbg[557] = -inf;
		ubg[557] = +inf;
		lbg[558] = -inf;
		ubg[558] = +inf;
		lbg[559] = -inf;
		ubg[559] = +inf;
		lbg[560] = -inf;
		ubg[560] = +inf;
		lbg[561] = -inf;
		ubg[561] = +inf;
		lbg[562] = -inf;
		ubg[562] = +inf;
		lbg[563] = -inf;
		ubg[563] = +inf;
		lbg[564] = -inf;
		ubg[564] = +inf;
		lbg[565] = -inf;
		ubg[565] = +inf;
		lbg[566] = -inf;
		ubg[566] = +inf;
		lbg[567] = -inf;
		ubg[567] = +inf;
		lbg[568] = -inf;
		ubg[568] = +inf;
		lbg[569] = -inf;
		ubg[569] = +inf;
		lbg[570] = -inf;
		ubg[570] = +inf;
		lbg[571] = -inf;
		ubg[571] = +inf;
		lbg[572] = -inf;
		ubg[572] = +inf;
		lbg[573] = -inf;
		ubg[573] = +inf;
		lbg[574] = -inf;
		ubg[574] = +inf;
		lbg[575] = -inf;
		ubg[575] = +inf;
		lbg[576] = -inf;
		ubg[576] = +inf;
		lbg[577] = -inf;
		ubg[577] = +inf;
		lbg[578] = -inf;
		ubg[578] = +inf;
		lbg[579] = -inf;
		ubg[579] = +inf;
		lbg[580] = -inf;
		ubg[580] = +inf;
		lbg[581] = -inf;
		ubg[581] = +inf;
		lbg[582] = -inf;
		ubg[582] = +inf;
		lbg[583] = -inf;
		ubg[583] = +inf;
		lbg[584] = -inf;
		ubg[584] = +inf;
		lbg[585] = -inf;
		ubg[585] = +inf;
		lbg[586] = -inf;
		ubg[586] = +inf;
		lbg[587] = -inf;
		ubg[587] = +inf;
		lbg[588] = -inf;
		ubg[588] = +inf;
		lbg[589] = -inf;
		ubg[589] = +inf;
		lbg[590] = -inf;
		ubg[590] = +inf;
		lbg[591] = -inf;
		ubg[591] = +inf;
		lbg[592] = -inf;
		ubg[592] = +inf;
		lbg[593] = -inf;
		ubg[593] = +inf;
		lbg[594] = -inf;
		ubg[594] = +inf;
		lbg[595] = -inf;
		ubg[595] = +inf;
	}else{
		lbg[472] = -inf;
		ubg[472] = 0;
		lbg[473] = -inf;
		ubg[473] = 0;
		lbg[474] = -inf;
		ubg[474] = 0;
		lbg[475] = -inf;
		ubg[475] = 0;
		lbg[476] = -inf;
		ubg[476] = 0;
		lbg[477] = -inf;
		ubg[477] = 0;
		lbg[478] = -inf;
		ubg[478] = 0;
		lbg[479] = -inf;
		ubg[479] = 0;
		lbg[480] = -inf;
		ubg[480] = 0;
		lbg[481] = -inf;
		ubg[481] = 0;
		lbg[482] = -inf;
		ubg[482] = 0;
		lbg[483] = -inf;
		ubg[483] = 0;
		lbg[484] = -inf;
		ubg[484] = 0;
		lbg[485] = -inf;
		ubg[485] = 0;
		lbg[486] = -inf;
		ubg[486] = 0;
		lbg[487] = -inf;
		ubg[487] = 0;
		lbg[488] = -inf;
		ubg[488] = 0;
		lbg[489] = -inf;
		ubg[489] = 0;
		lbg[490] = -inf;
		ubg[490] = 0;
		lbg[491] = -inf;
		ubg[491] = 0;
		lbg[492] = -inf;
		ubg[492] = 0;
		lbg[493] = -inf;
		ubg[493] = 0;
		lbg[494] = -inf;
		ubg[494] = 0;
		lbg[495] = -inf;
		ubg[495] = 0;
		lbg[496] = -inf;
		ubg[496] = 0;
		lbg[497] = -inf;
		ubg[497] = 0;
		lbg[498] = -inf;
		ubg[498] = 0;
		lbg[499] = -inf;
		ubg[499] = 0;
		lbg[500] = -inf;
		ubg[500] = 0;
		lbg[501] = -inf;
		ubg[501] = 0;
		lbg[502] = -inf;
		ubg[502] = 0;
		lbg[503] = -inf;
		ubg[503] = 0;
		lbg[504] = -inf;
		ubg[504] = 0;
		lbg[505] = -inf;
		ubg[505] = 0;
		lbg[506] = -inf;
		ubg[506] = 0;
		lbg[507] = -inf;
		ubg[507] = 0;
		lbg[508] = -inf;
		ubg[508] = 0;
		lbg[509] = -inf;
		ubg[509] = 0;
		lbg[510] = -inf;
		ubg[510] = 0;
		lbg[511] = -inf;
		ubg[511] = 0;
		lbg[512] = -inf;
		ubg[512] = 0;
		lbg[513] = -inf;
		ubg[513] = 0;
		lbg[514] = -inf;
		ubg[514] = 0;
		lbg[515] = -inf;
		ubg[515] = 0;
		lbg[516] = -inf;
		ubg[516] = 0;
		lbg[517] = -inf;
		ubg[517] = 0;
		lbg[518] = -inf;
		ubg[518] = 0;
		lbg[519] = -inf;
		ubg[519] = 0;
		lbg[520] = -inf;
		ubg[520] = 0;
		lbg[521] = -inf;
		ubg[521] = 0;
		lbg[522] = -inf;
		ubg[522] = 0;
		lbg[523] = -inf;
		ubg[523] = 0;
		lbg[524] = -inf;
		ubg[524] = 0;
		lbg[525] = -inf;
		ubg[525] = 0;
		lbg[526] = -inf;
		ubg[526] = 0;
		lbg[527] = -inf;
		ubg[527] = 0;
		lbg[528] = -inf;
		ubg[528] = 0;
		lbg[529] = -inf;
		ubg[529] = 0;
		lbg[530] = -inf;
		ubg[530] = 0;
		lbg[531] = -inf;
		ubg[531] = 0;
		lbg[532] = -inf;
		ubg[532] = 0;
		lbg[533] = -inf;
		ubg[533] = 0;
		lbg[534] = -inf;
		ubg[534] = 0;
		lbg[535] = -inf;
		ubg[535] = 0;
		lbg[536] = -inf;
		ubg[536] = 0;
		lbg[537] = -inf;
		ubg[537] = 0;
		lbg[538] = -inf;
		ubg[538] = 0;
		lbg[539] = -inf;
		ubg[539] = 0;
		lbg[540] = -inf;
		ubg[540] = 0;
		lbg[541] = -inf;
		ubg[541] = 0;
		lbg[542] = -inf;
		ubg[542] = 0;
		lbg[543] = -inf;
		ubg[543] = 0;
		lbg[544] = -inf;
		ubg[544] = 0;
		lbg[545] = -inf;
		ubg[545] = 0;
		lbg[546] = -inf;
		ubg[546] = 0;
		lbg[547] = -inf;
		ubg[547] = 0;
		lbg[548] = -inf;
		ubg[548] = 0;
		lbg[549] = -inf;
		ubg[549] = 0;
		lbg[550] = -inf;
		ubg[550] = 0;
		lbg[551] = -inf;
		ubg[551] = 0;
		lbg[552] = -inf;
		ubg[552] = 0;
		lbg[553] = -inf;
		ubg[553] = 0;
		lbg[554] = -inf;
		ubg[554] = 0;
		lbg[555] = -inf;
		ubg[555] = 0;
		lbg[556] = -inf;
		ubg[556] = 0;
		lbg[557] = -inf;
		ubg[557] = 0;
		lbg[558] = -inf;
		ubg[558] = 0;
		lbg[559] = -inf;
		ubg[559] = 0;
		lbg[560] = -inf;
		ubg[560] = 0;
		lbg[561] = -inf;
		ubg[561] = 0;
		lbg[562] = -inf;
		ubg[562] = 0;
		lbg[563] = -inf;
		ubg[563] = 0;
		lbg[564] = -inf;
		ubg[564] = 0;
		lbg[565] = -inf;
		ubg[565] = 0;
		lbg[566] = -inf;
		ubg[566] = 0;
		lbg[567] = -inf;
		ubg[567] = 0;
		lbg[568] = -inf;
		ubg[568] = 0;
		lbg[569] = -inf;
		ubg[569] = 0;
		lbg[570] = -inf;
		ubg[570] = 0;
		lbg[571] = -inf;
		ubg[571] = 0;
		lbg[572] = -inf;
		ubg[572] = 0;
		lbg[573] = -inf;
		ubg[573] = 0;
		lbg[574] = -inf;
		ubg[574] = 0;
		lbg[575] = -inf;
		ubg[575] = 0;
		lbg[576] = -inf;
		ubg[576] = 0;
		lbg[577] = -inf;
		ubg[577] = 0;
		lbg[578] = -inf;
		ubg[578] = 0;
		lbg[579] = -inf;
		ubg[579] = 0;
		lbg[580] = -inf;
		ubg[580] = 0;
		lbg[581] = -inf;
		ubg[581] = 0;
		lbg[582] = -inf;
		ubg[582] = 0;
		lbg[583] = -inf;
		ubg[583] = 0;
		lbg[584] = -inf;
		ubg[584] = 0;
		lbg[585] = -inf;
		ubg[585] = 0;
		lbg[586] = -inf;
		ubg[586] = 0;
		lbg[587] = -inf;
		ubg[587] = 0;
		lbg[588] = -inf;
		ubg[588] = 0;
		lbg[589] = -inf;
		ubg[589] = 0;
		lbg[590] = -inf;
		ubg[590] = 0;
		lbg[591] = -inf;
		ubg[591] = 0;
		lbg[592] = -inf;
		ubg[592] = 0;
		lbg[593] = -inf;
		ubg[593] = 0;
		lbg[594] = -inf;
		ubg[594] = 0;
		lbg[595] = -inf;
		ubg[595] = 0;
	}
	if(!obstacles[1].avoid){
		lbg[596] = -inf;
		ubg[596] = +inf;
		lbg[597] = -inf;
		ubg[597] = +inf;
		lbg[598] = -inf;
		ubg[598] = +inf;
		lbg[599] = -inf;
		ubg[599] = +inf;
		lbg[600] = -inf;
		ubg[600] = +inf;
		lbg[601] = -inf;
		ubg[601] = +inf;
		lbg[602] = -inf;
		ubg[602] = +inf;
		lbg[603] = -inf;
		ubg[603] = +inf;
		lbg[604] = -inf;
		ubg[604] = +inf;
		lbg[605] = -inf;
		ubg[605] = +inf;
		lbg[606] = -inf;
		ubg[606] = +inf;
		lbg[607] = -inf;
		ubg[607] = +inf;
		lbg[608] = -inf;
		ubg[608] = +inf;
		lbg[609] = -inf;
		ubg[609] = +inf;
		lbg[610] = -inf;
		ubg[610] = +inf;
		lbg[611] = -inf;
		ubg[611] = +inf;
		lbg[612] = -inf;
		ubg[612] = +inf;
		lbg[613] = -inf;
		ubg[613] = +inf;
		lbg[614] = -inf;
		ubg[614] = +inf;
		lbg[615] = -inf;
		ubg[615] = +inf;
		lbg[616] = -inf;
		ubg[616] = +inf;
		lbg[617] = -inf;
		ubg[617] = +inf;
		lbg[618] = -inf;
		ubg[618] = +inf;
		lbg[619] = -inf;
		ubg[619] = +inf;
		lbg[620] = -inf;
		ubg[620] = +inf;
		lbg[621] = -inf;
		ubg[621] = +inf;
		lbg[622] = -inf;
		ubg[622] = +inf;
		lbg[623] = -inf;
		ubg[623] = +inf;
		lbg[624] = -inf;
		ubg[624] = +inf;
		lbg[625] = -inf;
		ubg[625] = +inf;
		lbg[626] = -inf;
		ubg[626] = +inf;
		lbg[627] = -inf;
		ubg[627] = +inf;
		lbg[628] = -inf;
		ubg[628] = +inf;
		lbg[629] = -inf;
		ubg[629] = +inf;
		lbg[630] = -inf;
		ubg[630] = +inf;
		lbg[631] = -inf;
		ubg[631] = +inf;
		lbg[632] = -inf;
		ubg[632] = +inf;
		lbg[633] = -inf;
		ubg[633] = +inf;
		lbg[634] = -inf;
		ubg[634] = +inf;
		lbg[635] = -inf;
		ubg[635] = +inf;
		lbg[636] = -inf;
		ubg[636] = +inf;
		lbg[637] = -inf;
		ubg[637] = +inf;
		lbg[638] = -inf;
		ubg[638] = +inf;
		lbg[639] = -inf;
		ubg[639] = +inf;
		lbg[640] = -inf;
		ubg[640] = +inf;
		lbg[641] = -inf;
		ubg[641] = +inf;
		lbg[642] = -inf;
		ubg[642] = +inf;
		lbg[643] = -inf;
		ubg[643] = +inf;
		lbg[644] = -inf;
		ubg[644] = +inf;
		lbg[645] = -inf;
		ubg[645] = +inf;
		lbg[646] = -inf;
		ubg[646] = +inf;
		lbg[647] = -inf;
		ubg[647] = +inf;
		lbg[648] = -inf;
		ubg[648] = +inf;
		lbg[649] = -inf;
		ubg[649] = +inf;
		lbg[650] = -inf;
		ubg[650] = +inf;
		lbg[651] = -inf;
		ubg[651] = +inf;
		lbg[652] = -inf;
		ubg[652] = +inf;
		lbg[653] = -inf;
		ubg[653] = +inf;
		lbg[654] = -inf;
		ubg[654] = +inf;
		lbg[655] = -inf;
		ubg[655] = +inf;
		lbg[656] = -inf;
		ubg[656] = +inf;
		lbg[657] = -inf;
		ubg[657] = +inf;
		lbg[658] = -inf;
		ubg[658] = +inf;
		lbg[659] = -inf;
		ubg[659] = +inf;
		lbg[660] = -inf;
		ubg[660] = +inf;
		lbg[661] = -inf;
		ubg[661] = +inf;
		lbg[662] = -inf;
		ubg[662] = +inf;
		lbg[663] = -inf;
		ubg[663] = +inf;
		lbg[664] = -inf;
		ubg[664] = +inf;
		lbg[665] = -inf;
		ubg[665] = +inf;
		lbg[666] = -inf;
		ubg[666] = +inf;
		lbg[667] = -inf;
		ubg[667] = +inf;
		lbg[668] = -inf;
		ubg[668] = +inf;
		lbg[669] = -inf;
		ubg[669] = +inf;
		lbg[670] = -inf;
		ubg[670] = +inf;
		lbg[671] = -inf;
		ubg[671] = +inf;
		lbg[672] = -inf;
		ubg[672] = +inf;
		lbg[673] = -inf;
		ubg[673] = +inf;
		lbg[674] = -inf;
		ubg[674] = +inf;
		lbg[675] = -inf;
		ubg[675] = +inf;
		lbg[676] = -inf;
		ubg[676] = +inf;
		lbg[677] = -inf;
		ubg[677] = +inf;
		lbg[678] = -inf;
		ubg[678] = +inf;
		lbg[679] = -inf;
		ubg[679] = +inf;
		lbg[680] = -inf;
		ubg[680] = +inf;
		lbg[681] = -inf;
		ubg[681] = +inf;
		lbg[682] = -inf;
		ubg[682] = +inf;
		lbg[683] = -inf;
		ubg[683] = +inf;
		lbg[684] = -inf;
		ubg[684] = +inf;
		lbg[685] = -inf;
		ubg[685] = +inf;
		lbg[686] = -inf;
		ubg[686] = +inf;
		lbg[687] = -inf;
		ubg[687] = +inf;
		lbg[688] = -inf;
		ubg[688] = +inf;
		lbg[689] = -inf;
		ubg[689] = +inf;
		lbg[690] = -inf;
		ubg[690] = +inf;
		lbg[691] = -inf;
		ubg[691] = +inf;
		lbg[692] = -inf;
		ubg[692] = +inf;
		lbg[693] = -inf;
		ubg[693] = +inf;
		lbg[694] = -inf;
		ubg[694] = +inf;
		lbg[695] = -inf;
		ubg[695] = +inf;
		lbg[696] = -inf;
		ubg[696] = +inf;
		lbg[697] = -inf;
		ubg[697] = +inf;
		lbg[698] = -inf;
		ubg[698] = +inf;
		lbg[699] = -inf;
		ubg[699] = +inf;
		lbg[700] = -inf;
		ubg[700] = +inf;
		lbg[701] = -inf;
		ubg[701] = +inf;
		lbg[702] = -inf;
		ubg[702] = +inf;
		lbg[703] = -inf;
		ubg[703] = +inf;
		lbg[704] = -inf;
		ubg[704] = +inf;
		lbg[705] = -inf;
		ubg[705] = +inf;
		lbg[706] = -inf;
		ubg[706] = +inf;
		lbg[707] = -inf;
		ubg[707] = +inf;
		lbg[708] = -inf;
		ubg[708] = +inf;
		lbg[709] = -inf;
		ubg[709] = +inf;
		lbg[710] = -inf;
		ubg[710] = +inf;
		lbg[711] = -inf;
		ubg[711] = +inf;
		lbg[712] = -inf;
		ubg[712] = +inf;
		lbg[713] = -inf;
		ubg[713] = +inf;
		lbg[714] = -inf;
		ubg[714] = +inf;
		lbg[715] = -inf;
		ubg[715] = +inf;
		lbg[716] = -inf;
		ubg[716] = +inf;
		lbg[717] = -inf;
		ubg[717] = +inf;
		lbg[718] = -inf;
		ubg[718] = +inf;
		lbg[719] = -inf;
		ubg[719] = +inf;
	}else{
		lbg[596] = -inf;
		ubg[596] = 0;
		lbg[597] = -inf;
		ubg[597] = 0;
		lbg[598] = -inf;
		ubg[598] = 0;
		lbg[599] = -inf;
		ubg[599] = 0;
		lbg[600] = -inf;
		ubg[600] = 0;
		lbg[601] = -inf;
		ubg[601] = 0;
		lbg[602] = -inf;
		ubg[602] = 0;
		lbg[603] = -inf;
		ubg[603] = 0;
		lbg[604] = -inf;
		ubg[604] = 0;
		lbg[605] = -inf;
		ubg[605] = 0;
		lbg[606] = -inf;
		ubg[606] = 0;
		lbg[607] = -inf;
		ubg[607] = 0;
		lbg[608] = -inf;
		ubg[608] = 0;
		lbg[609] = -inf;
		ubg[609] = 0;
		lbg[610] = -inf;
		ubg[610] = 0;
		lbg[611] = -inf;
		ubg[611] = 0;
		lbg[612] = -inf;
		ubg[612] = 0;
		lbg[613] = -inf;
		ubg[613] = 0;
		lbg[614] = -inf;
		ubg[614] = 0;
		lbg[615] = -inf;
		ubg[615] = 0;
		lbg[616] = -inf;
		ubg[616] = 0;
		lbg[617] = -inf;
		ubg[617] = 0;
		lbg[618] = -inf;
		ubg[618] = 0;
		lbg[619] = -inf;
		ubg[619] = 0;
		lbg[620] = -inf;
		ubg[620] = 0;
		lbg[621] = -inf;
		ubg[621] = 0;
		lbg[622] = -inf;
		ubg[622] = 0;
		lbg[623] = -inf;
		ubg[623] = 0;
		lbg[624] = -inf;
		ubg[624] = 0;
		lbg[625] = -inf;
		ubg[625] = 0;
		lbg[626] = -inf;
		ubg[626] = 0;
		lbg[627] = -inf;
		ubg[627] = 0;
		lbg[628] = -inf;
		ubg[628] = 0;
		lbg[629] = -inf;
		ubg[629] = 0;
		lbg[630] = -inf;
		ubg[630] = 0;
		lbg[631] = -inf;
		ubg[631] = 0;
		lbg[632] = -inf;
		ubg[632] = 0;
		lbg[633] = -inf;
		ubg[633] = 0;
		lbg[634] = -inf;
		ubg[634] = 0;
		lbg[635] = -inf;
		ubg[635] = 0;
		lbg[636] = -inf;
		ubg[636] = 0;
		lbg[637] = -inf;
		ubg[637] = 0;
		lbg[638] = -inf;
		ubg[638] = 0;
		lbg[639] = -inf;
		ubg[639] = 0;
		lbg[640] = -inf;
		ubg[640] = 0;
		lbg[641] = -inf;
		ubg[641] = 0;
		lbg[642] = -inf;
		ubg[642] = 0;
		lbg[643] = -inf;
		ubg[643] = 0;
		lbg[644] = -inf;
		ubg[644] = 0;
		lbg[645] = -inf;
		ubg[645] = 0;
		lbg[646] = -inf;
		ubg[646] = 0;
		lbg[647] = -inf;
		ubg[647] = 0;
		lbg[648] = -inf;
		ubg[648] = 0;
		lbg[649] = -inf;
		ubg[649] = 0;
		lbg[650] = -inf;
		ubg[650] = 0;
		lbg[651] = -inf;
		ubg[651] = 0;
		lbg[652] = -inf;
		ubg[652] = 0;
		lbg[653] = -inf;
		ubg[653] = 0;
		lbg[654] = -inf;
		ubg[654] = 0;
		lbg[655] = -inf;
		ubg[655] = 0;
		lbg[656] = -inf;
		ubg[656] = 0;
		lbg[657] = -inf;
		ubg[657] = 0;
		lbg[658] = -inf;
		ubg[658] = 0;
		lbg[659] = -inf;
		ubg[659] = 0;
		lbg[660] = -inf;
		ubg[660] = 0;
		lbg[661] = -inf;
		ubg[661] = 0;
		lbg[662] = -inf;
		ubg[662] = 0;
		lbg[663] = -inf;
		ubg[663] = 0;
		lbg[664] = -inf;
		ubg[664] = 0;
		lbg[665] = -inf;
		ubg[665] = 0;
		lbg[666] = -inf;
		ubg[666] = 0;
		lbg[667] = -inf;
		ubg[667] = 0;
		lbg[668] = -inf;
		ubg[668] = 0;
		lbg[669] = -inf;
		ubg[669] = 0;
		lbg[670] = -inf;
		ubg[670] = 0;
		lbg[671] = -inf;
		ubg[671] = 0;
		lbg[672] = -inf;
		ubg[672] = 0;
		lbg[673] = -inf;
		ubg[673] = 0;
		lbg[674] = -inf;
		ubg[674] = 0;
		lbg[675] = -inf;
		ubg[675] = 0;
		lbg[676] = -inf;
		ubg[676] = 0;
		lbg[677] = -inf;
		ubg[677] = 0;
		lbg[678] = -inf;
		ubg[678] = 0;
		lbg[679] = -inf;
		ubg[679] = 0;
		lbg[680] = -inf;
		ubg[680] = 0;
		lbg[681] = -inf;
		ubg[681] = 0;
		lbg[682] = -inf;
		ubg[682] = 0;
		lbg[683] = -inf;
		ubg[683] = 0;
		lbg[684] = -inf;
		ubg[684] = 0;
		lbg[685] = -inf;
		ubg[685] = 0;
		lbg[686] = -inf;
		ubg[686] = 0;
		lbg[687] = -inf;
		ubg[687] = 0;
		lbg[688] = -inf;
		ubg[688] = 0;
		lbg[689] = -inf;
		ubg[689] = 0;
		lbg[690] = -inf;
		ubg[690] = 0;
		lbg[691] = -inf;
		ubg[691] = 0;
		lbg[692] = -inf;
		ubg[692] = 0;
		lbg[693] = -inf;
		ubg[693] = 0;
		lbg[694] = -inf;
		ubg[694] = 0;
		lbg[695] = -inf;
		ubg[695] = 0;
		lbg[696] = -inf;
		ubg[696] = 0;
		lbg[697] = -inf;
		ubg[697] = 0;
		lbg[698] = -inf;
		ubg[698] = 0;
		lbg[699] = -inf;
		ubg[699] = 0;
		lbg[700] = -inf;
		ubg[700] = 0;
		lbg[701] = -inf;
		ubg[701] = 0;
		lbg[702] = -inf;
		ubg[702] = 0;
		lbg[703] = -inf;
		ubg[703] = 0;
		lbg[704] = -inf;
		ubg[704] = 0;
		lbg[705] = -inf;
		ubg[705] = 0;
		lbg[706] = -inf;
		ubg[706] = 0;
		lbg[707] = -inf;
		ubg[707] = 0;
		lbg[708] = -inf;
		ubg[708] = 0;
		lbg[709] = -inf;
		ubg[709] = 0;
		lbg[710] = -inf;
		ubg[710] = 0;
		lbg[711] = -inf;
		ubg[711] = 0;
		lbg[712] = -inf;
		ubg[712] = 0;
		lbg[713] = -inf;
		ubg[713] = 0;
		lbg[714] = -inf;
		ubg[714] = 0;
		lbg[715] = -inf;
		ubg[715] = 0;
		lbg[716] = -inf;
		ubg[716] = 0;
		lbg[717] = -inf;
		ubg[717] = 0;
		lbg[718] = -inf;
		ubg[718] = 0;
		lbg[719] = -inf;
		ubg[719] = 0;
	}

}

void Point2Point::transformSplines(double current_time, double current_time_prev){
	int interval_prev = (int)(round((current_time_prev*(vehicle->getKnotIntervals())/horizon_time)*1.e6)/1.e6);
	int interval_now = (int)(round((current_time*(vehicle->getKnotIntervals())/horizon_time)*1.e6)/1.e6);
	if(interval_now > interval_prev){
		vector<double> spline_tf(13);
		for(int k=0; k<2; k++){
			for(int i=0; i<13; i++){
			spline_tf[i] = 0.0;
				for(int j=0; j<13; j++){
					spline_tf[i] += splines_tf["splines0"][i][j]*variables[0+k*13+j];
				}
			}
			for(int i=0; i<13; i++){
				variables[0+k*13+i] = spline_tf[i];
			}
		}
		for(int k=0; k<1; k++){
			for(int i=0; i<13; i++){
			spline_tf[i] = 0.0;
				for(int j=0; j<13; j++){
					spline_tf[i] += splines_tf["eps_00"][i][j]*variables[13+k*13+j];
				}
			}
			for(int i=0; i<13; i++){
				variables[13+k*13+i] = spline_tf[i];
			}
		}
		for(int k=0; k<1; k++){
			for(int i=0; i<13; i++){
			spline_tf[i] = 0.0;
				for(int j=0; j<13; j++){
					spline_tf[i] += splines_tf["eps_01"][i][j]*variables[26+k*13+j];
				}
			}
			for(int i=0; i<13; i++){
				variables[26+k*13+i] = spline_tf[i];
			}
		}
		for(int k=0; k<1; k++){
			for(int i=0; i<13; i++){
			spline_tf[i] = 0.0;
				for(int j=0; j<13; j++){
					spline_tf[i] += splines_tf["g0"][i][j]*variables[39+k*13+j];
				}
			}
			for(int i=0; i<13; i++){
				variables[39+k*13+i] = spline_tf[i];
			}
		}
		for(int k=0; k<1; k++){
			for(int i=0; i<13; i++){
			spline_tf[i] = 0.0;
				for(int j=0; j<13; j++){
					spline_tf[i] += splines_tf["g1"][i][j]*variables[52+k*13+j];
				}
			}
			for(int i=0; i<13; i++){
				variables[52+k*13+i] = spline_tf[i];
			}
		}
		for(int k=0; k<2; k++){
			for(int i=0; i<11; i++){
			spline_tf[i] = 0.0;
				for(int j=0; j<11; j++){
					spline_tf[i] += splines_tf["a_vehicle0_00"][i][j]*variables[65+k*11+j];
				}
			}
			for(int i=0; i<11; i++){
				variables[65+k*11+i] = spline_tf[i];
			}
		}
		for(int k=0; k<1; k++){
			for(int i=0; i<11; i++){
			spline_tf[i] = 0.0;
				for(int j=0; j<11; j++){
					spline_tf[i] += splines_tf["b_vehicle0_00"][i][j]*variables[76+k*11+j];
				}
			}
			for(int i=0; i<11; i++){
				variables[76+k*11+i] = spline_tf[i];
			}
		}
		for(int k=0; k<2; k++){
			for(int i=0; i<11; i++){
			spline_tf[i] = 0.0;
				for(int j=0; j<11; j++){
					spline_tf[i] += splines_tf["a_vehicle0_01"][i][j]*variables[87+k*11+j];
				}
			}
			for(int i=0; i<11; i++){
				variables[87+k*11+i] = spline_tf[i];
			}
		}
		for(int k=0; k<1; k++){
			for(int i=0; i<11; i++){
			spline_tf[i] = 0.0;
				for(int j=0; j<11; j++){
					spline_tf[i] += splines_tf["b_vehicle0_01"][i][j]*variables[98+k*11+j];
				}
			}
			for(int i=0; i<11; i++){
				variables[98+k*11+i] = spline_tf[i];
			}
		}
	}

}


}
