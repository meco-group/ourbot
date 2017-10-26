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
	splines_tf["g0"] = G0_TF;
	splines_tf["g1"] = G1_TF;
	splines_tf["a_vehicle0_00"] = A_VEHICLE0_00_TF;
	splines_tf["b_vehicle0_00"] = B_VEHICLE0_00_TF;

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
	par_vect[24] = par_dict["p2p0"]["T"][0];
	par_vect[25] = par_dict["p2p0"]["t"][0];

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
	}
	if (var_dict.find("obstacle0") != var_dict.end()){
	}
	if (var_dict.find("p2p0") != var_dict.end()){
		if (var_dict["p2p0"].find("g0") != var_dict["p2p0"].end()){
			for (int i=0; i<13; i++){
				var_vect[39+i] = var_dict["p2p0"]["g0"][i];
			}
		}
		if (var_dict["p2p0"].find("g1") != var_dict["p2p0"].end()){
			for (int i=0; i<13; i++){
				var_vect[52+i] = var_dict["p2p0"]["g1"][i];
			}
		}
	}
	if (var_dict.find("environment0") != var_dict.end()){
		if (var_dict["environment0"].find("a_vehicle0_00") != var_dict["environment0"].end()){
			for (int i=0; i<22; i++){
				var_vect[65+i] = var_dict["environment0"]["a_vehicle0_00"][i];
			}
		}
		if (var_dict["environment0"].find("b_vehicle0_00") != var_dict["environment0"].end()){
			for (int i=0; i<11; i++){
				var_vect[87+i] = var_dict["environment0"]["b_vehicle0_00"][i];
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
	var_dict["p2p0"]["g0"] = vec;
	vec.resize(13);
	for (int i=0; i<13; i++){
		vec[i] = var_vect[52+i];
	}
	var_dict["p2p0"]["g1"] = vec;
	vec.resize(22);
	for (int i=0; i<22; i++){
		vec[i] = var_vect[65+i];
	}
	var_dict["environment0"]["a_vehicle0_00"] = vec;
	vec.resize(11);
	for (int i=0; i<11; i++){
		vec[i] = var_vect[87+i];
	}
	var_dict["environment0"]["b_vehicle0_00"] = vec;

}

void Point2Point::updateBounds(double current_time, vector<obstacle_t>& obstacles){
	if(!obstacles[0].avoid){
		lbg[282] = -inf;
		ubg[282] = +inf;
		lbg[283] = -inf;
		ubg[283] = +inf;
		lbg[284] = -inf;
		ubg[284] = +inf;
		lbg[285] = -inf;
		ubg[285] = +inf;
		lbg[286] = -inf;
		ubg[286] = +inf;
		lbg[287] = -inf;
		ubg[287] = +inf;
		lbg[288] = -inf;
		ubg[288] = +inf;
		lbg[289] = -inf;
		ubg[289] = +inf;
		lbg[290] = -inf;
		ubg[290] = +inf;
		lbg[291] = -inf;
		ubg[291] = +inf;
		lbg[292] = -inf;
		ubg[292] = +inf;
		lbg[293] = -inf;
		ubg[293] = +inf;
		lbg[294] = -inf;
		ubg[294] = +inf;
		lbg[295] = -inf;
		ubg[295] = +inf;
		lbg[296] = -inf;
		ubg[296] = +inf;
		lbg[297] = -inf;
		ubg[297] = +inf;
		lbg[298] = -inf;
		ubg[298] = +inf;
		lbg[299] = -inf;
		ubg[299] = +inf;
		lbg[300] = -inf;
		ubg[300] = +inf;
		lbg[301] = -inf;
		ubg[301] = +inf;
		lbg[302] = -inf;
		ubg[302] = +inf;
		lbg[303] = -inf;
		ubg[303] = +inf;
		lbg[304] = -inf;
		ubg[304] = +inf;
		lbg[305] = -inf;
		ubg[305] = +inf;
		lbg[306] = -inf;
		ubg[306] = +inf;
		lbg[307] = -inf;
		ubg[307] = +inf;
		lbg[308] = -inf;
		ubg[308] = +inf;
		lbg[309] = -inf;
		ubg[309] = +inf;
		lbg[310] = -inf;
		ubg[310] = +inf;
		lbg[311] = -inf;
		ubg[311] = +inf;
		lbg[312] = -inf;
		ubg[312] = +inf;
		lbg[313] = -inf;
		ubg[313] = +inf;
		lbg[314] = -inf;
		ubg[314] = +inf;
		lbg[315] = -inf;
		ubg[315] = +inf;
		lbg[316] = -inf;
		ubg[316] = +inf;
		lbg[317] = -inf;
		ubg[317] = +inf;
		lbg[318] = -inf;
		ubg[318] = +inf;
		lbg[319] = -inf;
		ubg[319] = +inf;
		lbg[320] = -inf;
		ubg[320] = +inf;
		lbg[321] = -inf;
		ubg[321] = +inf;
		lbg[322] = -inf;
		ubg[322] = +inf;
		lbg[323] = -inf;
		ubg[323] = +inf;
		lbg[324] = -inf;
		ubg[324] = +inf;
		lbg[325] = -inf;
		ubg[325] = +inf;
		lbg[326] = -inf;
		ubg[326] = +inf;
		lbg[327] = -inf;
		ubg[327] = +inf;
		lbg[328] = -inf;
		ubg[328] = +inf;
		lbg[329] = -inf;
		ubg[329] = +inf;
		lbg[330] = -inf;
		ubg[330] = +inf;
		lbg[331] = -inf;
		ubg[331] = +inf;
		lbg[332] = -inf;
		ubg[332] = +inf;
		lbg[333] = -inf;
		ubg[333] = +inf;
		lbg[334] = -inf;
		ubg[334] = +inf;
		lbg[335] = -inf;
		ubg[335] = +inf;
		lbg[336] = -inf;
		ubg[336] = +inf;
		lbg[337] = -inf;
		ubg[337] = +inf;
		lbg[338] = -inf;
		ubg[338] = +inf;
		lbg[339] = -inf;
		ubg[339] = +inf;
		lbg[340] = -inf;
		ubg[340] = +inf;
		lbg[341] = -inf;
		ubg[341] = +inf;
		lbg[342] = -inf;
		ubg[342] = +inf;
		lbg[343] = -inf;
		ubg[343] = +inf;
		lbg[344] = -inf;
		ubg[344] = +inf;
		lbg[345] = -inf;
		ubg[345] = +inf;
		lbg[346] = -inf;
		ubg[346] = +inf;
		lbg[347] = -inf;
		ubg[347] = +inf;
		lbg[348] = -inf;
		ubg[348] = +inf;
		lbg[349] = -inf;
		ubg[349] = +inf;
		lbg[350] = -inf;
		ubg[350] = +inf;
		lbg[351] = -inf;
		ubg[351] = +inf;
		lbg[352] = -inf;
		ubg[352] = +inf;
		lbg[353] = -inf;
		ubg[353] = +inf;
		lbg[354] = -inf;
		ubg[354] = +inf;
		lbg[355] = -inf;
		ubg[355] = +inf;
		lbg[356] = -inf;
		ubg[356] = +inf;
		lbg[357] = -inf;
		ubg[357] = +inf;
		lbg[358] = -inf;
		ubg[358] = +inf;
		lbg[359] = -inf;
		ubg[359] = +inf;
		lbg[360] = -inf;
		ubg[360] = +inf;
		lbg[361] = -inf;
		ubg[361] = +inf;
		lbg[362] = -inf;
		ubg[362] = +inf;
		lbg[363] = -inf;
		ubg[363] = +inf;
		lbg[364] = -inf;
		ubg[364] = +inf;
		lbg[365] = -inf;
		ubg[365] = +inf;
		lbg[366] = -inf;
		ubg[366] = +inf;
		lbg[367] = -inf;
		ubg[367] = +inf;
		lbg[368] = -inf;
		ubg[368] = +inf;
		lbg[369] = -inf;
		ubg[369] = +inf;
		lbg[370] = -inf;
		ubg[370] = +inf;
		lbg[371] = -inf;
		ubg[371] = +inf;
		lbg[372] = -inf;
		ubg[372] = +inf;
		lbg[373] = -inf;
		ubg[373] = +inf;
		lbg[374] = -inf;
		ubg[374] = +inf;
		lbg[375] = -inf;
		ubg[375] = +inf;
		lbg[376] = -inf;
		ubg[376] = +inf;
		lbg[377] = -inf;
		ubg[377] = +inf;
		lbg[378] = -inf;
		ubg[378] = +inf;
		lbg[379] = -inf;
		ubg[379] = +inf;
		lbg[380] = -inf;
		ubg[380] = +inf;
		lbg[381] = -inf;
		ubg[381] = +inf;
		lbg[382] = -inf;
		ubg[382] = +inf;
		lbg[383] = -inf;
		ubg[383] = +inf;
		lbg[384] = -inf;
		ubg[384] = +inf;
		lbg[385] = -inf;
		ubg[385] = +inf;
		lbg[386] = -inf;
		ubg[386] = +inf;
		lbg[387] = -inf;
		ubg[387] = +inf;
		lbg[388] = -inf;
		ubg[388] = +inf;
		lbg[389] = -inf;
		ubg[389] = +inf;
		lbg[390] = -inf;
		ubg[390] = +inf;
		lbg[391] = -inf;
		ubg[391] = +inf;
		lbg[392] = -inf;
		ubg[392] = +inf;
		lbg[393] = -inf;
		ubg[393] = +inf;
		lbg[394] = -inf;
		ubg[394] = +inf;
		lbg[395] = -inf;
		ubg[395] = +inf;
		lbg[396] = -inf;
		ubg[396] = +inf;
		lbg[397] = -inf;
		ubg[397] = +inf;
		lbg[398] = -inf;
		ubg[398] = +inf;
		lbg[399] = -inf;
		ubg[399] = +inf;
		lbg[400] = -inf;
		ubg[400] = +inf;
		lbg[401] = -inf;
		ubg[401] = +inf;
		lbg[402] = -inf;
		ubg[402] = +inf;
		lbg[403] = -inf;
		ubg[403] = +inf;
		lbg[404] = -inf;
		ubg[404] = +inf;
		lbg[405] = -inf;
		ubg[405] = +inf;
	}else{
		lbg[282] = -inf;
		ubg[282] = 0;
		lbg[283] = -inf;
		ubg[283] = 0;
		lbg[284] = -inf;
		ubg[284] = 0;
		lbg[285] = -inf;
		ubg[285] = 0;
		lbg[286] = -inf;
		ubg[286] = 0;
		lbg[287] = -inf;
		ubg[287] = 0;
		lbg[288] = -inf;
		ubg[288] = 0;
		lbg[289] = -inf;
		ubg[289] = 0;
		lbg[290] = -inf;
		ubg[290] = 0;
		lbg[291] = -inf;
		ubg[291] = 0;
		lbg[292] = -inf;
		ubg[292] = 0;
		lbg[293] = -inf;
		ubg[293] = 0;
		lbg[294] = -inf;
		ubg[294] = 0;
		lbg[295] = -inf;
		ubg[295] = 0;
		lbg[296] = -inf;
		ubg[296] = 0;
		lbg[297] = -inf;
		ubg[297] = 0;
		lbg[298] = -inf;
		ubg[298] = 0;
		lbg[299] = -inf;
		ubg[299] = 0;
		lbg[300] = -inf;
		ubg[300] = 0;
		lbg[301] = -inf;
		ubg[301] = 0;
		lbg[302] = -inf;
		ubg[302] = 0;
		lbg[303] = -inf;
		ubg[303] = 0;
		lbg[304] = -inf;
		ubg[304] = 0;
		lbg[305] = -inf;
		ubg[305] = 0;
		lbg[306] = -inf;
		ubg[306] = 0;
		lbg[307] = -inf;
		ubg[307] = 0;
		lbg[308] = -inf;
		ubg[308] = 0;
		lbg[309] = -inf;
		ubg[309] = 0;
		lbg[310] = -inf;
		ubg[310] = 0;
		lbg[311] = -inf;
		ubg[311] = 0;
		lbg[312] = -inf;
		ubg[312] = 0;
		lbg[313] = -inf;
		ubg[313] = 0;
		lbg[314] = -inf;
		ubg[314] = 0;
		lbg[315] = -inf;
		ubg[315] = 0;
		lbg[316] = -inf;
		ubg[316] = 0;
		lbg[317] = -inf;
		ubg[317] = 0;
		lbg[318] = -inf;
		ubg[318] = 0;
		lbg[319] = -inf;
		ubg[319] = 0;
		lbg[320] = -inf;
		ubg[320] = 0;
		lbg[321] = -inf;
		ubg[321] = 0;
		lbg[322] = -inf;
		ubg[322] = 0;
		lbg[323] = -inf;
		ubg[323] = 0;
		lbg[324] = -inf;
		ubg[324] = 0;
		lbg[325] = -inf;
		ubg[325] = 0;
		lbg[326] = -inf;
		ubg[326] = 0;
		lbg[327] = -inf;
		ubg[327] = 0;
		lbg[328] = -inf;
		ubg[328] = 0;
		lbg[329] = -inf;
		ubg[329] = 0;
		lbg[330] = -inf;
		ubg[330] = 0;
		lbg[331] = -inf;
		ubg[331] = 0;
		lbg[332] = -inf;
		ubg[332] = 0;
		lbg[333] = -inf;
		ubg[333] = 0;
		lbg[334] = -inf;
		ubg[334] = 0;
		lbg[335] = -inf;
		ubg[335] = 0;
		lbg[336] = -inf;
		ubg[336] = 0;
		lbg[337] = -inf;
		ubg[337] = 0;
		lbg[338] = -inf;
		ubg[338] = 0;
		lbg[339] = -inf;
		ubg[339] = 0;
		lbg[340] = -inf;
		ubg[340] = 0;
		lbg[341] = -inf;
		ubg[341] = 0;
		lbg[342] = -inf;
		ubg[342] = 0;
		lbg[343] = -inf;
		ubg[343] = 0;
		lbg[344] = -inf;
		ubg[344] = 0;
		lbg[345] = -inf;
		ubg[345] = 0;
		lbg[346] = -inf;
		ubg[346] = 0;
		lbg[347] = -inf;
		ubg[347] = 0;
		lbg[348] = -inf;
		ubg[348] = 0;
		lbg[349] = -inf;
		ubg[349] = 0;
		lbg[350] = -inf;
		ubg[350] = 0;
		lbg[351] = -inf;
		ubg[351] = 0;
		lbg[352] = -inf;
		ubg[352] = 0;
		lbg[353] = -inf;
		ubg[353] = 0;
		lbg[354] = -inf;
		ubg[354] = 0;
		lbg[355] = -inf;
		ubg[355] = 0;
		lbg[356] = -inf;
		ubg[356] = 0;
		lbg[357] = -inf;
		ubg[357] = 0;
		lbg[358] = -inf;
		ubg[358] = 0;
		lbg[359] = -inf;
		ubg[359] = 0;
		lbg[360] = -inf;
		ubg[360] = 0;
		lbg[361] = -inf;
		ubg[361] = 0;
		lbg[362] = -inf;
		ubg[362] = 0;
		lbg[363] = -inf;
		ubg[363] = 0;
		lbg[364] = -inf;
		ubg[364] = 0;
		lbg[365] = -inf;
		ubg[365] = 0;
		lbg[366] = -inf;
		ubg[366] = 0;
		lbg[367] = -inf;
		ubg[367] = 0;
		lbg[368] = -inf;
		ubg[368] = 0;
		lbg[369] = -inf;
		ubg[369] = 0;
		lbg[370] = -inf;
		ubg[370] = 0;
		lbg[371] = -inf;
		ubg[371] = 0;
		lbg[372] = -inf;
		ubg[372] = 0;
		lbg[373] = -inf;
		ubg[373] = 0;
		lbg[374] = -inf;
		ubg[374] = 0;
		lbg[375] = -inf;
		ubg[375] = 0;
		lbg[376] = -inf;
		ubg[376] = 0;
		lbg[377] = -inf;
		ubg[377] = 0;
		lbg[378] = -inf;
		ubg[378] = 0;
		lbg[379] = -inf;
		ubg[379] = 0;
		lbg[380] = -inf;
		ubg[380] = 0;
		lbg[381] = -inf;
		ubg[381] = 0;
		lbg[382] = -inf;
		ubg[382] = 0;
		lbg[383] = -inf;
		ubg[383] = 0;
		lbg[384] = -inf;
		ubg[384] = 0;
		lbg[385] = -inf;
		ubg[385] = 0;
		lbg[386] = -inf;
		ubg[386] = 0;
		lbg[387] = -inf;
		ubg[387] = 0;
		lbg[388] = -inf;
		ubg[388] = 0;
		lbg[389] = -inf;
		ubg[389] = 0;
		lbg[390] = -inf;
		ubg[390] = 0;
		lbg[391] = -inf;
		ubg[391] = 0;
		lbg[392] = -inf;
		ubg[392] = 0;
		lbg[393] = -inf;
		ubg[393] = 0;
		lbg[394] = -inf;
		ubg[394] = 0;
		lbg[395] = -inf;
		ubg[395] = 0;
		lbg[396] = -inf;
		ubg[396] = 0;
		lbg[397] = -inf;
		ubg[397] = 0;
		lbg[398] = -inf;
		ubg[398] = 0;
		lbg[399] = -inf;
		ubg[399] = 0;
		lbg[400] = -inf;
		ubg[400] = 0;
		lbg[401] = -inf;
		ubg[401] = 0;
		lbg[402] = -inf;
		ubg[402] = 0;
		lbg[403] = -inf;
		ubg[403] = 0;
		lbg[404] = -inf;
		ubg[404] = 0;
		lbg[405] = -inf;
		ubg[405] = 0;
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
					spline_tf[i] += splines_tf["g0"][i][j]*variables[26+k*13+j];
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
					spline_tf[i] += splines_tf["g1"][i][j]*variables[39+k*13+j];
				}
			}
			for(int i=0; i<13; i++){
				variables[39+k*13+i] = spline_tf[i];
			}
		}
		for(int k=0; k<2; k++){
			for(int i=0; i<11; i++){
			spline_tf[i] = 0.0;
				for(int j=0; j<11; j++){
					spline_tf[i] += splines_tf["a_vehicle0_00"][i][j]*variables[52+k*11+j];
				}
			}
			for(int i=0; i<11; i++){
				variables[52+k*11+i] = spline_tf[i];
			}
		}
		for(int k=0; k<1; k++){
			for(int i=0; i<11; i++){
			spline_tf[i] = 0.0;
				for(int j=0; j<11; j++){
					spline_tf[i] += splines_tf["b_vehicle0_00"][i][j]*variables[63+k*11+j];
				}
			}
			for(int i=0; i<11; i++){
				variables[63+k*11+i] = spline_tf[i];
			}
		}
	}

}


}
