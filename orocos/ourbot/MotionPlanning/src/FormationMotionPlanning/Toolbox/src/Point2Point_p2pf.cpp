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

#include "Point2Point_p2pf.hpp"
#ifdef DEBUG
#include <ctime>
#endif
#include <unistd.h>

using namespace std;
using namespace casadi;

namespace p2pf{

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
	substitutes["fleet_center1"] = external("fleet_center1", obj_path+"/subst_fleet_center1.so");
	substitutes["fleet_center0"] = external("fleet_center0", obj_path+"/subst_fleet_center0.so");

}

void Point2Point::initSplines(){
	splines_tf["splines_seg0"] = SPLINES_SEG0_TF;
	splines_tf["eps_00"] = EPS_00_TF;
	splines_tf["g0"] = G0_TF;
	splines_tf["g1"] = G1_TF;
	splines_tf["a_vehicle0_seg0_00"] = A_VEHICLE0_SEG0_00_TF;
	splines_tf["b_vehicle0_seg0_00"] = B_VEHICLE0_SEG0_00_TF;
	splines_tf["xvar_fleet_center0"] = XVAR_FLEET_CENTER0_TF;
	splines_tf["xvar_fleet_center1"] = XVAR_FLEET_CENTER1_TF;

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
    var_dict[VEHICLELBL]["splines_seg0"] = init_var_veh_vec;
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
	par_dict["obstacle1"]["x"] = posT;
	par_dict["obstacle1"]["v"] = velT;
	par_dict["obstacle1"]["a"] = accT;
	par_dict["obstacle1"]["checkpoints"] = obstacles[0].checkpoints;
	par_dict["obstacle1"]["rad"] = obstacles[0].radii;


}

void Point2Point::extractData(){
    map<string, map<string, vector<double>>> var_dict;
    getVariableDict(variables, var_dict);
    spline_coeffs_vec = std::vector<double>(var_dict[VEHICLELBL]["splines_seg0"]);
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
		par_vect[0+i] = par_dict["vehicle0"]["rel_pos_c"][i];
	}
	for (int i=0; i<2; i++){
		par_vect[2+i] = par_dict["vehicle0"]["state0"][i];
	}
	for (int i=0; i<2; i++){
		par_vect[4+i] = par_dict["vehicle0"]["input0"][i];
	}
	for (int i=0; i<2; i++){
		par_vect[6+i] = par_dict["vehicle0"]["poseT"][i];
	}
	par_vect[8] = par_dict["p2p0"]["T"][0];
	par_vect[9] = par_dict["p2p0"]["t"][0];
	for (int i=0; i<26; i++){
		par_vect[10+i] = par_dict["admm0"]["z_i"][i];
	}
	for (int i=0; i<52; i++){
		par_vect[36+i] = par_dict["admm0"]["z_ji"][i];
	}
	for (int i=0; i<26; i++){
		par_vect[88+i] = par_dict["admm0"]["l_i"][i];
	}
	for (int i=0; i<52; i++){
		par_vect[114+i] = par_dict["admm0"]["l_ji"][i];
	}
	par_vect[166] = par_dict["admm0"]["rho"][0];
	for (int i=0; i<2; i++){
		par_vect[167+i] = par_dict["obstacle1"]["x"][i];
	}
	for (int i=0; i<2; i++){
		par_vect[169+i] = par_dict["obstacle1"]["v"][i];
	}
	for (int i=0; i<2; i++){
		par_vect[171+i] = par_dict["obstacle1"]["a"][i];
	}
	for (int i=0; i<8; i++){
		par_vect[173+i] = par_dict["obstacle1"]["checkpoints"][i];
	}
	for (int i=0; i<4; i++){
		par_vect[181+i] = par_dict["obstacle1"]["rad"][i];
	}

}

void Point2Point::getVariableVector(vector<double>& var_vect, map<string, map<string, vector<double>>>& var_dict){
	if (var_dict.find("vehicle0") != var_dict.end()){
		if (var_dict["vehicle0"].find("splines_seg0") != var_dict["vehicle0"].end()){
			for (int i=0; i<26; i++){
				var_vect[0+i] = var_dict["vehicle0"]["splines_seg0"][i];
			}
		}
		if (var_dict["vehicle0"].find("eps_00") != var_dict["vehicle0"].end()){
			for (int i=0; i<13; i++){
				var_vect[26+i] = var_dict["vehicle0"]["eps_00"][i];
			}
		}
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
	if (var_dict.find("environment1") != var_dict.end()){
		if (var_dict["environment1"].find("a_vehicle0_seg0_00") != var_dict["environment1"].end()){
			for (int i=0; i<22; i++){
				var_vect[65+i] = var_dict["environment1"]["a_vehicle0_seg0_00"][i];
			}
		}
		if (var_dict["environment1"].find("b_vehicle0_seg0_00") != var_dict["environment1"].end()){
			for (int i=0; i<11; i++){
				var_vect[87+i] = var_dict["environment1"]["b_vehicle0_seg0_00"][i];
			}
		}
	}
	if (var_dict.find("admm0") != var_dict.end()){
	}
	if (var_dict.find("obstacle1") != var_dict.end()){
	}

}

void Point2Point::getVariableDict(vector<double>& var_vect, map<string, map<string, vector<double>>>& var_dict){
	vector<double> vec;	vec.resize(26);
	for (int i=0; i<26; i++){
		vec[i] = var_vect[0+i];
	}
	var_dict["vehicle0"]["splines_seg0"] = vec;
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
	var_dict["environment1"]["a_vehicle0_seg0_00"] = vec;
	vec.resize(11);
	for (int i=0; i<11; i++){
		vec[i] = var_vect[87+i];
	}
	var_dict["environment1"]["b_vehicle0_seg0_00"] = vec;

}

void Point2Point::updateBounds(double current_time, vector<obstacle_t>& obstacles){
	if(!obstacles[0].avoid){
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
		lbg[406] = -inf;
		ubg[406] = +inf;
		lbg[407] = -inf;
		ubg[407] = +inf;
		lbg[408] = -inf;
		ubg[408] = +inf;
		lbg[409] = -inf;
		ubg[409] = +inf;
		lbg[410] = -inf;
		ubg[410] = +inf;
		lbg[411] = -inf;
		ubg[411] = +inf;
		lbg[412] = -inf;
		ubg[412] = +inf;
		lbg[413] = -inf;
		ubg[413] = +inf;
		lbg[414] = -inf;
		ubg[414] = +inf;
		lbg[415] = -inf;
		ubg[415] = +inf;
		lbg[416] = -inf;
		ubg[416] = +inf;
		lbg[417] = -inf;
		ubg[417] = +inf;
		lbg[418] = -inf;
		ubg[418] = +inf;
		lbg[419] = -inf;
		ubg[419] = +inf;
		lbg[420] = -inf;
		ubg[420] = +inf;
		lbg[421] = -inf;
		ubg[421] = +inf;
		lbg[422] = -inf;
		ubg[422] = +inf;
		lbg[423] = -inf;
		ubg[423] = +inf;
		lbg[424] = -inf;
		ubg[424] = +inf;
		lbg[425] = -inf;
		ubg[425] = +inf;
		lbg[426] = -inf;
		ubg[426] = +inf;
		lbg[427] = -inf;
		ubg[427] = +inf;
		lbg[428] = -inf;
		ubg[428] = +inf;
		lbg[429] = -inf;
		ubg[429] = +inf;
		lbg[430] = -inf;
		ubg[430] = +inf;
		lbg[431] = -inf;
		ubg[431] = +inf;
		lbg[432] = -inf;
		ubg[432] = +inf;
		lbg[433] = -inf;
		ubg[433] = +inf;
		lbg[434] = -inf;
		ubg[434] = +inf;
		lbg[435] = -inf;
		ubg[435] = +inf;
		lbg[436] = -inf;
		ubg[436] = +inf;
		lbg[437] = -inf;
		ubg[437] = +inf;
		lbg[438] = -inf;
		ubg[438] = +inf;
		lbg[439] = -inf;
		ubg[439] = +inf;
		lbg[440] = -inf;
		ubg[440] = +inf;
		lbg[441] = -inf;
		ubg[441] = +inf;
		lbg[442] = -inf;
		ubg[442] = +inf;
		lbg[443] = -inf;
		ubg[443] = +inf;
		lbg[444] = -inf;
		ubg[444] = +inf;
		lbg[445] = -inf;
		ubg[445] = +inf;
		lbg[446] = -inf;
		ubg[446] = +inf;
		lbg[447] = -inf;
		ubg[447] = +inf;
		lbg[448] = -inf;
		ubg[448] = +inf;
		lbg[449] = -inf;
		ubg[449] = +inf;
		lbg[450] = -inf;
		ubg[450] = +inf;
		lbg[451] = -inf;
		ubg[451] = +inf;
		lbg[452] = -inf;
		ubg[452] = +inf;
		lbg[453] = -inf;
		ubg[453] = +inf;
		lbg[454] = -inf;
		ubg[454] = +inf;
		lbg[455] = -inf;
		ubg[455] = +inf;
		lbg[456] = -inf;
		ubg[456] = +inf;
		lbg[457] = -inf;
		ubg[457] = +inf;
		lbg[458] = -inf;
		ubg[458] = +inf;
		lbg[459] = -inf;
		ubg[459] = +inf;
		lbg[460] = -inf;
		ubg[460] = +inf;
		lbg[461] = -inf;
		ubg[461] = +inf;
		lbg[462] = -inf;
		ubg[462] = +inf;
		lbg[463] = -inf;
		ubg[463] = +inf;
		lbg[464] = -inf;
		ubg[464] = +inf;
		lbg[465] = -inf;
		ubg[465] = +inf;
		lbg[466] = -inf;
		ubg[466] = +inf;
		lbg[467] = -inf;
		ubg[467] = +inf;
		lbg[468] = -inf;
		ubg[468] = +inf;
		lbg[469] = -inf;
		ubg[469] = +inf;
		lbg[470] = -inf;
		ubg[470] = +inf;
		lbg[471] = -inf;
		ubg[471] = +inf;
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
	}else{
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
		lbg[406] = -inf;
		ubg[406] = 0;
		lbg[407] = -inf;
		ubg[407] = 0;
		lbg[408] = -inf;
		ubg[408] = 0;
		lbg[409] = -inf;
		ubg[409] = 0;
		lbg[410] = -inf;
		ubg[410] = 0;
		lbg[411] = -inf;
		ubg[411] = 0;
		lbg[412] = -inf;
		ubg[412] = 0;
		lbg[413] = -inf;
		ubg[413] = 0;
		lbg[414] = -inf;
		ubg[414] = 0;
		lbg[415] = -inf;
		ubg[415] = 0;
		lbg[416] = -inf;
		ubg[416] = 0;
		lbg[417] = -inf;
		ubg[417] = 0;
		lbg[418] = -inf;
		ubg[418] = 0;
		lbg[419] = -inf;
		ubg[419] = 0;
		lbg[420] = -inf;
		ubg[420] = 0;
		lbg[421] = -inf;
		ubg[421] = 0;
		lbg[422] = -inf;
		ubg[422] = 0;
		lbg[423] = -inf;
		ubg[423] = 0;
		lbg[424] = -inf;
		ubg[424] = 0;
		lbg[425] = -inf;
		ubg[425] = 0;
		lbg[426] = -inf;
		ubg[426] = 0;
		lbg[427] = -inf;
		ubg[427] = 0;
		lbg[428] = -inf;
		ubg[428] = 0;
		lbg[429] = -inf;
		ubg[429] = 0;
		lbg[430] = -inf;
		ubg[430] = 0;
		lbg[431] = -inf;
		ubg[431] = 0;
		lbg[432] = -inf;
		ubg[432] = 0;
		lbg[433] = -inf;
		ubg[433] = 0;
		lbg[434] = -inf;
		ubg[434] = 0;
		lbg[435] = -inf;
		ubg[435] = 0;
		lbg[436] = -inf;
		ubg[436] = 0;
		lbg[437] = -inf;
		ubg[437] = 0;
		lbg[438] = -inf;
		ubg[438] = 0;
		lbg[439] = -inf;
		ubg[439] = 0;
		lbg[440] = -inf;
		ubg[440] = 0;
		lbg[441] = -inf;
		ubg[441] = 0;
		lbg[442] = -inf;
		ubg[442] = 0;
		lbg[443] = -inf;
		ubg[443] = 0;
		lbg[444] = -inf;
		ubg[444] = 0;
		lbg[445] = -inf;
		ubg[445] = 0;
		lbg[446] = -inf;
		ubg[446] = 0;
		lbg[447] = -inf;
		ubg[447] = 0;
		lbg[448] = -inf;
		ubg[448] = 0;
		lbg[449] = -inf;
		ubg[449] = 0;
		lbg[450] = -inf;
		ubg[450] = 0;
		lbg[451] = -inf;
		ubg[451] = 0;
		lbg[452] = -inf;
		ubg[452] = 0;
		lbg[453] = -inf;
		ubg[453] = 0;
		lbg[454] = -inf;
		ubg[454] = 0;
		lbg[455] = -inf;
		ubg[455] = 0;
		lbg[456] = -inf;
		ubg[456] = 0;
		lbg[457] = -inf;
		ubg[457] = 0;
		lbg[458] = -inf;
		ubg[458] = 0;
		lbg[459] = -inf;
		ubg[459] = 0;
		lbg[460] = -inf;
		ubg[460] = 0;
		lbg[461] = -inf;
		ubg[461] = 0;
		lbg[462] = -inf;
		ubg[462] = 0;
		lbg[463] = -inf;
		ubg[463] = 0;
		lbg[464] = -inf;
		ubg[464] = 0;
		lbg[465] = -inf;
		ubg[465] = 0;
		lbg[466] = -inf;
		ubg[466] = 0;
		lbg[467] = -inf;
		ubg[467] = 0;
		lbg[468] = -inf;
		ubg[468] = 0;
		lbg[469] = -inf;
		ubg[469] = 0;
		lbg[470] = -inf;
		ubg[470] = 0;
		lbg[471] = -inf;
		ubg[471] = 0;
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
					spline_tf[i] += splines_tf["splines_seg0"][i][j]*variables[0+k*13+j];
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
					spline_tf[i] += splines_tf["a_vehicle0_seg0_00"][i][j]*variables[52+k*11+j];
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
					spline_tf[i] += splines_tf["b_vehicle0_seg0_00"][i][j]*variables[63+k*11+j];
				}
			}
			for(int i=0; i<11; i++){
				variables[63+k*11+i] = spline_tf[i];
			}
		}
	}

}


}
