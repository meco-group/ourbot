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

namespace omg{

Point2Point::Point2Point(Vehicle* vehicle,
    double update_time, double sample_time, double horizon_time, int trajectory_length, bool initialize):
parameters(N_PAR), variables(N_VAR), lbg(LBG_DEF), ubg(UBG_DEF) {
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
    #ifdef DEBUG
    dumpData();
    #endif
    // update current time
    current_time_prev = current_time;
    current_time += update_time;
    return true;
}

bool Point2Point::solve(double current_time, vector<obstacle_t>& obstacles){
    // init variables if first time
    if(fabs(current_time)<=1.e-6){
        initVariables();
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
    string obstacle_lbls [N_OBS] = OBSTACLELBLS;
    for (int k=0; k<n_obs; k++){
        par_dict[obstacle_lbls[k]]["x"] = obstacles[k].position;
        par_dict[obstacle_lbls[k]]["v"] = obstacles[k].velocity;
        par_dict[obstacle_lbls[k]]["a"] = obstacles[k].acceleration;
        par_dict[obstacle_lbls[k]]["checkpoints"] = obstacles[k].checkpoints;
        par_dict[obstacle_lbls[k]]["rad"] = obstacles[k].radii;
    }
}

void Point2Point::extractData(){
    map<string, map<string, vector<double>>> var_dict;
    getVariableDict(variables, var_dict);
    vector<double> spline_coeffs_vec(var_dict[VEHICLELBL]["splines0"]);
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
		par_vect[4+i] = par_dict["vehicle0"]["positionT"][i];
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
		lbg[720] = -inf;
		ubg[720] = +inf;
		lbg[721] = -inf;
		ubg[721] = +inf;
		lbg[722] = -inf;
		ubg[722] = +inf;
		lbg[723] = -inf;
		ubg[723] = +inf;
		lbg[724] = -inf;
		ubg[724] = +inf;
		lbg[725] = -inf;
		ubg[725] = +inf;
		lbg[726] = -inf;
		ubg[726] = +inf;
		lbg[727] = -inf;
		ubg[727] = +inf;
		lbg[728] = -inf;
		ubg[728] = +inf;
		lbg[729] = -inf;
		ubg[729] = +inf;
		lbg[730] = -inf;
		ubg[730] = +inf;
		lbg[731] = -inf;
		ubg[731] = +inf;
		lbg[732] = -inf;
		ubg[732] = +inf;
		lbg[733] = -inf;
		ubg[733] = +inf;
		lbg[734] = -inf;
		ubg[734] = +inf;
		lbg[735] = -inf;
		ubg[735] = +inf;
		lbg[736] = -inf;
		ubg[736] = +inf;
		lbg[737] = -inf;
		ubg[737] = +inf;
		lbg[738] = -inf;
		ubg[738] = +inf;
		lbg[739] = -inf;
		ubg[739] = +inf;
		lbg[740] = -inf;
		ubg[740] = +inf;
		lbg[741] = -inf;
		ubg[741] = +inf;
		lbg[742] = -inf;
		ubg[742] = +inf;
		lbg[743] = -inf;
		ubg[743] = +inf;
		lbg[744] = -inf;
		ubg[744] = +inf;
		lbg[745] = -inf;
		ubg[745] = +inf;
		lbg[746] = -inf;
		ubg[746] = +inf;
		lbg[747] = -inf;
		ubg[747] = +inf;
		lbg[748] = -inf;
		ubg[748] = +inf;
		lbg[749] = -inf;
		ubg[749] = +inf;
		lbg[750] = -inf;
		ubg[750] = +inf;
		lbg[751] = -inf;
		ubg[751] = +inf;
		lbg[752] = -inf;
		ubg[752] = +inf;
		lbg[753] = -inf;
		ubg[753] = +inf;
		lbg[754] = -inf;
		ubg[754] = +inf;
		lbg[755] = -inf;
		ubg[755] = +inf;
		lbg[756] = -inf;
		ubg[756] = +inf;
		lbg[757] = -inf;
		ubg[757] = +inf;
		lbg[758] = -inf;
		ubg[758] = +inf;
		lbg[759] = -inf;
		ubg[759] = +inf;
		lbg[760] = -inf;
		ubg[760] = +inf;
		lbg[761] = -inf;
		ubg[761] = +inf;
		lbg[762] = -inf;
		ubg[762] = +inf;
		lbg[763] = -inf;
		ubg[763] = +inf;
		lbg[764] = -inf;
		ubg[764] = +inf;
		lbg[765] = -inf;
		ubg[765] = +inf;
		lbg[766] = -inf;
		ubg[766] = +inf;
		lbg[767] = -inf;
		ubg[767] = +inf;
		lbg[768] = -inf;
		ubg[768] = +inf;
		lbg[769] = -inf;
		ubg[769] = +inf;
		lbg[770] = -inf;
		ubg[770] = +inf;
		lbg[771] = -inf;
		ubg[771] = +inf;
		lbg[772] = -inf;
		ubg[772] = +inf;
		lbg[773] = -inf;
		ubg[773] = +inf;
		lbg[774] = -inf;
		ubg[774] = +inf;
		lbg[775] = -inf;
		ubg[775] = +inf;
		lbg[776] = -inf;
		ubg[776] = +inf;
		lbg[777] = -inf;
		ubg[777] = +inf;
		lbg[778] = -inf;
		ubg[778] = +inf;
		lbg[779] = -inf;
		ubg[779] = +inf;
		lbg[780] = -inf;
		ubg[780] = +inf;
		lbg[781] = -inf;
		ubg[781] = +inf;
		lbg[782] = -inf;
		ubg[782] = +inf;
		lbg[783] = -inf;
		ubg[783] = +inf;
		lbg[784] = -inf;
		ubg[784] = +inf;
		lbg[785] = -inf;
		ubg[785] = +inf;
		lbg[786] = -inf;
		ubg[786] = +inf;
		lbg[787] = -inf;
		ubg[787] = +inf;
		lbg[788] = -inf;
		ubg[788] = +inf;
		lbg[789] = -inf;
		ubg[789] = +inf;
		lbg[790] = -inf;
		ubg[790] = +inf;
		lbg[791] = -inf;
		ubg[791] = +inf;
		lbg[792] = -inf;
		ubg[792] = +inf;
		lbg[793] = -inf;
		ubg[793] = +inf;
		lbg[794] = -inf;
		ubg[794] = +inf;
		lbg[795] = -inf;
		ubg[795] = +inf;
		lbg[796] = -inf;
		ubg[796] = +inf;
		lbg[797] = -inf;
		ubg[797] = +inf;
		lbg[798] = -inf;
		ubg[798] = +inf;
		lbg[799] = -inf;
		ubg[799] = +inf;
		lbg[800] = -inf;
		ubg[800] = +inf;
		lbg[801] = -inf;
		ubg[801] = +inf;
		lbg[802] = -inf;
		ubg[802] = +inf;
		lbg[803] = -inf;
		ubg[803] = +inf;
	}else{
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
		lbg[720] = -inf;
		ubg[720] = 0;
		lbg[721] = -inf;
		ubg[721] = 0;
		lbg[722] = -inf;
		ubg[722] = 0;
		lbg[723] = -inf;
		ubg[723] = 0;
		lbg[724] = -inf;
		ubg[724] = 0;
		lbg[725] = -inf;
		ubg[725] = 0;
		lbg[726] = -inf;
		ubg[726] = 0;
		lbg[727] = -inf;
		ubg[727] = 0;
		lbg[728] = -inf;
		ubg[728] = 0;
		lbg[729] = -inf;
		ubg[729] = 0;
		lbg[730] = -inf;
		ubg[730] = 0;
		lbg[731] = -inf;
		ubg[731] = 0;
		lbg[732] = -inf;
		ubg[732] = 0;
		lbg[733] = -inf;
		ubg[733] = 0;
		lbg[734] = -inf;
		ubg[734] = 0;
		lbg[735] = -inf;
		ubg[735] = 0;
		lbg[736] = -inf;
		ubg[736] = 0;
		lbg[737] = -inf;
		ubg[737] = 0;
		lbg[738] = -inf;
		ubg[738] = 0;
		lbg[739] = -inf;
		ubg[739] = 0;
		lbg[740] = -inf;
		ubg[740] = 0;
		lbg[741] = -inf;
		ubg[741] = 0;
		lbg[742] = -inf;
		ubg[742] = 0;
		lbg[743] = -inf;
		ubg[743] = 0;
		lbg[744] = -inf;
		ubg[744] = 0;
		lbg[745] = -inf;
		ubg[745] = 0;
		lbg[746] = -inf;
		ubg[746] = 0;
		lbg[747] = -inf;
		ubg[747] = 0;
		lbg[748] = -inf;
		ubg[748] = 0;
		lbg[749] = -inf;
		ubg[749] = 0;
		lbg[750] = -inf;
		ubg[750] = 0;
		lbg[751] = -inf;
		ubg[751] = 0;
		lbg[752] = -inf;
		ubg[752] = 0;
		lbg[753] = -inf;
		ubg[753] = 0;
		lbg[754] = -inf;
		ubg[754] = 0;
		lbg[755] = -inf;
		ubg[755] = 0;
		lbg[756] = -inf;
		ubg[756] = 0;
		lbg[757] = -inf;
		ubg[757] = 0;
		lbg[758] = -inf;
		ubg[758] = 0;
		lbg[759] = -inf;
		ubg[759] = 0;
		lbg[760] = -inf;
		ubg[760] = 0;
		lbg[761] = -inf;
		ubg[761] = 0;
		lbg[762] = -inf;
		ubg[762] = 0;
		lbg[763] = -inf;
		ubg[763] = 0;
		lbg[764] = -inf;
		ubg[764] = 0;
		lbg[765] = -inf;
		ubg[765] = 0;
		lbg[766] = -inf;
		ubg[766] = 0;
		lbg[767] = -inf;
		ubg[767] = 0;
		lbg[768] = -inf;
		ubg[768] = 0;
		lbg[769] = -inf;
		ubg[769] = 0;
		lbg[770] = -inf;
		ubg[770] = 0;
		lbg[771] = -inf;
		ubg[771] = 0;
		lbg[772] = -inf;
		ubg[772] = 0;
		lbg[773] = -inf;
		ubg[773] = 0;
		lbg[774] = -inf;
		ubg[774] = 0;
		lbg[775] = -inf;
		ubg[775] = 0;
		lbg[776] = -inf;
		ubg[776] = 0;
		lbg[777] = -inf;
		ubg[777] = 0;
		lbg[778] = -inf;
		ubg[778] = 0;
		lbg[779] = -inf;
		ubg[779] = 0;
		lbg[780] = -inf;
		ubg[780] = 0;
		lbg[781] = -inf;
		ubg[781] = 0;
		lbg[782] = -inf;
		ubg[782] = 0;
		lbg[783] = -inf;
		ubg[783] = 0;
		lbg[784] = -inf;
		ubg[784] = 0;
		lbg[785] = -inf;
		ubg[785] = 0;
		lbg[786] = -inf;
		ubg[786] = 0;
		lbg[787] = -inf;
		ubg[787] = 0;
		lbg[788] = -inf;
		ubg[788] = 0;
		lbg[789] = -inf;
		ubg[789] = 0;
		lbg[790] = -inf;
		ubg[790] = 0;
		lbg[791] = -inf;
		ubg[791] = 0;
		lbg[792] = -inf;
		ubg[792] = 0;
		lbg[793] = -inf;
		ubg[793] = 0;
		lbg[794] = -inf;
		ubg[794] = 0;
		lbg[795] = -inf;
		ubg[795] = 0;
		lbg[796] = -inf;
		ubg[796] = 0;
		lbg[797] = -inf;
		ubg[797] = 0;
		lbg[798] = -inf;
		ubg[798] = 0;
		lbg[799] = -inf;
		ubg[799] = 0;
		lbg[800] = -inf;
		ubg[800] = 0;
		lbg[801] = -inf;
		ubg[801] = 0;
		lbg[802] = -inf;
		ubg[802] = 0;
		lbg[803] = -inf;
		ubg[803] = 0;
	}
	if(!obstacles[1].avoid){
		lbg[804] = -inf;
		ubg[804] = +inf;
		lbg[805] = -inf;
		ubg[805] = +inf;
		lbg[806] = -inf;
		ubg[806] = +inf;
		lbg[807] = -inf;
		ubg[807] = +inf;
		lbg[808] = -inf;
		ubg[808] = +inf;
		lbg[809] = -inf;
		ubg[809] = +inf;
		lbg[810] = -inf;
		ubg[810] = +inf;
		lbg[811] = -inf;
		ubg[811] = +inf;
		lbg[812] = -inf;
		ubg[812] = +inf;
		lbg[813] = -inf;
		ubg[813] = +inf;
		lbg[814] = -inf;
		ubg[814] = +inf;
		lbg[815] = -inf;
		ubg[815] = +inf;
		lbg[816] = -inf;
		ubg[816] = +inf;
		lbg[817] = -inf;
		ubg[817] = +inf;
		lbg[818] = -inf;
		ubg[818] = +inf;
		lbg[819] = -inf;
		ubg[819] = +inf;
		lbg[820] = -inf;
		ubg[820] = +inf;
		lbg[821] = -inf;
		ubg[821] = +inf;
		lbg[822] = -inf;
		ubg[822] = +inf;
		lbg[823] = -inf;
		ubg[823] = +inf;
		lbg[824] = -inf;
		ubg[824] = +inf;
		lbg[825] = -inf;
		ubg[825] = +inf;
		lbg[826] = -inf;
		ubg[826] = +inf;
		lbg[827] = -inf;
		ubg[827] = +inf;
		lbg[828] = -inf;
		ubg[828] = +inf;
		lbg[829] = -inf;
		ubg[829] = +inf;
		lbg[830] = -inf;
		ubg[830] = +inf;
		lbg[831] = -inf;
		ubg[831] = +inf;
		lbg[832] = -inf;
		ubg[832] = +inf;
		lbg[833] = -inf;
		ubg[833] = +inf;
		lbg[834] = -inf;
		ubg[834] = +inf;
		lbg[835] = -inf;
		ubg[835] = +inf;
		lbg[836] = -inf;
		ubg[836] = +inf;
		lbg[837] = -inf;
		ubg[837] = +inf;
		lbg[838] = -inf;
		ubg[838] = +inf;
		lbg[839] = -inf;
		ubg[839] = +inf;
		lbg[840] = -inf;
		ubg[840] = +inf;
		lbg[841] = -inf;
		ubg[841] = +inf;
		lbg[842] = -inf;
		ubg[842] = +inf;
		lbg[843] = -inf;
		ubg[843] = +inf;
		lbg[844] = -inf;
		ubg[844] = +inf;
		lbg[845] = -inf;
		ubg[845] = +inf;
		lbg[846] = -inf;
		ubg[846] = +inf;
		lbg[847] = -inf;
		ubg[847] = +inf;
		lbg[848] = -inf;
		ubg[848] = +inf;
		lbg[849] = -inf;
		ubg[849] = +inf;
		lbg[850] = -inf;
		ubg[850] = +inf;
		lbg[851] = -inf;
		ubg[851] = +inf;
		lbg[852] = -inf;
		ubg[852] = +inf;
		lbg[853] = -inf;
		ubg[853] = +inf;
		lbg[854] = -inf;
		ubg[854] = +inf;
		lbg[855] = -inf;
		ubg[855] = +inf;
		lbg[856] = -inf;
		ubg[856] = +inf;
		lbg[857] = -inf;
		ubg[857] = +inf;
		lbg[858] = -inf;
		ubg[858] = +inf;
		lbg[859] = -inf;
		ubg[859] = +inf;
		lbg[860] = -inf;
		ubg[860] = +inf;
		lbg[861] = -inf;
		ubg[861] = +inf;
		lbg[862] = -inf;
		ubg[862] = +inf;
		lbg[863] = -inf;
		ubg[863] = +inf;
		lbg[864] = -inf;
		ubg[864] = +inf;
		lbg[865] = -inf;
		ubg[865] = +inf;
		lbg[866] = -inf;
		ubg[866] = +inf;
		lbg[867] = -inf;
		ubg[867] = +inf;
		lbg[868] = -inf;
		ubg[868] = +inf;
		lbg[869] = -inf;
		ubg[869] = +inf;
		lbg[870] = -inf;
		ubg[870] = +inf;
		lbg[871] = -inf;
		ubg[871] = +inf;
		lbg[872] = -inf;
		ubg[872] = +inf;
		lbg[873] = -inf;
		ubg[873] = +inf;
		lbg[874] = -inf;
		ubg[874] = +inf;
		lbg[875] = -inf;
		ubg[875] = +inf;
		lbg[876] = -inf;
		ubg[876] = +inf;
		lbg[877] = -inf;
		ubg[877] = +inf;
		lbg[878] = -inf;
		ubg[878] = +inf;
		lbg[879] = -inf;
		ubg[879] = +inf;
		lbg[880] = -inf;
		ubg[880] = +inf;
		lbg[881] = -inf;
		ubg[881] = +inf;
		lbg[882] = -inf;
		ubg[882] = +inf;
		lbg[883] = -inf;
		ubg[883] = +inf;
		lbg[884] = -inf;
		ubg[884] = +inf;
		lbg[885] = -inf;
		ubg[885] = +inf;
		lbg[886] = -inf;
		ubg[886] = +inf;
		lbg[887] = -inf;
		ubg[887] = +inf;
		lbg[888] = -inf;
		ubg[888] = +inf;
		lbg[889] = -inf;
		ubg[889] = +inf;
		lbg[890] = -inf;
		ubg[890] = +inf;
		lbg[891] = -inf;
		ubg[891] = +inf;
		lbg[892] = -inf;
		ubg[892] = +inf;
		lbg[893] = -inf;
		ubg[893] = +inf;
		lbg[894] = -inf;
		ubg[894] = +inf;
		lbg[895] = -inf;
		ubg[895] = +inf;
		lbg[896] = -inf;
		ubg[896] = +inf;
		lbg[897] = -inf;
		ubg[897] = +inf;
		lbg[898] = -inf;
		ubg[898] = +inf;
		lbg[899] = -inf;
		ubg[899] = +inf;
		lbg[900] = -inf;
		ubg[900] = +inf;
		lbg[901] = -inf;
		ubg[901] = +inf;
		lbg[902] = -inf;
		ubg[902] = +inf;
		lbg[903] = -inf;
		ubg[903] = +inf;
		lbg[904] = -inf;
		ubg[904] = +inf;
		lbg[905] = -inf;
		ubg[905] = +inf;
		lbg[906] = -inf;
		ubg[906] = +inf;
		lbg[907] = -inf;
		ubg[907] = +inf;
		lbg[908] = -inf;
		ubg[908] = +inf;
		lbg[909] = -inf;
		ubg[909] = +inf;
		lbg[910] = -inf;
		ubg[910] = +inf;
		lbg[911] = -inf;
		ubg[911] = +inf;
		lbg[912] = -inf;
		ubg[912] = +inf;
		lbg[913] = -inf;
		ubg[913] = +inf;
		lbg[914] = -inf;
		ubg[914] = +inf;
		lbg[915] = -inf;
		ubg[915] = +inf;
		lbg[916] = -inf;
		ubg[916] = +inf;
		lbg[917] = -inf;
		ubg[917] = +inf;
		lbg[918] = -inf;
		ubg[918] = +inf;
		lbg[919] = -inf;
		ubg[919] = +inf;
		lbg[920] = -inf;
		ubg[920] = +inf;
		lbg[921] = -inf;
		ubg[921] = +inf;
		lbg[922] = -inf;
		ubg[922] = +inf;
		lbg[923] = -inf;
		ubg[923] = +inf;
		lbg[924] = -inf;
		ubg[924] = +inf;
		lbg[925] = -inf;
		ubg[925] = +inf;
		lbg[926] = -inf;
		ubg[926] = +inf;
		lbg[927] = -inf;
		ubg[927] = +inf;
	}else{
		lbg[804] = -inf;
		ubg[804] = 0;
		lbg[805] = -inf;
		ubg[805] = 0;
		lbg[806] = -inf;
		ubg[806] = 0;
		lbg[807] = -inf;
		ubg[807] = 0;
		lbg[808] = -inf;
		ubg[808] = 0;
		lbg[809] = -inf;
		ubg[809] = 0;
		lbg[810] = -inf;
		ubg[810] = 0;
		lbg[811] = -inf;
		ubg[811] = 0;
		lbg[812] = -inf;
		ubg[812] = 0;
		lbg[813] = -inf;
		ubg[813] = 0;
		lbg[814] = -inf;
		ubg[814] = 0;
		lbg[815] = -inf;
		ubg[815] = 0;
		lbg[816] = -inf;
		ubg[816] = 0;
		lbg[817] = -inf;
		ubg[817] = 0;
		lbg[818] = -inf;
		ubg[818] = 0;
		lbg[819] = -inf;
		ubg[819] = 0;
		lbg[820] = -inf;
		ubg[820] = 0;
		lbg[821] = -inf;
		ubg[821] = 0;
		lbg[822] = -inf;
		ubg[822] = 0;
		lbg[823] = -inf;
		ubg[823] = 0;
		lbg[824] = -inf;
		ubg[824] = 0;
		lbg[825] = -inf;
		ubg[825] = 0;
		lbg[826] = -inf;
		ubg[826] = 0;
		lbg[827] = -inf;
		ubg[827] = 0;
		lbg[828] = -inf;
		ubg[828] = 0;
		lbg[829] = -inf;
		ubg[829] = 0;
		lbg[830] = -inf;
		ubg[830] = 0;
		lbg[831] = -inf;
		ubg[831] = 0;
		lbg[832] = -inf;
		ubg[832] = 0;
		lbg[833] = -inf;
		ubg[833] = 0;
		lbg[834] = -inf;
		ubg[834] = 0;
		lbg[835] = -inf;
		ubg[835] = 0;
		lbg[836] = -inf;
		ubg[836] = 0;
		lbg[837] = -inf;
		ubg[837] = 0;
		lbg[838] = -inf;
		ubg[838] = 0;
		lbg[839] = -inf;
		ubg[839] = 0;
		lbg[840] = -inf;
		ubg[840] = 0;
		lbg[841] = -inf;
		ubg[841] = 0;
		lbg[842] = -inf;
		ubg[842] = 0;
		lbg[843] = -inf;
		ubg[843] = 0;
		lbg[844] = -inf;
		ubg[844] = 0;
		lbg[845] = -inf;
		ubg[845] = 0;
		lbg[846] = -inf;
		ubg[846] = 0;
		lbg[847] = -inf;
		ubg[847] = 0;
		lbg[848] = -inf;
		ubg[848] = 0;
		lbg[849] = -inf;
		ubg[849] = 0;
		lbg[850] = -inf;
		ubg[850] = 0;
		lbg[851] = -inf;
		ubg[851] = 0;
		lbg[852] = -inf;
		ubg[852] = 0;
		lbg[853] = -inf;
		ubg[853] = 0;
		lbg[854] = -inf;
		ubg[854] = 0;
		lbg[855] = -inf;
		ubg[855] = 0;
		lbg[856] = -inf;
		ubg[856] = 0;
		lbg[857] = -inf;
		ubg[857] = 0;
		lbg[858] = -inf;
		ubg[858] = 0;
		lbg[859] = -inf;
		ubg[859] = 0;
		lbg[860] = -inf;
		ubg[860] = 0;
		lbg[861] = -inf;
		ubg[861] = 0;
		lbg[862] = -inf;
		ubg[862] = 0;
		lbg[863] = -inf;
		ubg[863] = 0;
		lbg[864] = -inf;
		ubg[864] = 0;
		lbg[865] = -inf;
		ubg[865] = 0;
		lbg[866] = -inf;
		ubg[866] = 0;
		lbg[867] = -inf;
		ubg[867] = 0;
		lbg[868] = -inf;
		ubg[868] = 0;
		lbg[869] = -inf;
		ubg[869] = 0;
		lbg[870] = -inf;
		ubg[870] = 0;
		lbg[871] = -inf;
		ubg[871] = 0;
		lbg[872] = -inf;
		ubg[872] = 0;
		lbg[873] = -inf;
		ubg[873] = 0;
		lbg[874] = -inf;
		ubg[874] = 0;
		lbg[875] = -inf;
		ubg[875] = 0;
		lbg[876] = -inf;
		ubg[876] = 0;
		lbg[877] = -inf;
		ubg[877] = 0;
		lbg[878] = -inf;
		ubg[878] = 0;
		lbg[879] = -inf;
		ubg[879] = 0;
		lbg[880] = -inf;
		ubg[880] = 0;
		lbg[881] = -inf;
		ubg[881] = 0;
		lbg[882] = -inf;
		ubg[882] = 0;
		lbg[883] = -inf;
		ubg[883] = 0;
		lbg[884] = -inf;
		ubg[884] = 0;
		lbg[885] = -inf;
		ubg[885] = 0;
		lbg[886] = -inf;
		ubg[886] = 0;
		lbg[887] = -inf;
		ubg[887] = 0;
		lbg[888] = -inf;
		ubg[888] = 0;
		lbg[889] = -inf;
		ubg[889] = 0;
		lbg[890] = -inf;
		ubg[890] = 0;
		lbg[891] = -inf;
		ubg[891] = 0;
		lbg[892] = -inf;
		ubg[892] = 0;
		lbg[893] = -inf;
		ubg[893] = 0;
		lbg[894] = -inf;
		ubg[894] = 0;
		lbg[895] = -inf;
		ubg[895] = 0;
		lbg[896] = -inf;
		ubg[896] = 0;
		lbg[897] = -inf;
		ubg[897] = 0;
		lbg[898] = -inf;
		ubg[898] = 0;
		lbg[899] = -inf;
		ubg[899] = 0;
		lbg[900] = -inf;
		ubg[900] = 0;
		lbg[901] = -inf;
		ubg[901] = 0;
		lbg[902] = -inf;
		ubg[902] = 0;
		lbg[903] = -inf;
		ubg[903] = 0;
		lbg[904] = -inf;
		ubg[904] = 0;
		lbg[905] = -inf;
		ubg[905] = 0;
		lbg[906] = -inf;
		ubg[906] = 0;
		lbg[907] = -inf;
		ubg[907] = 0;
		lbg[908] = -inf;
		ubg[908] = 0;
		lbg[909] = -inf;
		ubg[909] = 0;
		lbg[910] = -inf;
		ubg[910] = 0;
		lbg[911] = -inf;
		ubg[911] = 0;
		lbg[912] = -inf;
		ubg[912] = 0;
		lbg[913] = -inf;
		ubg[913] = 0;
		lbg[914] = -inf;
		ubg[914] = 0;
		lbg[915] = -inf;
		ubg[915] = 0;
		lbg[916] = -inf;
		ubg[916] = 0;
		lbg[917] = -inf;
		ubg[917] = 0;
		lbg[918] = -inf;
		ubg[918] = 0;
		lbg[919] = -inf;
		ubg[919] = 0;
		lbg[920] = -inf;
		ubg[920] = 0;
		lbg[921] = -inf;
		ubg[921] = 0;
		lbg[922] = -inf;
		ubg[922] = 0;
		lbg[923] = -inf;
		ubg[923] = 0;
		lbg[924] = -inf;
		ubg[924] = 0;
		lbg[925] = -inf;
		ubg[925] = 0;
		lbg[926] = -inf;
		ubg[926] = 0;
		lbg[927] = -inf;
		ubg[927] = 0;
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
