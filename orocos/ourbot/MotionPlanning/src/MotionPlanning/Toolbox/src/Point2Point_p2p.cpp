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


	par_dict["obstacle1"]["traj_coeffs"] = obstacles[1].traj_coeffs;
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
	for (int i=0; i<2; i++){
		par_vect[12+i] = par_dict["obstacle0"]["checkpoints"][i];
	}
	par_vect[14] = par_dict["obstacle0"]["rad"][0];
	for (int i=0; i<26; i++){
		par_vect[15+i] = par_dict["obstacle1"]["traj_coeffs"][i];
	}
	for (int i=0; i<2; i++){
		par_vect[41+i] = par_dict["obstacle1"]["checkpoints"][i];
	}
	par_vect[43] = par_dict["obstacle1"]["rad"][0];
	par_vect[44] = par_dict["p2p0"]["T"][0];
	par_vect[45] = par_dict["p2p0"]["t"][0];

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
	}
	if(!obstacles[1].avoid){
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
	}else{
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
