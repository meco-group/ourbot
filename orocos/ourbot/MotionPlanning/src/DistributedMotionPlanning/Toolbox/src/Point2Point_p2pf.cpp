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

namespace omgf{

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
	substitutes["fleet_center1"] = external("fleet_center1", obj_path+"/subst_fleet_center1.so");
	substitutes["fleet_center0"] = external("fleet_center0", obj_path+"/subst_fleet_center0.so");

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
    std::vector<double> pos0(2), vel0(2), acc0(2);
    std::vector<double> posT(2), velT(2), accT(2);
    for (int k=0; k<n_obs; k++){
        pos0 = obstacles[k].position;
        vel0 = obstacles[k].velocity;
        acc0 = obstacles[k].acceleration;
        // prediction over update_time
        for (int j=0; j<2; j++){
            posT[j] = pos0[j] + update_time*vel0[j] + 0.5*pow(update_time,2)*acc0[j];
            velT[j] = vel0[j] + update_time*acc0[j];
            accT[j] = acc0[j];
        }
        par_dict[obstacle_lbls[k]]["x"] = posT;
        par_dict[obstacle_lbls[k]]["v"] = velT;
        par_dict[obstacle_lbls[k]]["a"] = accT;
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
		par_vect[0+i] = par_dict["vehicle0"]["rel_pos_c"][i];
	}
	for (int i=0; i<2; i++){
		par_vect[2+i] = par_dict["vehicle0"]["state0"][i];
	}
	for (int i=0; i<2; i++){
		par_vect[4+i] = par_dict["vehicle0"]["input0"][i];
	}
	for (int i=0; i<2; i++){
		par_vect[6+i] = par_dict["vehicle0"]["positionT"][i];
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
		par_vect[167+i] = par_dict["obstacle2"]["x"][i];
	}
	for (int i=0; i<2; i++){
		par_vect[169+i] = par_dict["obstacle2"]["v"][i];
	}
	for (int i=0; i<2; i++){
		par_vect[171+i] = par_dict["obstacle2"]["a"][i];
	}
	for (int i=0; i<8; i++){
		par_vect[173+i] = par_dict["obstacle2"]["checkpoints"][i];
	}
	for (int i=0; i<4; i++){
		par_vect[181+i] = par_dict["obstacle2"]["rad"][i];
	}
	for (int i=0; i<2; i++){
		par_vect[185+i] = par_dict["obstacle3"]["x"][i];
	}
	for (int i=0; i<2; i++){
		par_vect[187+i] = par_dict["obstacle3"]["v"][i];
	}
	for (int i=0; i<2; i++){
		par_vect[189+i] = par_dict["obstacle3"]["a"][i];
	}
	for (int i=0; i<8; i++){
		par_vect[191+i] = par_dict["obstacle3"]["checkpoints"][i];
	}
	for (int i=0; i<4; i++){
		par_vect[199+i] = par_dict["obstacle3"]["rad"][i];
	}

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
	if (var_dict.find("environment1") != var_dict.end()){
		if (var_dict["environment1"].find("a_vehicle0_00") != var_dict["environment1"].end()){
			for (int i=0; i<22; i++){
				var_vect[78+i] = var_dict["environment1"]["a_vehicle0_00"][i];
			}
		}
		if (var_dict["environment1"].find("b_vehicle0_00") != var_dict["environment1"].end()){
			for (int i=0; i<11; i++){
				var_vect[100+i] = var_dict["environment1"]["b_vehicle0_00"][i];
			}
		}
		if (var_dict["environment1"].find("a_vehicle0_01") != var_dict["environment1"].end()){
			for (int i=0; i<22; i++){
				var_vect[111+i] = var_dict["environment1"]["a_vehicle0_01"][i];
			}
		}
		if (var_dict["environment1"].find("b_vehicle0_01") != var_dict["environment1"].end()){
			for (int i=0; i<11; i++){
				var_vect[133+i] = var_dict["environment1"]["b_vehicle0_01"][i];
			}
		}
	}
	if (var_dict.find("admm0") != var_dict.end()){
	}
	if (var_dict.find("obstacle2") != var_dict.end()){
	}
	if (var_dict.find("obstacle3") != var_dict.end()){
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
	var_dict["environment1"]["a_vehicle0_00"] = vec;
	vec.resize(11);
	for (int i=0; i<11; i++){
		vec[i] = var_vect[100+i];
	}
	var_dict["environment1"]["b_vehicle0_00"] = vec;
	vec.resize(22);
	for (int i=0; i<22; i++){
		vec[i] = var_vect[111+i];
	}
	var_dict["environment1"]["a_vehicle0_01"] = vec;
	vec.resize(11);
	for (int i=0; i<11; i++){
		vec[i] = var_vect[133+i];
	}
	var_dict["environment1"]["b_vehicle0_01"] = vec;

}

void Point2Point::updateBounds(double current_time, vector<obstacle_t>& obstacles){
	if(!obstacles[0].avoid){
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
	}else{
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
	}
	if(!obstacles[1].avoid){
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
		lbg[928] = -inf;
		ubg[928] = +inf;
		lbg[929] = -inf;
		ubg[929] = +inf;
		lbg[930] = -inf;
		ubg[930] = +inf;
		lbg[931] = -inf;
		ubg[931] = +inf;
		lbg[932] = -inf;
		ubg[932] = +inf;
		lbg[933] = -inf;
		ubg[933] = +inf;
		lbg[934] = -inf;
		ubg[934] = +inf;
		lbg[935] = -inf;
		ubg[935] = +inf;
		lbg[936] = -inf;
		ubg[936] = +inf;
		lbg[937] = -inf;
		ubg[937] = +inf;
		lbg[938] = -inf;
		ubg[938] = +inf;
		lbg[939] = -inf;
		ubg[939] = +inf;
		lbg[940] = -inf;
		ubg[940] = +inf;
		lbg[941] = -inf;
		ubg[941] = +inf;
		lbg[942] = -inf;
		ubg[942] = +inf;
		lbg[943] = -inf;
		ubg[943] = +inf;
		lbg[944] = -inf;
		ubg[944] = +inf;
		lbg[945] = -inf;
		ubg[945] = +inf;
		lbg[946] = -inf;
		ubg[946] = +inf;
		lbg[947] = -inf;
		ubg[947] = +inf;
		lbg[948] = -inf;
		ubg[948] = +inf;
		lbg[949] = -inf;
		ubg[949] = +inf;
		lbg[950] = -inf;
		ubg[950] = +inf;
		lbg[951] = -inf;
		ubg[951] = +inf;
		lbg[952] = -inf;
		ubg[952] = +inf;
		lbg[953] = -inf;
		ubg[953] = +inf;
		lbg[954] = -inf;
		ubg[954] = +inf;
		lbg[955] = -inf;
		ubg[955] = +inf;
		lbg[956] = -inf;
		ubg[956] = +inf;
		lbg[957] = -inf;
		ubg[957] = +inf;
		lbg[958] = -inf;
		ubg[958] = +inf;
		lbg[959] = -inf;
		ubg[959] = +inf;
		lbg[960] = -inf;
		ubg[960] = +inf;
		lbg[961] = -inf;
		ubg[961] = +inf;
		lbg[962] = -inf;
		ubg[962] = +inf;
		lbg[963] = -inf;
		ubg[963] = +inf;
		lbg[964] = -inf;
		ubg[964] = +inf;
		lbg[965] = -inf;
		ubg[965] = +inf;
		lbg[966] = -inf;
		ubg[966] = +inf;
		lbg[967] = -inf;
		ubg[967] = +inf;
		lbg[968] = -inf;
		ubg[968] = +inf;
		lbg[969] = -inf;
		ubg[969] = +inf;
		lbg[970] = -inf;
		ubg[970] = +inf;
		lbg[971] = -inf;
		ubg[971] = +inf;
		lbg[972] = -inf;
		ubg[972] = +inf;
		lbg[973] = -inf;
		ubg[973] = +inf;
		lbg[974] = -inf;
		ubg[974] = +inf;
		lbg[975] = -inf;
		ubg[975] = +inf;
		lbg[976] = -inf;
		ubg[976] = +inf;
		lbg[977] = -inf;
		ubg[977] = +inf;
		lbg[978] = -inf;
		ubg[978] = +inf;
		lbg[979] = -inf;
		ubg[979] = +inf;
		lbg[980] = -inf;
		ubg[980] = +inf;
		lbg[981] = -inf;
		ubg[981] = +inf;
		lbg[982] = -inf;
		ubg[982] = +inf;
		lbg[983] = -inf;
		ubg[983] = +inf;
		lbg[984] = -inf;
		ubg[984] = +inf;
		lbg[985] = -inf;
		ubg[985] = +inf;
		lbg[986] = -inf;
		ubg[986] = +inf;
		lbg[987] = -inf;
		ubg[987] = +inf;
		lbg[988] = -inf;
		ubg[988] = +inf;
		lbg[989] = -inf;
		ubg[989] = +inf;
		lbg[990] = -inf;
		ubg[990] = +inf;
		lbg[991] = -inf;
		ubg[991] = +inf;
		lbg[992] = -inf;
		ubg[992] = +inf;
		lbg[993] = -inf;
		ubg[993] = +inf;
		lbg[994] = -inf;
		ubg[994] = +inf;
		lbg[995] = -inf;
		ubg[995] = +inf;
		lbg[996] = -inf;
		ubg[996] = +inf;
		lbg[997] = -inf;
		ubg[997] = +inf;
		lbg[998] = -inf;
		ubg[998] = +inf;
		lbg[999] = -inf;
		ubg[999] = +inf;
		lbg[1000] = -inf;
		ubg[1000] = +inf;
		lbg[1001] = -inf;
		ubg[1001] = +inf;
		lbg[1002] = -inf;
		ubg[1002] = +inf;
		lbg[1003] = -inf;
		ubg[1003] = +inf;
		lbg[1004] = -inf;
		ubg[1004] = +inf;
		lbg[1005] = -inf;
		ubg[1005] = +inf;
		lbg[1006] = -inf;
		ubg[1006] = +inf;
		lbg[1007] = -inf;
		ubg[1007] = +inf;
		lbg[1008] = -inf;
		ubg[1008] = +inf;
		lbg[1009] = -inf;
		ubg[1009] = +inf;
		lbg[1010] = -inf;
		ubg[1010] = +inf;
		lbg[1011] = -inf;
		ubg[1011] = +inf;
		lbg[1012] = -inf;
		ubg[1012] = +inf;
		lbg[1013] = -inf;
		ubg[1013] = +inf;
		lbg[1014] = -inf;
		ubg[1014] = +inf;
		lbg[1015] = -inf;
		ubg[1015] = +inf;
		lbg[1016] = -inf;
		ubg[1016] = +inf;
		lbg[1017] = -inf;
		ubg[1017] = +inf;
		lbg[1018] = -inf;
		ubg[1018] = +inf;
		lbg[1019] = -inf;
		ubg[1019] = +inf;
		lbg[1020] = -inf;
		ubg[1020] = +inf;
		lbg[1021] = -inf;
		ubg[1021] = +inf;
		lbg[1022] = -inf;
		ubg[1022] = +inf;
		lbg[1023] = -inf;
		ubg[1023] = +inf;
		lbg[1024] = -inf;
		ubg[1024] = +inf;
		lbg[1025] = -inf;
		ubg[1025] = +inf;
		lbg[1026] = -inf;
		ubg[1026] = +inf;
		lbg[1027] = -inf;
		ubg[1027] = +inf;
		lbg[1028] = -inf;
		ubg[1028] = +inf;
		lbg[1029] = -inf;
		ubg[1029] = +inf;
		lbg[1030] = -inf;
		ubg[1030] = +inf;
		lbg[1031] = -inf;
		ubg[1031] = +inf;
	}else{
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
		lbg[928] = -inf;
		ubg[928] = 0;
		lbg[929] = -inf;
		ubg[929] = 0;
		lbg[930] = -inf;
		ubg[930] = 0;
		lbg[931] = -inf;
		ubg[931] = 0;
		lbg[932] = -inf;
		ubg[932] = 0;
		lbg[933] = -inf;
		ubg[933] = 0;
		lbg[934] = -inf;
		ubg[934] = 0;
		lbg[935] = -inf;
		ubg[935] = 0;
		lbg[936] = -inf;
		ubg[936] = 0;
		lbg[937] = -inf;
		ubg[937] = 0;
		lbg[938] = -inf;
		ubg[938] = 0;
		lbg[939] = -inf;
		ubg[939] = 0;
		lbg[940] = -inf;
		ubg[940] = 0;
		lbg[941] = -inf;
		ubg[941] = 0;
		lbg[942] = -inf;
		ubg[942] = 0;
		lbg[943] = -inf;
		ubg[943] = 0;
		lbg[944] = -inf;
		ubg[944] = 0;
		lbg[945] = -inf;
		ubg[945] = 0;
		lbg[946] = -inf;
		ubg[946] = 0;
		lbg[947] = -inf;
		ubg[947] = 0;
		lbg[948] = -inf;
		ubg[948] = 0;
		lbg[949] = -inf;
		ubg[949] = 0;
		lbg[950] = -inf;
		ubg[950] = 0;
		lbg[951] = -inf;
		ubg[951] = 0;
		lbg[952] = -inf;
		ubg[952] = 0;
		lbg[953] = -inf;
		ubg[953] = 0;
		lbg[954] = -inf;
		ubg[954] = 0;
		lbg[955] = -inf;
		ubg[955] = 0;
		lbg[956] = -inf;
		ubg[956] = 0;
		lbg[957] = -inf;
		ubg[957] = 0;
		lbg[958] = -inf;
		ubg[958] = 0;
		lbg[959] = -inf;
		ubg[959] = 0;
		lbg[960] = -inf;
		ubg[960] = 0;
		lbg[961] = -inf;
		ubg[961] = 0;
		lbg[962] = -inf;
		ubg[962] = 0;
		lbg[963] = -inf;
		ubg[963] = 0;
		lbg[964] = -inf;
		ubg[964] = 0;
		lbg[965] = -inf;
		ubg[965] = 0;
		lbg[966] = -inf;
		ubg[966] = 0;
		lbg[967] = -inf;
		ubg[967] = 0;
		lbg[968] = -inf;
		ubg[968] = 0;
		lbg[969] = -inf;
		ubg[969] = 0;
		lbg[970] = -inf;
		ubg[970] = 0;
		lbg[971] = -inf;
		ubg[971] = 0;
		lbg[972] = -inf;
		ubg[972] = 0;
		lbg[973] = -inf;
		ubg[973] = 0;
		lbg[974] = -inf;
		ubg[974] = 0;
		lbg[975] = -inf;
		ubg[975] = 0;
		lbg[976] = -inf;
		ubg[976] = 0;
		lbg[977] = -inf;
		ubg[977] = 0;
		lbg[978] = -inf;
		ubg[978] = 0;
		lbg[979] = -inf;
		ubg[979] = 0;
		lbg[980] = -inf;
		ubg[980] = 0;
		lbg[981] = -inf;
		ubg[981] = 0;
		lbg[982] = -inf;
		ubg[982] = 0;
		lbg[983] = -inf;
		ubg[983] = 0;
		lbg[984] = -inf;
		ubg[984] = 0;
		lbg[985] = -inf;
		ubg[985] = 0;
		lbg[986] = -inf;
		ubg[986] = 0;
		lbg[987] = -inf;
		ubg[987] = 0;
		lbg[988] = -inf;
		ubg[988] = 0;
		lbg[989] = -inf;
		ubg[989] = 0;
		lbg[990] = -inf;
		ubg[990] = 0;
		lbg[991] = -inf;
		ubg[991] = 0;
		lbg[992] = -inf;
		ubg[992] = 0;
		lbg[993] = -inf;
		ubg[993] = 0;
		lbg[994] = -inf;
		ubg[994] = 0;
		lbg[995] = -inf;
		ubg[995] = 0;
		lbg[996] = -inf;
		ubg[996] = 0;
		lbg[997] = -inf;
		ubg[997] = 0;
		lbg[998] = -inf;
		ubg[998] = 0;
		lbg[999] = -inf;
		ubg[999] = 0;
		lbg[1000] = -inf;
		ubg[1000] = 0;
		lbg[1001] = -inf;
		ubg[1001] = 0;
		lbg[1002] = -inf;
		ubg[1002] = 0;
		lbg[1003] = -inf;
		ubg[1003] = 0;
		lbg[1004] = -inf;
		ubg[1004] = 0;
		lbg[1005] = -inf;
		ubg[1005] = 0;
		lbg[1006] = -inf;
		ubg[1006] = 0;
		lbg[1007] = -inf;
		ubg[1007] = 0;
		lbg[1008] = -inf;
		ubg[1008] = 0;
		lbg[1009] = -inf;
		ubg[1009] = 0;
		lbg[1010] = -inf;
		ubg[1010] = 0;
		lbg[1011] = -inf;
		ubg[1011] = 0;
		lbg[1012] = -inf;
		ubg[1012] = 0;
		lbg[1013] = -inf;
		ubg[1013] = 0;
		lbg[1014] = -inf;
		ubg[1014] = 0;
		lbg[1015] = -inf;
		ubg[1015] = 0;
		lbg[1016] = -inf;
		ubg[1016] = 0;
		lbg[1017] = -inf;
		ubg[1017] = 0;
		lbg[1018] = -inf;
		ubg[1018] = 0;
		lbg[1019] = -inf;
		ubg[1019] = 0;
		lbg[1020] = -inf;
		ubg[1020] = 0;
		lbg[1021] = -inf;
		ubg[1021] = 0;
		lbg[1022] = -inf;
		ubg[1022] = 0;
		lbg[1023] = -inf;
		ubg[1023] = 0;
		lbg[1024] = -inf;
		ubg[1024] = 0;
		lbg[1025] = -inf;
		ubg[1025] = 0;
		lbg[1026] = -inf;
		ubg[1026] = 0;
		lbg[1027] = -inf;
		ubg[1027] = 0;
		lbg[1028] = -inf;
		ubg[1028] = 0;
		lbg[1029] = -inf;
		ubg[1029] = 0;
		lbg[1030] = -inf;
		ubg[1030] = 0;
		lbg[1031] = -inf;
		ubg[1031] = 0;
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
