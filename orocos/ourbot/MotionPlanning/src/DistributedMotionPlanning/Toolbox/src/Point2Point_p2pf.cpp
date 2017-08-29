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
	splines_tf["splines0"] = SPLINES0_TF;
	splines_tf["eps_00"] = EPS_00_TF;
	splines_tf["g0"] = G0_TF;
	splines_tf["g1"] = G1_TF;
	splines_tf["a_vehicle0_00"] = A_VEHICLE0_00_TF;
	splines_tf["b_vehicle0_00"] = B_VEHICLE0_00_TF;
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
	par_dict["obstacle1"]["x"] = posT;
	par_dict["obstacle1"]["v"] = velT;
	par_dict["obstacle1"]["a"] = accT;
	par_dict["obstacle1"]["checkpoints"] = obstacles[0].checkpoints;
	par_dict["obstacle1"]["rad"] = obstacles[0].radii;


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
		if (var_dict["environment1"].find("a_vehicle0_00") != var_dict["environment1"].end()){
			for (int i=0; i<22; i++){
				var_vect[65+i] = var_dict["environment1"]["a_vehicle0_00"][i];
			}
		}
		if (var_dict["environment1"].find("b_vehicle0_00") != var_dict["environment1"].end()){
			for (int i=0; i<11; i++){
				var_vect[87+i] = var_dict["environment1"]["b_vehicle0_00"][i];
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
	var_dict["environment1"]["a_vehicle0_00"] = vec;
	vec.resize(11);
	for (int i=0; i<11; i++){
		vec[i] = var_vect[87+i];
	}
	var_dict["environment1"]["b_vehicle0_00"] = vec;

}

void Point2Point::updateBounds(double current_time, vector<obstacle_t>& obstacles){
	if(!obstacles[0].avoid){
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
	}else{
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
