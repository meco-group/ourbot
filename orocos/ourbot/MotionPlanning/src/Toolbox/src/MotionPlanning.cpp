#include "MotionPlanning.hpp"
#ifdef DEBUG
#include <ctime>
#endif
#include <unistd.h>

using namespace std;
using namespace casadi;

namespace omg{

MotionPlanning::MotionPlanning(double update_time, double sample_time, double horizon_time):
parameters(N_PAR), variables(N_VAR), lbg(LBG_DEF), ubg(UBG_DEF),
y_coeffs(N_Y, vector<double>(Y_LENGTH)), time(int(update_time/sample_time)+1),
state_trajectory(int(update_time/sample_time)+1, vector<double>(N_ST)),
input_trajectory(int(update_time/sample_time)+1, vector<double>(N_IN)),
y0(N_Y, vector<double>(N_DER+1)), yT(N_Y, vector<double>(N_DER+1)),
ypred(N_Y, vector<double>(N_DER+1)), ideal_update(false){
    this->update_time = update_time;
    this->sample_time = sample_time;
    this->horizon_time = horizon_time;
    for (int k=0; k<time.size(); k++){
        time[k] = k*sample_time;
    }
    generateProblem();
    args["p"] = parameters;
    args["x0"] = variables;
    args["lbg"] = lbg;
    args["ubg"] = ubg;
    initSplines();
}

void MotionPlanning::generateProblem(){
    // change pwd to CASADIOBJ
    char cwd[1024];
    getcwd(cwd, sizeof(cwd));
    chdir(CASADIOBJ);
    // load object files
    ExternalFunction grad_f("grad_f");
    ExternalFunction jac_g("jac_g");
    ExternalFunction hess_lag("hess_lag");
    ExternalFunction nlp("nlp");
    // change back to original pwd
    chdir(cwd);
    // set options
    Dict options;
    options["grad_f"] = grad_f;
    options["jac_g"] = jac_g;
    options["jac_g"] = jac_g;
    options["hess_lag"] = hess_lag;
    options["print_level"] = 0;
    options["print_time"] = 0;
    options["tol"] = TOL;
    options["linear_solver"] = LINEAR_SOLVER;
    options["warm_start_init_point"] = "yes";
    // create nlp solver
    NlpSolver problem("problem", "ipopt", nlp, options);
    this->problem = problem;
}

void MotionPlanning::initSplines(){
	spline_t spline_y = {Y_KNOTS,Y_DEGREE,Y_TF};
	splines["y"] = spline_y;
	spline_t spline_b_vehicle0obstacle1 = {B_VEHICLE0OBSTACLE1_KNOTS,B_VEHICLE0OBSTACLE1_DEGREE,B_VEHICLE0OBSTACLE1_TF};
	splines["b_vehicle0obstacle1"] = spline_b_vehicle0obstacle1;
	spline_t spline_b_vehicle0obstacle0 = {B_VEHICLE0OBSTACLE0_KNOTS,B_VEHICLE0OBSTACLE0_DEGREE,B_VEHICLE0OBSTACLE0_TF};
	splines["b_vehicle0obstacle0"] = spline_b_vehicle0obstacle0;
	spline_t spline_a_vehicle0obstacle0 = {A_VEHICLE0OBSTACLE0_KNOTS,A_VEHICLE0OBSTACLE0_DEGREE,A_VEHICLE0OBSTACLE0_TF};
	splines["a_vehicle0obstacle0"] = spline_a_vehicle0obstacle0;
	spline_t spline_a_vehicle0obstacle1 = {A_VEHICLE0OBSTACLE1_KNOTS,A_VEHICLE0OBSTACLE1_DEGREE,A_VEHICLE0OBSTACLE1_TF};
	splines["a_vehicle0obstacle1"] = spline_a_vehicle0obstacle1;
	spline_t spline_g_0 = {G_0_KNOTS,G_0_DEGREE,G_0_TF};
	splines["g_0"] = spline_g_0;

}

void MotionPlanning::reset(){
    for (int k=0; k<int(update_time/sample_time)+1; k++){
        for (int j=0; j<n_in; j++){
            this->input_trajectory[k][j] = 0.0;
        }
    }
    for (int i=0; i<n_y; i++){
        for (int d=1; d<n_der+1; d++){
            this->ypred[i][d] = 0.0;
        }
    }
}

bool MotionPlanning::update(vector<double>& state0, vector<double>& stateT, vector<vector<double>>& state_trajectory, vector<vector<double>>& input_trajectory, vector<obstacle_t>& obstacles){
    #ifdef DEBUG
    double tmeas;
    clock_t begin;
    clock_t end;
    #endif
    // transform splines: good init guess for this update
    #ifdef DEBUG
    begin = clock();
    #endif
    transformSplines(current_time);
    #ifdef DEBUG
    end = clock();
    tmeas = double(end-begin)/CLOCKS_PER_SEC;
    cout << "time in transformSplines: " << tmeas << "s" << endl;
    #endif
    // predict initial y for problem
    #ifdef DEBUG
    begin = clock();
    #endif
    if (ideal_update){
        if (fabs(current_time)<=1.e-6){
            vector<double> zeros(n_in);
            getY(state0, zeros, ypred);
        }
        this->y0 = ypred;
    }else{
        predict(state0, this->input_trajectory, this->y0, update_time);
    }
    #ifdef DEBUG
    end = clock();
    tmeas = double(end-begin)/CLOCKS_PER_SEC;
    cout << "time in predict: " << tmeas << "s" << endl;
    #endif
    // translate terminal state/input
    #ifdef DEBUG
    begin = clock();
    #endif
    vector<double> zeros(n_in);
    getY(stateT, zeros, this->yT);
    #ifdef DEBUG
    end = clock();
    tmeas = double(end-begin)/CLOCKS_PER_SEC;
    cout << "time in getY: " << tmeas << "s" << endl;
    #endif
    // solve problem
    #ifdef DEBUG
    begin = clock();
    #endif
    bool check = solve(obstacles);
    #ifdef DEBUG
    end = clock();
    tmeas = double(end-begin)/CLOCKS_PER_SEC;
    cout << "time in solve: " << tmeas << "s" << endl;
    #endif
    // interprete variables (extract y_coeffs, T)
    #ifdef DEBUG
    begin = clock();
    #endif
    interpreteVariables();
    #ifdef DEBUG
    end = clock();
    tmeas = double(end-begin)/CLOCKS_PER_SEC;
    cout << "time in interpreteVariables: " << tmeas << "s" << endl;
    #endif
    // sample y splines and compute input and ideal ypred (y0 of next update = ypred in ideal cases)
    #ifdef DEBUG
    begin = clock();
    #endif
    sampleSplines(ypred, this->state_trajectory, this->input_trajectory);
    #ifdef DEBUG
    end = clock();
    tmeas = double(end-begin)/CLOCKS_PER_SEC;
    cout << "time in sampleSplines: " << tmeas << "s" << endl;
    #endif
    // ref state and input for system are one sample shorter!!
    for (int k=0; k<time.size()-1; k++){
        for (int j=0; j<n_st; j++){
            state_trajectory[k][j] = this->state_trajectory[k][j];
        }
        for (int j=0; j<n_in; j++){
            input_trajectory[k][j] = this->input_trajectory[k][j];
        }
    }
    // update current time
    current_time += update_time;

    return check;
}

bool MotionPlanning::solve(vector<obstacle_t>& obstacles){
    // init variables if first time
    if(fabs(current_time)<=1.e-6){
        initVariables();
    }
    setParameters(obstacles);
    args["p"] = parameters;
    args["x0"] = variables;
    args["lbg"] = lbg;
    args["ubg"] = ubg;
    sol = problem(args);
    solver_output = string(problem.getStats().at("return_status"));
    vector<double> var(sol.at("x"));
    for (int k=0; k<n_var; k++){
        variables[k] = var[k];
    }
    if (solver_output.compare("Solve_Succeeded") != 0){
        cout << solver_output << endl;
        return false;
    } else{
        return true;
    }
}

void MotionPlanning::setParameters(vector<obstacle_t>& obstacles){
    vector<double> y0(n_y);
    vector<double> dy0(n_y*n_der);
    vector<double> yT(n_y);
    vector<double> dyT(n_y*n_der);
    for (int i=0; i<n_y; i++){
        y0[i] = this->y0[i][0];
        yT[i] = this->yT[i][0];
        for (int j=0; j<n_der; j++){
            dy0[i*n_der+j] = this->y0[i][j+1];
            dyT[i*n_der+j] = this->yT[i][j+1];
        }
    }
	for (int i=0; i<2; i++){
		parameters[0+i] = obstacles[0].acceleration[i];
	}
	for (int i=0; i<2; i++){
		parameters[2+i] = obstacles[1].acceleration[i];
	}
	for (int i=0; i<2; i++){
		parameters[4+i] = obstacles[1].velocity[i];
	}
	for (int i=0; i<2; i++){
		parameters[6+i] = obstacles[0].velocity[i];
	}
	for (int i=0; i<2; i++){
		parameters[8+i] = obstacles[1].position[i];
	}
	for (int i=0; i<2; i++){
		parameters[10+i] = obstacles[0].position[i];
	}
	for (int i=0; i<4; i++){
		parameters[12+i] = dy0[i];
	}
	for (int i=0; i<4; i++){
		parameters[16+i] = dyT[i];
	}
	parameters[20] = fmod(round(current_time*1000.)/1000.,horizon_time/10);
	for (int i=0; i<2; i++){
		parameters[21+i] = yT[i];
	}
	for (int i=0; i<2; i++){
		parameters[23+i] = y0[i];
	}
	parameters[25] = horizon_time;

}

void MotionPlanning::initVariables(){
	for (int i=0; i<2; i++){
		for (int j=0; j<3; j++){
			variables[0+i*13+j] = y0[i][0];
			variables[10+i*13+j] = yT[i][0];
		}
		for (int j=0; j<7; j++){
			variables[3+i*13+j] = y0[i][0]+j*(yT[i][0] - y0[i][0])/(6);
		}
	}

}

void MotionPlanning::updateBounds(double current_time){

}

void MotionPlanning::interpreteVariables(){
	for (int i=0; i<2; i++){
		for (int j=0; j<13; j++){
			y_coeffs[i][j] = variables[0+i*13+j];
		}
	}

}

void MotionPlanning::predict(vector<double>& state0, vector<vector<double>>& input, vector<vector<double>>& y, double predictTime)
{
    vector<double> stateT(n_st);
    int steps = int(predictTime/sample_time);
    integrate(state0, input, stateT, steps);
    getY(stateT, input[steps], y); //ok: input[steps] is the one which starts at updateTime
}

void MotionPlanning::integrate(vector<double>& state0, vector<vector<double>>& input, vector<double>& stateT, int steps){
    //integration via 4th order Runge-Kutta
    vector<double> k1(n_st);
    vector<double> k2(n_st);
    vector<double> k3(n_st);
    vector<double> k4(n_st);
    vector<double> st(n_st);
    for (int j=0; j<n_st; j++){
        stateT[j] = state0[j];
    }
    for (int i=0; i<steps; i++){
        updateModel(state0, input[i], k1);
        for (int j=0; j<n_st; j++){
            st[j] = state0[j] + 0.5*sample_time*k1[j];
        }
        updateModel(st, input[i], k2);
        for (int j=0; j<n_st; j++){
            st[j] = state0[j] + 0.5*sample_time*k2[j];
        }
        updateModel(st, input[i], k3);
        for (int j=0; j<n_st; j++){
            st[j] = state0[j] + sample_time*k3[j];
        }
        updateModel(st, input[i+1], k4);
        for (int j=0; j<n_st; j++){
            stateT[j] += (sample_time/6.0)*(k1[j]+2*k2[j]+2*k3[j]+k4[j]);
        }
    }
}

void MotionPlanning::sampleSplines(vector<vector<double>>& y, vector<vector<double>>& state_trajectory, vector<vector<double>>& input_trajectory){
	vector<double> time(this->time);
	for(int k=0;k<time.size();k++){
		time[k] += fmod(round(current_time*1000.)/1000.,horizon_time/10);
	}

    int len_coeffs = splines["y"].knots.size() - y_degree - 1;
    for (int k=0; k<time.size(); k++){
        for (int i=0; i<n_y; i++){
            y[i][0] = evalSpline(time[k]/horizon_time, splines["y"].knots, y_coeffs[i], y_degree);
        }
        for (int d=1; d<n_der+1; d++){
            vector<double> dknots(splines["y"].knots.begin()+d, splines["y"].knots.end()-d);
            for (int i=0; i<n_y; i++){
                vector<double> dcoeffs(len_coeffs-d);
                for (int l=0; l<len_coeffs-d; l++){
                    for (int m=0; m<len_coeffs; m++){
                        dcoeffs[l] += (1.0/pow(horizon_time, d))*derivative_T[d-1][l][m]*y_coeffs[i][m];
                    }
                }
                y[i][d] = evalSpline(time[k]/horizon_time, dknots, dcoeffs, y_degree-d);
            }
        }
        getState(y, state_trajectory[k]);
        getInput(y, input_trajectory[k]);
    }
}

double MotionPlanning::evalSpline(double x, vector<double>& knots, vector<double>& coeffs, int degree){
    int len_k = knots.size();
    double b;
    double bottom;
    double basis[degree+1][len_k-1];
    for (int i=0; i<(len_k-1); i++){
        if((i < degree+1) and (knots[0] == knots[i])){
            basis[0][i] = ((x >= knots[i]) and (x <= knots[i+1]));
        }else{
            basis[0][i] = ((x > knots[i]) and (x <= knots[i + 1]));
        }
    }
    for (int d=1; d<(degree+1); d++){
        for (int i=0; i<(len_k - d - 1); i++){
            b = 0;
            bottom = knots[i+d] - knots[i];
            if (bottom != 0){
                b = (x - knots[i])*basis[d-1][i]/bottom;
            }
            bottom = knots[i+d+1] - knots[i+1];
            if (bottom != 0){
                b += (knots[i+d+1] - x)*basis[d-1][i+1]/bottom;
            }
            basis[d][i] = b;
        }
    }
    double y = 0.0;
    for (int l=0; l<(len_k-degree-1); l++){
        y += coeffs[l]*basis[degree][l];
    }
    return y;
}

void MotionPlanning::transformSplines(double current_time){
	if(((current_time > 0) and fabs(fmod(round(current_time*1000.)/1000., horizon_time/10)) <1.e-6)){
		vector<double> spline_tf(13);
		for(int k=0; k<2; k++){
			for(int i=0; i<13; i++){
			spline_tf[i] = 0.0;
				for(int j=0; j<13; j++){
					spline_tf[i] += splines["y"].transform[i][j]*variables[0+k*13+j];
				}
			}
			for(int i=0; i<13; i++){
				variables[0+k*13+i] = spline_tf[i];
			}
		}
		for(int k=0; k<1; k++){
			for(int i=0; i<11; i++){
			spline_tf[i] = 0.0;
				for(int j=0; j<11; j++){
					spline_tf[i] += splines["b_vehicle0obstacle1"].transform[i][j]*variables[26+k*11+j];
				}
			}
			for(int i=0; i<11; i++){
				variables[26+k*11+i] = spline_tf[i];
			}
		}
		for(int k=0; k<1; k++){
			for(int i=0; i<11; i++){
			spline_tf[i] = 0.0;
				for(int j=0; j<11; j++){
					spline_tf[i] += splines["b_vehicle0obstacle0"].transform[i][j]*variables[37+k*11+j];
				}
			}
			for(int i=0; i<11; i++){
				variables[37+k*11+i] = spline_tf[i];
			}
		}
		for(int k=0; k<2; k++){
			for(int i=0; i<11; i++){
			spline_tf[i] = 0.0;
				for(int j=0; j<11; j++){
					spline_tf[i] += splines["a_vehicle0obstacle0"].transform[i][j]*variables[48+k*11+j];
				}
			}
			for(int i=0; i<11; i++){
				variables[48+k*11+i] = spline_tf[i];
			}
		}
		for(int k=0; k<2; k++){
			for(int i=0; i<11; i++){
			spline_tf[i] = 0.0;
				for(int j=0; j<11; j++){
					spline_tf[i] += splines["a_vehicle0obstacle1"].transform[i][j]*variables[70+k*11+j];
				}
			}
			for(int i=0; i<11; i++){
				variables[70+k*11+i] = spline_tf[i];
			}
		}
		for(int k=0; k<2; k++){
			for(int i=0; i<13; i++){
			spline_tf[i] = 0.0;
				for(int j=0; j<13; j++){
					spline_tf[i] += splines["g_0"].transform[i][j]*variables[92+k*13+j];
				}
			}
			for(int i=0; i<13; i++){
				variables[92+k*13+i] = spline_tf[i];
			}
		}
	}
}

void MotionPlanning::updateModel(vector<double>& state, vector<double>& input, vector<double>& dstate){
	vector<double> _dstate {input[0], input[1]};
	dstate = _dstate;

}

void MotionPlanning::getY(vector<double>& state, vector<double>& input, vector<vector<double>>& y){
	vector<vector<double>> _y {{state[0],input[0],0},{state[1],input[1],0}};
	y = _y;

}

void MotionPlanning::getState(vector<vector<double>>& y, vector<double>& state){
	vector<double> _state {y[0][0],y[1][0]};
	state = _state;

}

void MotionPlanning::getInput(vector<vector<double>>& y, vector<double>& input){
	vector<double> _input {y[0][1],y[1][1]};
	input = _input;

}

void MotionPlanning::setIdealUpdate(bool ideal_update){
    this->ideal_update = ideal_update;
}

}

