// Copyright 2019 Alexander Liniger

// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at

//     http://www.apache.org/licenses/LICENSE-2.0

// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
///////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////

#ifndef MPCC_PARAMS_H
#define MPCC_PARAMS_H

// #include <iostream>
// #include <fstream>
#include <vector>
#include <nlohmann/json.hpp>
#include "config.h"
#include "types.h"

namespace mpcc{
//used namespace
using json = nlohmann::json;

class Param{
public:
    double Cm1;
    double Cm2;
    double Cr0;
    double Cr2;

    double Br;
    double Cr;
    double Dr;

    double Bf;
    double Cf;
    double Df;

    double m;
    double Iz;
    double lf;
    double lr;

    double car_l;
    double car_w;

    double g;

    double r_in;
    double r_out;

    double max_dist_proj;

    double e_long;
    double e_eps;

    double max_alpha;

    double initial_velocity;
    double s_trust_region;

    Param();
    Param(std::string file);

};

class CostParam{
public:
    double q_c;
    double q_l;
    double q_vs;

    double q_mu;

    double q_w;

    double q_beta;
    int beta_kin_cost;

    double r_v;
    double r_w;
    double r_vs;

    double r_dV;
    double r_dW;
    double r_dVs;

    double q_c_N_mult;
    double q_w_N_mult;

    double sc_quad_track;
    double sc_quad_tire;
    double sc_quad_alpha;

    double sc_lin_track;
    double sc_lin_tire;
    double sc_lin_alpha;

    CostParam();
    CostParam(std::string file);

};

class BoundsParam{
public:
    struct LowerStateBounds{
        double X_l;
        double Y_l;
        double th_l;
        double v_l;
        double w_l;
        double s_l;
        double vs_l;
    };
    struct UpperStateBounds{
        double X_u;
        double Y_u;
        double th_u;
        double v_u;
        double w_u;
        double s_u;
        double vs_u;
    };
    struct LowerInputBounds{
        double dV_l;
        double dW_l;
        double dVs_l;
    };
    struct UpperInputBounds{
        double dV_u;
        double dW_u;
        double dVs_u;
    };

    LowerStateBounds lower_state_bounds;
    UpperStateBounds upper_state_bounds;

    LowerInputBounds lower_input_bounds;
    UpperInputBounds upper_input_bounds;

    BoundsParam();
    BoundsParam(std::string file);

};

class NormalizationParam{
public:
    TX_MPC T_x;
    TX_MPC T_x_inv;

    TU_MPC T_u;
    TU_MPC T_u_inv;

    TS_MPC T_s;
    TS_MPC T_s_inv;

    NormalizationParam();
    NormalizationParam(std::string file);
};
}
#endif //MPCC_PARAMS_H
