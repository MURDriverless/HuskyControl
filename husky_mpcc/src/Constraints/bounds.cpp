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

#include "bounds.h"
namespace mpcc{
Bounds::Bounds()
{
    std::cout << "default constructor, not everything is initialized properly" << std::endl;
}

Bounds::Bounds(BoundsParam bounds_param) 
{
    l_bounds_x_(0) = bounds_param.lower_state_bounds.X_l;
    l_bounds_x_(1) = bounds_param.lower_state_bounds.Y_l;
    l_bounds_x_(2) = bounds_param.lower_state_bounds.th_l;
    l_bounds_x_(3) = bounds_param.lower_state_bounds.v_l;
    l_bounds_x_(4) = bounds_param.lower_state_bounds.w_l;
    l_bounds_x_(5) = bounds_param.lower_state_bounds.s_l;
    l_bounds_x_(6) = bounds_param.lower_state_bounds.vs_l;

    u_bounds_x_(0) = bounds_param.upper_state_bounds.X_u;
    u_bounds_x_(1) = bounds_param.upper_state_bounds.Y_u;
    u_bounds_x_(2) = bounds_param.upper_state_bounds.th_u;
    u_bounds_x_(3) = bounds_param.upper_state_bounds.v_u;
    u_bounds_x_(4) = bounds_param.upper_state_bounds.w_u;
    u_bounds_x_(5) = bounds_param.upper_state_bounds.s_u;
    u_bounds_x_(6) = bounds_param.upper_state_bounds.vs_u;
    
    l_bounds_u_(0) = bounds_param.lower_input_bounds.dV_l;
    l_bounds_u_(1) = bounds_param.lower_input_bounds.dW_l;
    l_bounds_u_(2) = bounds_param.lower_input_bounds.dVs_l;

    u_bounds_u_(0) = bounds_param.upper_input_bounds.dV_u;
    u_bounds_u_(1) = bounds_param.upper_input_bounds.dW_u;
    u_bounds_u_(2) = bounds_param.upper_input_bounds.dVs_u;

    l_bounds_s_ = Bounds_s::Zero();
    u_bounds_s_ = Bounds_s::Zero();

    std::cout << "bounds initialized" << std::endl;
}

// Added minus term all over here compared to main
Bounds_x Bounds::getBoundsLX(const State &x) const
{
    return  l_bounds_x_-stateToVector(x);
}

Bounds_x Bounds::getBoundsUX(const State &x) const
{
    return  u_bounds_x_-stateToVector(x);
}

Bounds_u Bounds::getBoundsLU(const Input &u) const
{
    return  l_bounds_u_-inputToVector(u);
}

Bounds_u Bounds::getBoundsUU(const Input &u) const
{
    return  u_bounds_u_-inputToVector(u);
}

Bounds_s Bounds::getBoundsLS() const
{
    return  l_bounds_s_;
}

Bounds_s Bounds::getBoundsUS() const{
    return  u_bounds_s_;
}
}