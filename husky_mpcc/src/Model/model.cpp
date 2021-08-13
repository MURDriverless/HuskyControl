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

#include "model.h"
namespace mpcc{
Model::Model()
:Ts_(1.0)
{
    std::cout << "default constructor, not everything is initialized properly" << std::endl;
}

Model::Model(double Ts,const PathToJson &path)
:Ts_(Ts),param_(Param(path.param_path))
{
}

// double Model::getSlipAngleFront(const State &x) const
// {
//     // compute slip angels given current state
//     return -std::atan2(x.vy+x.r*param_.lf,x.vx) + x.delta;
// }

// double Model::getSlipAngleRear(const State &x) const
// {
//     // compute slip angels given current state
//     return -std::atan2(x.vy-x.r*param_.lr,x.vx);
// }

// TireForces Model::getForceFront(const State &x) const
// {
//     const double alpha_f = getSlipAngleFront(x);
//     const double F_y = param_.Df * std::sin(param_.Cf * std::atan(param_.Bf * alpha_f ));
//     const double F_x = 0.0;

//     return {F_y,F_x};
// }

// TireForces Model::getForceRear(const State &x) const
// {
//     const double alpha_r = getSlipAngleRear(x);
//     const double F_y = param_.Dr * std::sin(param_.Cr * std::atan(param_.Br * alpha_r ));
//     const double F_x = param_.Cm1*x.D - param_.Cm2*x.D*x.vx;// - param_.Cr0 - param_.Cr2*std::pow(x.vx,2.0);

//     return {F_y,F_x};
// }

// double Model::getForceFriction(const State &x) const
// {
//     return -param_.Cr0 - param_.Cr2*std::pow(x.v,2.0);
// }

// NormalForces Model::getForceNormal(const State &x) const
// {
//     // at this point aero forces could be modeled
//     const double f_n_front = param_.lr/(param_.lf + param_.lr)*param_.m*param_.g;
//     const double f_n_rear = param_.lf/(param_.lf + param_.lr)*param_.m*param_.g;
//     return {f_n_front,f_n_rear};
// }

// TireForcesDerivatives Model::getForceFrontDerivatives(const State &x) const
// {
//     const double alpha_f = getSlipAngleFront(x);
//     const double vx = x.vx;
//     const double vy = x.vy;
//     const double r  = x.r;

//     // F_fx
//     const double dF_x_vx    = 0.0;
//     const double dF_x_vy    = 0.0;
//     const double dF_x_r     = 0.0;
//     const double dF_x_D     = 0.0;
//     const double dF_x_delta = 0.0;
//     // F_fy
//     const double dF_y_vx    = (param_.Bf*param_.Cf*param_.Df*std::cos(param_.Cf*std::atan(param_.Bf*alpha_f)))
//                                             /(1.+std::pow(param_.Bf,2)*std::pow(alpha_f,2))*((param_.lf*r + vy)
//                                             /(std::pow((param_.lf*r + vy),2)+std::pow(vx,2)));
//     const double dF_y_vy    = (param_.Bf*param_.Cf*param_.Df*std::cos(param_.Cf*std::atan(param_.Bf*alpha_f)))
//                                             /(1.+std::pow(param_.Bf,2)*std::pow(alpha_f,2))
//                                             *(-vx/(std::pow((param_.lf*r + vy),2)+std::pow(vx,2)));
//     const double dF_y_r     =  (param_.Bf*param_.Cf*param_.Df*std::cos(param_.Cf*std::atan(param_.Bf*alpha_f)))
//                                             /(1.+std::pow(param_.Bf,2)*std::pow(alpha_f,2))*((-param_.lf*vx)
//                                             /(std::pow((param_.lf*r + vy),2)+std::pow(vx,2)));
//     const double dF_y_D     =  0.0;
//     const double dF_y_delta = (param_.Bf*param_.Cf*param_.Df*std::cos(param_.Cf*std::atan(param_.Bf*alpha_f)))
//                                             /(1.+std::pow(param_.Bf,2)*std::pow(alpha_f,2));

//     return {dF_y_vx,dF_y_vy,dF_y_r,dF_y_D,dF_y_delta,dF_x_vx,dF_x_vy,dF_x_r,dF_x_D,dF_x_delta};
// }

// TireForcesDerivatives Model::getForceRearDerivatives(const State &x) const
// {
//     const double alpha_r = getSlipAngleRear(x);
//     const double vx = x.vx;
//     const double vy = x.vy;
//     const double r  = x.r;
//     const double D  = x.D;

//     //F_rx
//     const double dF_x_vx    = -param_.Cm2*D;// - 2.0*param_.Cr2*vx;
//     const double dF_x_vy    = 0.0;
//     const double dF_x_r     = 0.0;
//     const double dF_x_D     = param_.Cm1 - param_.Cm2*vx;
//     const double dF_x_delta = 0.0;
//     // F_ry
//     const double dF_y_vx    = ((param_.Br*param_.Cr*param_.Dr*std::cos(param_.Cr*std::atan(param_.Br*alpha_r)))
//                                             /(1.+std::pow(param_.Br,2)*std::pow(alpha_r,2)))*(-(param_.lr*r - vy)
//                                             /(std::pow((-param_.lr*r + vy),2)+std::pow(vx,2)));
//     const double dF_y_vy    = ((param_.Br*param_.Cr*param_.Dr*std::cos(param_.Cr*std::atan(param_.Br*alpha_r)))
//                                             /(1.+std::pow(param_.Br,2)*std::pow(alpha_r,2)))
//                                             *((-vx)/(std::pow((-param_.lr*r + vy),2)+std::pow(vx,2)));
//     const double dF_y_r     = ((param_.Br*param_.Cr*param_.Dr*std::cos(param_.Cr*std::atan(param_.Br*alpha_r)))
//                                             /(1.+std::pow(param_.Br,2)*std::pow(alpha_r,2)))*((param_.lr*vx)
//                                             /(std::pow((-param_.lr*r + vy),2)+std::pow(vx,2)));
//     const double dF_y_D     = 0.0;
//     const double dF_y_delta = 0.0;

//     return {dF_y_vx,dF_y_vy,dF_y_r,dF_y_D,dF_y_delta,dF_x_vx,dF_x_vy,dF_x_r,dF_x_D,dF_x_delta};
// }

// FrictionForceDerivatives Model::getForceFrictionDerivatives(const State &x) const
// {
//     return {-2.0*param_.Cr2*x.v,0.0,0.0,0.0,0.0};
// }

// Get X_dot values at particular x and u
StateVector Model::getF(const State &x,const Input &u) const
{
    const double th = x.th;
    const double v = x.v;
    const double w  = x.w;
    const double vs = x.vs;

    const double dV = u.dV;
    const double dW = u.dW;
    const double dVs = u.dVs;

    // const TireForces tire_forces_front = getForceFront(x);
    // const TireForces tire_forces_rear  = getForceRear(x);
    // const double friction_force = getForceFriction(x);

    StateVector f;
    f(0) = v*std::cos(th);
    f(1) = v*std::sin(th);
    f(2) = w;
    // f(3) = 1.0/param_.m*(tire_forces_rear.F_x + friction_force - tire_forces_front.F_y*std::sin(delta) + param_.m*vy*r);
    // f(4) = 1.0/param_.m*(tire_forces_rear.F_y + tire_forces_front.F_y*std::cos(delta) - param_.m*vx*r);
    // f(5) = 1.0/param_.Iz*(tire_forces_front.F_y*param_.lf*std::cos(delta) - tire_forces_rear.F_y*param_.lr);
    f(3) = dV;
    f(4) = dW;
    f(5) = vs;
    f(6) = dVs;

    return f;
}

// Successive Linearization
LinModelMatrix Model::getModelJacobian(const State &x, const Input &u) const
{
    // compute jacobian of the model
    // state values
    const double th = x.th;
    const double v = x.v;
    const double w = x.w;

//    LinModelMatrix lin_model_c;
    A_MPC A_c = A_MPC::Zero();
    B_MPC B_c = B_MPC::Zero();
    g_MPC g_c = g_MPC::Zero();

    const StateVector f = getF(x,u);

//     const TireForces F_front = getForceFront(x);
// //    TireForces F_rear  = getForceRear(x);

//     const TireForcesDerivatives dF_front = getForceFrontDerivatives(x);
//     const TireForcesDerivatives dF_rear  = getForceRearDerivatives(x);
//     const FrictionForceDerivatives dF_fric = getForceFrictionDerivatives(x);

    // Partial Derivatives of state
    // f1 = v*std::cos(th)
    const double df1_dth = -v*std::sin(th);
    const double df1_dv  = std::cos(th);

    // f2 = v*std::sin(th)
    const double df2_dth = v*std::cos(th);
    const double df2_dv  = std::sin(th);

    // f3 = w;
    const double df3_dw = 1.0;

    // // f4 = 1/param_.m*(F_rx + F_fric - F_fy*std::sin(delta) + param_.m*v_y*r);
    // const double df4_dvx     = 1.0/param_.m*(dF_rear.dF_x_vx + dF_fric.dF_f_vx - dF_front.dF_y_vx*std::sin(delta));
    // const double df4_dvy     = 1.0/param_.m*(                - dF_front.dF_y_vy*std::sin(delta)    + param_.m*r);
    // const double df4_dr      = 1.0/param_.m*(                - dF_front.dF_y_r*std::sin(delta)     + param_.m*vy);
    // const double df4_dD      = 1.0/param_.m* dF_rear.dF_x_D;
    // const double df4_ddelta  = 1.0/param_.m*(                - dF_front.dF_y_delta*std::sin(delta) - F_front.F_y*std::cos(delta));

    // // f5 = 1/param_.m*(F_ry + F_fy*std::cos(delta) - param_.m*v_x*r);
    // const double df5_dvx     = 1.0/param_.m*(dF_rear.dF_y_vx  + dF_front.dF_y_vx*std::cos(delta)    - param_.m*r);
    // const double df5_dvy     = 1.0/param_.m*(dF_rear.dF_y_vy  + dF_front.dF_y_vy*std::cos(delta));
    // const double df5_dr      = 1.0/param_.m*(dF_rear.dF_y_r   + dF_front.dF_y_r*std::cos(delta)     - param_.m*vx);
    // const double df5_ddelta  = 1.0/param_.m*(                   dF_front.dF_y_delta*std::cos(delta) - F_front.F_y*std::sin(delta));

    // // f6 = 1/param_.Iz*(F_fy*l_f*std::cos(delta)- F_ry*l_r)
    // const double df6_dvx     = 1.0/param_.Iz*(dF_front.dF_y_vx*param_.lf*std::cos(delta)    - dF_rear.dF_y_vx*param_.lr);
    // const double df6_dvy     = 1.0/param_.Iz*(dF_front.dF_y_vy*param_.lf*std::cos(delta)    - dF_rear.dF_y_vy*param_.lr);
    // const double df6_dr      = 1.0/param_.Iz*(dF_front.dF_y_r*param_.lf*std::cos(delta)     - dF_rear.dF_y_r*param_.lr);
    // const double df6_ddelta  = 1.0/param_.Iz*(dF_front.dF_y_delta*param_.lf*std::cos(delta) - F_front.F_y*param_.lf*std::sin(delta));

    // Jacobians
    // Matrix A
    // Column 1, dx
    // all zero
    // Column 2, dy
    // all zero
    // Column 3, dth
    A_c(si_index.X,si_index.th) = df1_dth; // x
    A_c(si_index.Y,si_index.th) = df2_dth; // y
    // Column 4, dv
    A_c(si_index.X,si_index.v) = df1_dv; // x
    A_c(si_index.Y,si_index.v) = df2_dv; // y
    // Column 5, dw
    A_c(si_index.th,si_index.w) = df3_dw; // th
    // Column 6, ds
    A_c(si_index.s,si_index.vs) = 1.0; // s
    // Column 7, dvs
    // all zero

    // Matrix B
    // Column 1, d dV
    B_c(si_index.v,si_index.dV) = 1.0; // dV
    // Column 2, d dW
    B_c(si_index.w,si_index.dW) = 1.0; // dW
    // Column 3, d dVs
    B_c(si_index.vs,si_index.dVs) = 1.0; // dVs

    //zero order term
    g_c = f - A_c*stateToVector(x) - B_c*inputToVector(u);

    return {A_c,B_c,g_c};
}

// Mixed RK4 - EXPM method
LinModelMatrix Model::discretizeModel(const LinModelMatrix &lin_model_c, const State &x, const Input &u,const State &x_next) const
{
    // exact ZOH discretization
    // A_d = expm(A*Ts)
    const A_MPC A_d = (lin_model_c.A*Ts_).exp();

    // B_d = A\(A_d - I)*B
    A_MPC eye_A = Eigen::Matrix<double,NX,NX>::Identity();
    A_MPC tmp = A_d - eye_A;
    const B_MPC B_d = lin_model_c.A.fullPivLu().solve(tmp*lin_model_c.B); // https://eigen.tuxfamily.org/dox/group__TutorialLinearAlgebra.html

    // TODO: use correct RK4 instead of inline RK4,
    // main uses expm but here use RK4
    const StateVector x_vec = stateToVector(x);

    const StateVector k1 = getF(vectorToState(x_vec),u);
    const StateVector k2 = getF(vectorToState(x_vec+Ts_/2.*k1),u);
    const StateVector k3 = getF(vectorToState(x_vec+Ts_/2.*k2),u);
    const StateVector k4 = getF(vectorToState(x_vec+Ts_*k3),u);
    // combining to give output
    const StateVector x_RK = x_vec + Ts_*(k1/6.+k2/3.+k3/3.+k4/6.);

    const g_MPC g_d =  -stateToVector(x_next) + x_RK;

    // return {A_d,B_d,g_d};
    return {A_d,B_d,g_d};
}

// // EF Method (not suited for used input lifting technique)
// LinModelMatrix Model::discretizeModel(const LinModelMatrix &lin_model_c, const State &x, const Input &u,const State &x_next) const
// {
//     // disctetize the continuous time linear model \dot x = A x + B u + g using ZHO
//     Eigen::Matrix<double,NX+NU,NX+NU> temp = Eigen::Matrix<double,NX+NU,NX+NU>::Zero();
//     // building matrix necessary for expm
//     // temp = Ts*[A,B,g;zeros]
//     temp.block<NX,NX>(0,0) = lin_model_c.A;
//     temp.block<NX,NU>(0,NX) = lin_model_c.B;
//     temp = temp*Ts_;

//     Eigen::Matrix<double,NX+NU,NX+NU> eye;
//     eye.setIdentity();
//     // take the matrix exponential of temp
//     const Eigen::Matrix<double,NX+NU,NX+NU> temp_res = eye + temp;
//     // extract dynamics out of big matrix
//     // x_{k+1} = Ad x_k + Bd u_k
//     //temp_res = [Ad,Bd;zeros]
//     const A_MPC A_d = temp_res.block<NX,NX>(0,0);
//     const B_MPC B_d = temp_res.block<NX,NU>(0,NX);

//     const StateVector x_vec = stateToVector(x);

//     const StateVector f = getF(vectorToState(x_vec),u);
//     // combining to give output
//     const StateVector x_EF = x_vec + Ts_*f;

//     const g_MPC g_d =  -stateToVector(x_next) + x_EF;
//     // return {A_d,B_d,g_d};
//     return {A_d,B_d,g_d};
// }

LinModelMatrix Model::getLinModel(const State &x, const Input &u, const State &x_next) const
{
    // compute linearized and discretized model
    const LinModelMatrix lin_model_c = getModelJacobian(x,u);
    // discretize the system
    return discretizeModel(lin_model_c,x,u,x_next);
}
}