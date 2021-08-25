/**
 * Husky Model Predictive Contouring Control
 * 
 * Part of Husky Controller Package, visit https://github.com/MURDriverless/HuskyControl for latest version and instructions on how to use
 * This code is heavily based on Alex Liniger's MPCC repo @ https://github.com/alexliniger/MPCC
 * 
 * Current Version by Kheng Yu Yeoh, contact @ khengyu_061192@hotmail.com
 */

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

    StateVector f;
    f(0) = v*std::cos(th);
    f(1) = v*std::sin(th);
    f(2) = w;
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

    A_MPC A_c = A_MPC::Zero();
    B_MPC B_c = B_MPC::Zero();
    g_MPC g_c = g_MPC::Zero();

    const StateVector f = getF(x,u);

    // Partial Derivatives of state
    // f1 = v*std::cos(th)
    const double df1_dth = -v*std::sin(th);
    const double df1_dv  = std::cos(th);

    // f2 = v*std::sin(th)
    const double df2_dth = v*std::cos(th);
    const double df2_dv  = std::sin(th);

    // f3 = w;
    const double df3_dw = 1.0;

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
    // all zero
    // Column 7, dvs
    A_c(si_index.s,si_index.vs) = 1.0; // s

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

LinModelMatrix Model::getLinModel(const State &x, const Input &u, const State &x_next) const
{
    // compute linearized and discretized model
    const LinModelMatrix lin_model_c = getModelJacobian(x,u);
    // discretize the system
    return discretizeModel(lin_model_c,x,u,x_next);
}
}