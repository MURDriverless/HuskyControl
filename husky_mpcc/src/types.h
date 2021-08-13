// Define States of Husky

#ifndef MPCC_TYPES_H
#define MPCC_TYPES_H

#include "config.h"
namespace mpcc{
struct State{
    double X;
    double Y;
    double th; // theta
    double v; // linear velocity in x
    double w; // angular velocity in z
    double s;
    double vs;
    
    void setZero()
    {
        X = 0.0;
        Y = 0.0;
        th = 0.0;
        v = 0.0;
        w = 0.0;
        s = 0.0;
        vs = 0.0;
    }

    void set(double X, double Y, double th, double v, double w, double s, double vs)
    {
        this->X = X;
        this->Y = Y;
        this->th = th;
        this->v = v;
        this->w = w;
        this->s = s;
        this->vs = vs;
    }

    // Fix angle issues when near -180 and 180 deg
    // Also allows restarting of lap
    void unwrap(double track_length)
    {
        if (th > M_PI)
            th -= 2.0 * M_PI;
        if (th < -M_PI)
            th += 2.0 * M_PI;

        if (s > track_length)
            s -= track_length;
        if (s < 0)
            s += track_length;
    }
};

struct Input{
    double dV;
    double dW;
    double dVs;

    void setZero()
    {
        dV = 0.0;
        dW = 0.0;
        dVs = 0.0;
    }
};

struct PathToJson{
    const std::string param_path;
    const std::string cost_path;
    const std::string bounds_path;
    const std::string track_path;
    const std::string normalization_path;
};

typedef Eigen::Matrix<double,NX,1> StateVector;
typedef Eigen::Matrix<double,NU,1> InputVector;

typedef Eigen::Matrix<double,NX,NX> A_MPC;
typedef Eigen::Matrix<double,NX,NU> B_MPC;
typedef Eigen::Matrix<double,NX,1> g_MPC;

typedef Eigen::Matrix<double,NX,NX> Q_MPC;
typedef Eigen::Matrix<double,NU,NU> R_MPC;
typedef Eigen::Matrix<double,NX,NU> S_MPC;

typedef Eigen::Matrix<double,NX,1> q_MPC;
typedef Eigen::Matrix<double,NU,1> r_MPC;

typedef Eigen::Matrix<double,NPC,NX> C_MPC;
typedef Eigen::Matrix<double,1,NX> C_i_MPC;
typedef Eigen::Matrix<double,NPC,NU> D_MPC;
typedef Eigen::Matrix<double,NPC,1> d_MPC;

typedef Eigen::Matrix<double,NS,NS> Z_MPC;
typedef Eigen::Matrix<double,NS,1> z_MPC;

typedef Eigen::Matrix<double,NX,NX> TX_MPC;
typedef Eigen::Matrix<double,NU,NU> TU_MPC;
typedef Eigen::Matrix<double,NS,NS> TS_MPC;

typedef Eigen::Matrix<double,NX,1> Bounds_x;
typedef Eigen::Matrix<double,NU,1> Bounds_u;
typedef Eigen::Matrix<double,NS,1> Bounds_s;

StateVector stateToVector(const State &x);
InputVector inputToVector(const Input &u);

State vectorToState(const StateVector &xk);
Input vectorToInput(const InputVector &uk);

State arrayToState(double *xk);
Input arrayToInput(double *uk);
}
#endif //MPCC_TYPES_H
