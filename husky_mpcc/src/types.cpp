// State Functions for Husky

#include "types.h"
namespace mpcc{

StateVector stateToVector(const State &x)
{
    StateVector xk;
    xk(0) = x.X;
    xk(1) = x.Y;
    xk(2) = x.th;
    xk(3) = x.v;
    xk(4) = x.w;
    xk(5) = x.s;
    xk(6) = x.vs;
    return xk;
}

InputVector inputToVector(const Input &u)
{
    InputVector uk = {u.dV,u.dW,u.dVs};
    return uk;
}

State vectorToState(const StateVector &xk)
{
    State x;
    x.X     = xk(0);
    x.Y     = xk(1);
    x.th    = xk(2);
    x.v     = xk(3);
    x.w     = xk(4);
    x.s     = xk(5);
    x.vs    = xk(6);

    return x;
}

Input vectorToInput(const InputVector &uk)
{
    Input u;
    u.dV     = uk(0);
    u.dW     = uk(1);
    u.dVs    = uk(2);

    return u;
}

State arrayToState(double *xk)
{
    State x;
    x.X     = xk[0];
    x.Y     = xk[1];
    x.th    = xk[2];
    x.v     = xk[3];
    x.w     = xk[4];
    x.s     = xk[5];
    x.vs    = xk[6];
    
    return x;
}

Input arrayToInput(double *uk)
{
    Input u;
    u.dV     = uk[0];
    u.dW     = uk[1];
    u.dVs    = uk[2];

    return u;
}

}