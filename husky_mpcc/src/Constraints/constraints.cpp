/**
 * Husky Model Predictive Contouring Control
 * 
 * Part of Husky Controller Package, visit https://github.com/MURDriverless/HuskyControl for latest version and instructions on how to use
 * This code is heavily based on Alex Liniger's MPCC repo @ https://github.com/alexliniger/MPCC
 * 
 * Current Version by Kheng Yu Yeoh, contact @ khengyu_061192@hotmail.com
 */

#include "constraints.h"

namespace mpcc{
Constraints::Constraints()
{   
    std::cout << "default constructor, not everything is initialized properly" << std::endl;
}

Constraints::Constraints(double Ts,const PathToJson &path) 
:model_(Ts,path),
param_(Param(path.param_path)),
bounds_(BoundsParam(path.bounds_path))
{
}

OneDConstraint Constraints::getTrackConstraints(const ArcLengthSpline &track,const State &x) const
{
    // given arc length s and the track -> compute linearized track constraints
    const double s = x.s;

    // X-Y point of the center line
    const Eigen::Vector2d pos_center = track.getPostion(s);
    const Eigen::Vector2d d_center   = track.getDerivative(s);
    // Tangent of center line at s
    const Eigen::Vector2d tan_center = {-d_center(1),d_center(0)};

    // inner and outer track boundary given left and right width of track
    // TODO make R_out and R_in dependent on s
    const Eigen::Vector2d pos_outer = pos_center + param_.r_out*tan_center;
    const Eigen::Vector2d pos_inner = pos_center - param_.r_in*tan_center;

    // Define track Jacobian as Perpendicular vector
    C_i_MPC C_track_constraint = C_i_MPC::Zero();
    C_track_constraint(0,0) = tan_center(0);
    C_track_constraint(0,1) = tan_center(1);
    // Compute bounds
    const double track_constraint_lower = tan_center(0)*pos_inner(0) + tan_center(1)*pos_inner(1);
    const double track_constraint_upper = tan_center(0)*pos_outer(0) + tan_center(1)*pos_outer(1);

    // std::cout << "Lower track constraint:" << pos_outer << std::endl;
    // std::cout << "Upper track constraint:" << pos_inner << std::endl;

    return {C_track_constraint,track_constraint_lower,track_constraint_upper};
}

// Actual Husky Dynamics may result in maximum wheel velocity of 1m/s,
// which is not accounted for in unicycle model, allowing it to command
// 1m/s linear velocity and some amount of angular velocity
OneDConstraint Constraints::getLeftWheelConstraints(const State &x) const
{
    // Obtain current velocity
    const double v = x.v;
    const double w = x.w;

    // v and w are related by below equation
    // v = (vL + vR)*0.5
    // w = (vR - vL)/0.555, where 0.555 is the effective width of Husky, signified by param_.husky_track
    // Rearrange, and you'll get
    // vL = v - 0.5*w*param_.husky_track;
    // vR = v + 0.5*w*param_.husky_track;

    // Make Constraint matrix
    C_i_MPC C_vL_constraint = C_i_MPC::Zero();
    C_vL_constraint(si_index.v) = 1; // Partial derivative to v
    C_vL_constraint(si_index.w) = -0.5*param_.husky_track; // Partial derivative to w

    // Bound from bounds.json
    const double vL_constraint_lower = bounds_.lower_state_bounds.v_l;
    const double vL_constraint_upper = bounds_.upper_state_bounds.v_u;

    return {C_vL_constraint,vL_constraint_lower,vL_constraint_upper};
}

// Actual Husky Dynamics may result in maximum wheel velocity of 1m/s,
// which is not accounted for in unicycle model, allowing it to command
// 1m/s linear velocity and some amount of angular velocity
OneDConstraint Constraints::getRightWheelConstraints(const State &x) const
{
    // Obtain current velocity
    const double v = x.v;
    const double w = x.w;

    // v and w are related by below equation
    // v = (vL + vR)*0.5
    // w = (vR - vL)/0.555, where 0.555 is the effective width of Husky, signified by param_.husky_track
    // Rearrange, and you'll get
    // vL = v - 0.5*w*param_.husky_track;
    // vR = v + 0.5*w*param_.husky_track;

    // Make Constraint matrix
    C_i_MPC C_vR_constraint = C_i_MPC::Zero();
    C_vR_constraint(si_index.v) = 1; // Partial derivative to v
    C_vR_constraint(si_index.w) = 0.5*param_.husky_track; // Partial derivative to w

    // Bound from bounds.json
    const double vR_constraint_lower = bounds_.lower_state_bounds.v_l;
    const double vR_constraint_upper = bounds_.upper_state_bounds.v_u;

    return {C_vR_constraint,vR_constraint_lower,vR_constraint_upper};
}

ConstrainsMatrix Constraints::getConstraints(const ArcLengthSpline &track,const State &x,const Input &u) const
{
    // compute all the polytopic state constraints
    ConstrainsMatrix constrains_matrix;
    const OneDConstraint track_constraints = getTrackConstraints(track,x);
    const OneDConstraint leftwheel_constraints = getLeftWheelConstraints(x);
    const OneDConstraint rightwheel_constraints = getLeftWheelConstraints(x);

    C_MPC C_constrains_matrix;
    d_MPC dl_constrains_matrix;
    d_MPC du_constrains_matrix;

    C_constrains_matrix.row(si_index.con_track) = track_constraints.C_i;
    dl_constrains_matrix(si_index.con_track) = track_constraints.dl_i;
    du_constrains_matrix(si_index.con_track) = track_constraints.du_i;

    C_constrains_matrix.row(si_index.con_leftwheel) = leftwheel_constraints.C_i;
    dl_constrains_matrix(si_index.con_leftwheel) = leftwheel_constraints.dl_i;
    du_constrains_matrix(si_index.con_leftwheel) = leftwheel_constraints.du_i;

    C_constrains_matrix.row(si_index.con_rightwheel) = rightwheel_constraints.C_i;
    dl_constrains_matrix(si_index.con_rightwheel) = rightwheel_constraints.dl_i;
    du_constrains_matrix(si_index.con_rightwheel) = rightwheel_constraints.du_i;

    // TODO consider the zero order term directly in the functions construdcing the constraints
    // Missing from main branch
    dl_constrains_matrix = dl_constrains_matrix -  C_constrains_matrix*stateToVector(x);   
    du_constrains_matrix = du_constrains_matrix -  C_constrains_matrix*stateToVector(x);   

    return {C_constrains_matrix,D_MPC::Zero(),dl_constrains_matrix,du_constrains_matrix};
}
}