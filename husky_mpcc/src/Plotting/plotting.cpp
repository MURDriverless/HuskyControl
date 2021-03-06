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

#include "plotting.h"
namespace mpcc{

Plotting::Plotting(double Ts,PathToJson path)
:model_(Model(Ts,path)),
param_(Param(path.param_path))
{
}
void Plotting::plotRun(const std::list<MPCReturn> &log, const TrackPos &track_xy, const mpcc::ArcLengthSpline &track, const std::vector<double> plot_lapTime) const
{

    std::vector<double> plot_xc(track_xy.X.data(),track_xy.X.data() + track_xy.X.size());
    std::vector<double> plot_yc(track_xy.Y.data(),track_xy.Y.data() + track_xy.Y.size());

    std::vector<double> plot_xi(track_xy.X_inner.data(),track_xy.X_inner.data() + track_xy.X_inner.size());
    std::vector<double> plot_yi(track_xy.Y_inner.data(),track_xy.Y_inner.data() + track_xy.Y_inner.size());
    std::vector<double> plot_xo(track_xy.X_outer.data(),track_xy.X_outer.data() + track_xy.X_outer.size());
    std::vector<double> plot_yo(track_xy.Y_outer.data(),track_xy.Y_outer.data() + track_xy.Y_outer.size());

    std::vector<double> plot_x;
    std::vector<double> plot_y;
    std::vector<double> plot_th;
    std::vector<double> plot_v;
    std::vector<double> plot_w;
    std::vector<double> plot_s;
    std::vector<double> plot_vs;
    
    std::vector<double> plot_dV;
    std::vector<double> plot_dW;
    std::vector<double> plot_dVs;

    std::vector<double> plot_innerbound_x;
    std::vector<double> plot_innerbound_y;
    std::vector<double> plot_outerbound_x;
    std::vector<double> plot_outerbound_y;

    for(MPCReturn log_i : log)
    {
        plot_x.push_back(log_i.mpc_horizon[0].xk.X);
        plot_y.push_back(log_i.mpc_horizon[0].xk.Y);
        plot_th.push_back(log_i.mpc_horizon[0].xk.th);
        plot_v.push_back(log_i.mpc_horizon[0].xk.v);
        plot_w.push_back(log_i.mpc_horizon[0].xk.w);
        plot_s.push_back(log_i.mpc_horizon[0].xk.s);
        plot_vs.push_back(log_i.mpc_horizon[0].xk.vs);
        
        plot_dV.push_back(log_i.mpc_horizon[0].uk.dV);
        plot_dW.push_back(log_i.mpc_horizon[0].uk.dW);
        plot_dVs.push_back(log_i.mpc_horizon[0].uk.dVs);

        // given arc length s and the track -> compute linearized track constraints
        double s = log_i.mpc_horizon[0].xk.s;

        // X-Y point of the center line
        Eigen::Vector2d pos_center = track.getPostion(s);
        Eigen::Vector2d d_center   = track.getDerivative(s);
        // Tangent of center line at s
        Eigen::Vector2d tan_center = {-d_center(1),d_center(0)};

        // inner and outer track boundary given left and right width of track
        Eigen::Vector2d pos_outer = pos_center + param_.r_out*tan_center;
        Eigen::Vector2d pos_inner = pos_center - param_.r_in*tan_center;

        // inner estimated boundary
        plot_innerbound_x.push_back(pos_inner(0));
        plot_innerbound_y.push_back(pos_inner(1));

        // outer estimated boundary
        plot_outerbound_x.push_back(pos_outer(0));
        plot_outerbound_y.push_back(pos_outer(1));
    }

    plt::figure(1);
    plt::named_plot("Track Bounds", plot_xi,plot_yi,"k-");
    plt::plot(plot_xo,plot_yo,"k-");
    plt::named_plot("Predicted Bounds", plot_innerbound_x,plot_innerbound_y,"r-");
    plt::plot(plot_outerbound_x,plot_outerbound_y,"r-");
    plt::named_plot("Actual Path", plot_x,plot_y,"b-");
    plt::named_plot("Centerline Reference", plot_xc,plot_yc,"r--");
    plt::axis("equal");
    plt::xlabel("X [m]");
    plt::ylabel("Y [m]");
    plt::xlim(-39, 47);
    plt::ylim(-77, 3);
    plt::legend();

    plt::figure(2);
    plt::subplot(3,1,1);
    plt::plot(plot_x);
    plt::ylabel("X [m]");
    plt::subplot(3,1,2);
    plt::plot(plot_y);
    plt::ylabel("Y [m]");
    plt::subplot(3,1,3);
    plt::plot(plot_th);
    plt::ylabel("theta [rad]");

    plt::figure(3);
    plt::subplot(3,1,1);
    plt::plot(plot_v);
    plt::ylabel("linear velocity [m/s]");
    plt::subplot(3,1,2);
    plt::plot(plot_w);
    plt::ylabel("angular velocity [rad/s]");
    plt::subplot(3,1,3);
    plt::plot(plot_vs);
    plt::ylabel("v_s [m/s]");

    plt::figure(4);
    plt::subplot(3,1,1);
    plt::plot(plot_dV);
    plt::ylabel("dot{lin vel} [m/s2]");
    plt::subplot(3,1,2);
    plt::plot(plot_dW);
    plt::ylabel("dot{ang vel} [rad/s2]");
    plt::subplot(3,1,3);
    plt::plot(plot_dVs);
    plt::ylabel("dot{v_s} [m/s^2]");

    plt::figure(5);
    plt::bar(plot_lapTime);
    plt::ylabel("lap time [s]");

    plt::show();
}
void Plotting::plotSim(const std::list<MPCReturn> &log, const TrackPos &track_xy, const mpcc::ArcLengthSpline &track) const
{
    std::vector<double> plot_xc(track_xy.X.data(),track_xy.X.data() + track_xy.X.size());
    std::vector<double> plot_yc(track_xy.Y.data(),track_xy.Y.data() + track_xy.Y.size());

    std::vector<double> plot_xi(track_xy.X_inner.data(),track_xy.X_inner.data() + track_xy.X_inner.size());
    std::vector<double> plot_yi(track_xy.Y_inner.data(),track_xy.Y_inner.data() + track_xy.Y_inner.size());
    std::vector<double> plot_xo(track_xy.X_outer.data(),track_xy.X_outer.data() + track_xy.X_outer.size());
    std::vector<double> plot_yo(track_xy.Y_outer.data(),track_xy.Y_outer.data() + track_xy.Y_outer.size());

    std::vector<double> plot_horizonx;
    std::vector<double> plot_horizony;
    std::vector<double> plot_x;
    std::vector<double> plot_y;

    std::vector<double> plot_innerbound_x;
    std::vector<double> plot_innerbound_y;
    std::vector<double> plot_outerbound_x;
    std::vector<double> plot_outerbound_y;

    for(MPCReturn log_i : log)
    {
        plot_x.push_back(log_i.mpc_horizon[0].xk.X);
        plot_y.push_back(log_i.mpc_horizon[0].xk.Y);
    }

    for(MPCReturn log_i : log)
    {
        plot_horizonx.resize(0);
        plot_horizony.resize(0);
        plot_innerbound_x.resize(0);
        plot_innerbound_y.resize(0);
        plot_outerbound_x.resize(0);
        plot_outerbound_y.resize(0);
        for(int j=0;j<log_i.mpc_horizon.size();j++)
        {
            plot_horizonx.push_back(log_i.mpc_horizon[j].xk.X);
            plot_horizony.push_back(log_i.mpc_horizon[j].xk.Y);

            // given arc length s and the track -> compute linearized track constraints
            double s = log_i.mpc_horizon[j].xk.s;

            // X-Y point of the center line
            Eigen::Vector2d pos_center = track.getPostion(s);
            Eigen::Vector2d d_center   = track.getDerivative(s);
            // Tangent of center line at s
            Eigen::Vector2d tan_center = {-d_center(1),d_center(0)};

            // inner and outer track boundary given left and right width of track
            Eigen::Vector2d pos_outer = pos_center + param_.r_out*tan_center;
            Eigen::Vector2d pos_inner = pos_center - param_.r_in*tan_center;

            // inner estimated boundary
            plot_innerbound_x.push_back(pos_inner(0));
            plot_innerbound_y.push_back(pos_inner(1));

            // outer estimated boundary
            plot_outerbound_x.push_back(pos_outer(0));
            plot_outerbound_y.push_back(pos_outer(1));
        }
        plt::clf();
        plt::named_plot("Track Bounds", plot_xi,plot_yi,"k-");
        plt::plot(plot_xo,plot_yo,"k-");
        plt::named_plot("Predicted Bounds", plot_innerbound_x,plot_innerbound_y,"r-");
        plt::plot(plot_outerbound_x,plot_outerbound_y,"r-");
        plotBox(log_i.mpc_horizon[0].xk);
        plt::named_plot("Actual Path", plot_x,plot_y,"b-");
        plt::named_plot("Centerline Reference", plot_xc,plot_yc,"r--");
        plt::named_plot("Prediction Horizon Path", plot_horizonx,plot_horizony,"g-");
        plt::axis("equal");
        plt::legend();
        // plt::xlim(-2,2);
        // plt::ylim(-2,2);
        plt::pause(0.01);
    }
}

void Plotting::plotBox(const State &x0) const
{
    std::vector<double> corner_x;
    std::vector<double> corner_y;
    double body_xl = std::cos(x0.th)*param_.car_l;
    double body_xw = std::sin(x0.th)*param_.car_w;
    double body_yl = std::sin(x0.th)*param_.car_l;
    double body_yw = -std::cos(x0.th)*param_.car_w;

    corner_x.push_back(x0.X + body_xl + body_xw);
    corner_x.push_back(x0.X + body_xl - body_xw);
    corner_x.push_back(x0.X - body_xl - body_xw);
    corner_x.push_back(x0.X - body_xl + body_xw);
    corner_x.push_back(x0.X + body_xl + body_xw);

    corner_y.push_back(x0.Y + body_yl + body_yw);
    corner_y.push_back(x0.Y + body_yl - body_yw);
    corner_y.push_back(x0.Y - body_yl - body_yw);
    corner_y.push_back(x0.Y - body_yl + body_yw);
    corner_y.push_back(x0.Y + body_yl + body_yw);

    plt::plot(corner_x,corner_y,"k-");
}
}