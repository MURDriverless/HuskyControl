/**
 * Husky Model Predictive Contouring Control
 * 
 * Part of Husky Controller Package, visit https://github.com/MURDriverless/HuskyControl for latest version and instructions on how to use
 * This code is heavily based on Alex Liniger's MPCC repo @ https://github.com/alexliniger/MPCC
 * 
 * Current Version by Kheng Yu Yeoh, contact @ khengyu_061192@hotmail.com
 */

// #include "Tests/spline_test.h"
// #include "Tests/model_integrator_test.h"
// #include "Tests/constratins_test.h"
// #include "Tests/cost_test.h"

#include "MPC/mpc.h"
#include "Model/integrator.h"
#include "Params/track.h"
#include "Plotting/plotting.h"

#include "ros/ros.h" // Must include for all ROS C++
#include "ROSnode/fastlapnode.h"

#include <nlohmann/json.hpp>
using json = nlohmann::json;

#include <stack>
#include <ctime>
#include <chrono>

// Time testing
std::stack<clock_t> tictoc_stack;

void tic() {
    tictoc_stack.push(clock());
}

void toc() {
    double t = ((double)(clock() - tictoc_stack.top())) / CLOCKS_PER_SEC;
    std::cout << "Time elapsed: "
              << t
              << "s, Hz: "
              << 1.0/t
              << std::endl;
    tictoc_stack.pop();
}

// Main
int main(int argc, char **argv) {
    // Initialize ROS node
    ros::init(argc, argv, "husky_mpcc");

    // Get parameters from CLI
    bool comm = atoi(argv[1]);
    bool skip = atoi(argv[2]);

    using namespace mpcc;
    std::ifstream iConfig("/home/khengyu/catkin_ws/src/husky_mpcc/src/Params/config.json");
    json jsonConfig;
    iConfig >> jsonConfig;

    PathToJson json_paths {jsonConfig["model_path"],
                           jsonConfig["cost_path"],
                           jsonConfig["bounds_path"],
                           jsonConfig["track_path"],
                           jsonConfig["normalization_path"]};

    // std::cout << testSpline() << std::endl;
    // std::cout << testArcLengthSpline(json_paths) << std::endl;

    // std::cout << testIntegrator(json_paths) << std::endl;
    // std::cout << testLinModel(json_paths) << std::endl;

    // std::cout << testAlphaConstraint(json_paths) << std::endl;
    // std::cout << testTireForceConstraint(json_paths) << std::endl;
    // std::cout << testTrackConstraint(json_paths) << std::endl;

    // std::cout << testCost(json_paths) << std::endl;

    Integrator integrator = Integrator(jsonConfig["Ts"],json_paths);
    Plotting plotter = Plotting(jsonConfig["Ts"],json_paths);

    // Create ROS Node
    FastLapControlNode controlNode = FastLapControlNode();
    bool firstRun = true;

    // ROS INFO
    ROS_INFO_STREAM("FAST LAP CONTROL STARTED.");

    // Empty Objects
    MPC mpc(jsonConfig["n_sqp"],jsonConfig["n_reset"],jsonConfig["sqp_mixing"],jsonConfig["Ts"],json_paths);
    State x0;
    std::list<MPCReturn> log;
    Track track;
    TrackPos track_xy;

    // Set Ts
    ros::Rate rate(jsonConfig["Hz"]);

    int count = 0; // For debug

    while(ros::ok())
    {
        if (!comm)
        {
            if (count > jsonConfig["n_sim"])
            {
                controlNode.publishVel(0, 0);
                controlNode.fastlapready = false; // end fast lap
                double mean_time = 0.0;
                double max_time = 0.0;
                for(MPCReturn log_i : log)
                {
                    mean_time += log_i.time_total;
                    if(log_i.time_total > max_time)
                        max_time = log_i.time_total;
                }
                // std::cout << "mean nmpc time " << mean_time/double(jsonConfig["n_sim"]) << std::endl;
                std::cout << "mean nmpc time " << mean_time/count << std::endl;
                std::cout << "max nmpc time " << max_time << std::endl;
                plotter.plotSim(log,track_xy);
                return 0;
            }
            else
            {
                count++;
                std::cout << "count: " << count << std::endl;
            }
        } 
        if (skip)
        {
            controlNode.fastlapready = true; // Want start fast lap immediately
        }
        // Update 
        ros::spinOnce();

        // Fast Lap Control is ready
        if (firstRun && controlNode.getFastLapReady())
        {
            // ROS_INFO("RUNNING FIRST RUN");
            if (skip)
            {
                track = Track(json_paths.track_path);
                track_xy = track.getTrack();
                const double phi_0 = std::atan2(track_xy.Y(1) - track_xy.Y(0),track_xy.X(1) - track_xy.X(0));
                x0.set(track_xy.X(0),track_xy.Y(0),phi_0,0,0,0,0);
                // x0.set(0,0,0,0.05,0,0,0.05);
            }
            else
            {
                track = controlNode.generateTrack();
                track_xy = track.getTrack();
                x0 = controlNode.initialize();
            }
            
            mpc.setTrack(track_xy.X,track_xy.Y);

            ROS_INFO("Starting State:\nx:%lf\ny:%lf\ntheta:%lf\ns:%lf\nv:%lf\nw:%lf.", x0.X, x0.Y, x0.th, x0.s, x0.v, x0.w); 
            
            MPCReturn mpc_sol = mpc.runMPC(x0); // Updates s
            log.push_back(mpc_sol);
            x0 = integrator.simTimeStep(x0,mpc_sol.u0,jsonConfig["Ts"]); // Updates s and vs
            controlNode.publishVel(x0.v, x0.w); // Command desired velocity
            if(comm)
            {
                ros::spinOnce();
                x0 = controlNode.update(x0, mpc_sol.u0, jsonConfig["Ts"]); // Update all states from SLAMe
            }
            
            firstRun = false;
        }

        else if (controlNode.getFastLapReady())     
        {
            ros::spinOnce();
            ROS_INFO("States:\nx:%lf\ny:%lf\ntheta:%lf\ns:%lf\nv:%lf\nw:%lf.", x0.X, x0.Y, x0.th, x0.s, x0.v, x0.w);
            ROS_INFO("Constraint State:\nvL:%lf\nvR:%lf\n", (x0.v-0.5*x0.w*0.555), (x0.v+0.5*x0.w*0.555));
            tic();
            MPCReturn mpc_sol = mpc.runMPC(x0); // Updates s
            toc();
            log.push_back(mpc_sol);

            x0 = integrator.simTimeStep(x0,mpc_sol.u0,jsonConfig["Ts"]); // Update s and vs
            controlNode.publishVel(x0.v, x0.w); // Command desired velocity
            if(comm)
            {
                ros::spinOnce();
                x0 = controlNode.update(x0, mpc_sol.u0, jsonConfig["Ts"]); // Update all state with SLAM data 
            }
            
            // TODO: Lap count
        }

        if(comm)
        {
            rate.sleep(); // Wait until Ts
        }
    }
}


