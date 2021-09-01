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
#include <iomanip>

#define STATROW 13
#define BARWIDTH 30
#define NUMLAP 10

// Time testing
std::stack<clock_t> tictoc_stack;

void tic() {
    tictoc_stack.push(clock());
}

double toc() {
    double t = ((double)(clock() - tictoc_stack.top())) / CLOCKS_PER_SEC;
    // std::cout << "Time elapsed: "
    //           << t
    //           << "s, Hz: "
    //           << 1.0/t
    //           << std::endl;
    tictoc_stack.pop();
    return t;
}

// Print Statistics
void printStatistics(mpcc::State x0, Eigen::Vector4d command, mpcc::ArcLengthSpline track_, 
    double lap_count, double t, Eigen::Vector2d error_count, Eigen::Vector2d wheel_constraint)
{
    // Decimals
    std::cout << std::fixed; 
    std::cout << std::setprecision(5);

    std::cout << "States:" << std::endl;
    std::cout << "x: " << x0.X << std::endl;
    std::cout << "y: " << x0.Y << std::endl;
    std::cout << "theta: " << x0.th << std::endl;
    std::cout << "s: " << x0.s << std::endl;
    std::cout << "v: " << command(0) << std::endl;
    std::cout << "w: " << command(1) << std::endl;
    std::cout << "vL: " << command(2) << std::endl;
    std::cout << "vR: " << command(3) << std::endl;
    std::cout << "Controller Status: " << std::endl;
    std::cout << "Solve time: " << t << "s, Hz: " << 1.0/t << std::endl;
    std::cout << "exitflag1 count: " << int(error_count(0)) << ", exitflag2 count: " << int(error_count(1)) << std::endl;
    std::cout << "Constraint Violation count: " << int(wheel_constraint(0)) << ", Max Violation: " << wheel_constraint(1) << std::endl;

    std::cout << "lap " << int(lap_count) << " progress: [";
    int pos = BARWIDTH * x0.s/track_.getLength();
    for (int i = 0; i < BARWIDTH; ++i) {
        if (i < pos) std::cout << "=";
        else if (i == pos) std::cout << ">";
        else std::cout << " ";
    }
    std::cout << "] " << int(x0.s/track_.getLength() * 100.0) << "%   ";
    
    // Set stationary
    std::cout << "\r";
    for (int i = 0; i < STATROW; i++)
        std::cout << "\033[F";
    std::cout.flush();
}


// Main
int main(int argc, char **argv) {
    // Initialize ROS node
    ros::init(argc, argv, "husky_mpcc");

    // Get parameters from CLI
    bool comm = atoi(argv[1]);
    bool skip = atoi(argv[2]);

    using namespace mpcc;
    std::ifstream iConfig("/home/khengyu/catkin_ws/src/HuskyControl/husky_mpcc/src/Params/config.json");
    json jsonConfig;
    iConfig >> jsonConfig;

    ROS_INFO_STREAM("FAST LAP CONTROL - MPCC - Initialized configuration.");

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
    FastLapControlNode controlNode = FastLapControlNode(json_paths);
    bool firstRun = true;

    // ROS INFO
    ROS_INFO_STREAM("FAST LAP CONTROL - MPCC - Initializing MPC.");

    // Empty Objects
    MPC mpc(jsonConfig["n_sqp"],jsonConfig["n_reset"],jsonConfig["sqp_mixing"],jsonConfig["Ts"],json_paths);
    State x0;
    std::list<MPCReturn> log;
    Track track;
    TrackPos track_xy;
    ArcLengthSpline track_; // parameterized track

    // Set Ts
    ros::Rate rate(jsonConfig["Hz"]);

    // Lap count
    double cur_s = 0;
    int lap_count = 0;

    // Debug and stats
    int count = 0; // For debug and simplesim
    Eigen::Vector2d wheel_violate; // Wheel Constraint Testing, 0 for count, 1 for max
    double t = 0; // solve time and Hz
    Eigen::Vector2d error_count; // count exit error 1 and 2

    ROS_INFO_STREAM("FAST LAP CONTROL STARTED.");

    while(ros::ok())
    {
        // Lap Count
        if (lap_count == NUMLAP)
        {
            if (x0.s > 1) // Cross a little bit then stop
            {
                controlNode.publishVel(0, 0); // Command desired velocity
                return 0; // Finish race
            }
        }

        if (!comm) // Simple sim test without Gazebo/RVIZ
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
                for (int i = 0; i < STATROW+1; i++)
                    std::cout << std::endl;
                std::cout << "mean nmpc time " << mean_time/count << std::endl;
                std::cout << "max nmpc time " << max_time << std::endl;
                std::cout << "wheel violation count " << wheel_violate(0) << std::endl;
                std::cout << "max wheel violation " << wheel_violate(1) << std::endl;
                std::cout << "exitflag 1 count " << error_count(0) << std::endl;
                std::cout << "exitflag 2 count " << error_count(1) << std::endl;
                plotter.plotSim(log,track_xy, track_);
                return 0;
            }
            else
            {
                count++;
                // std::cout << "count: " << count << std::endl;
            }
        }
        if (skip) // Want start fast lap immediately
        {
            controlNode.fastlapready = true; 
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
            
            track_ = mpc.setTrack(track_xy.X,track_xy.Y);

            ROS_INFO("Fast Lap Starting State:\nx:%lf\ny:%lf\ntheta:%lf\ns:%lf\nv:%lf\nw:%lf.", x0.X, x0.Y, x0.th, x0.s, x0.v, x0.w); 
            
            firstRun = false;
            count = 0;
        }

        else if (controlNode.getFastLapReady())     
        {
            // ROS_INFO("States:\nx:%lf\ny:%lf\ntheta:%lf\ns:%lf\nv:%lf\nw:%lf.", x0.X, x0.Y, x0.th, x0.s, x0.v, x0.w);
            // ROS_INFO("Constraint State:\nvL:%lf\nvR:%lf\n", (x0.v-0.5*x0.w*0.555), (x0.v+0.5*x0.w*0.555));
            tic();
            MPCReturn mpc_sol = mpc.runMPC(x0, error_count); // Updates s
            t = toc();
            log.push_back(mpc_sol);

            x0 = integrator.simTimeStep(x0,mpc_sol.u0,jsonConfig["Ts"]); // Update s and vs
            controlNode.publishVel(x0.v, x0.w); // Command desired velocity

            // Save command
            double vL = x0.v-0.5*x0.w*0.555;
            double vR = x0.v+0.5*x0.w*0.555;
            Eigen::Vector4d command(x0.v, x0.w, vL, vR);

            // Check if command satisfy wheel constraints
            if (abs(vL) > 1.001 || abs(vR) > 1.001)
            {
                wheel_violate(0)++;
                wheel_violate(1) = abs(vL) > wheel_violate(1) ? abs(vL) : wheel_violate(1);
                wheel_violate(1) = abs(vR) > wheel_violate(1) ? abs(vR) : wheel_violate(1);
                // ROS_INFO_STREAM("VIOLATED WHEEL VEL CONSTRAINT: " << violate);
            }
            
            // Update States from SLAM
            if(comm)
            {
                ros::spinOnce();
                x0 = controlNode.update(x0, mpc_sol.u0, jsonConfig["Ts"]); // Update all state with SLAM data 
            }

            // Print stats
            if (++count > (double)jsonConfig["Hz"]) // after 1s for clean print
            {
                printStatistics(x0, command, track_, lap_count, t, error_count, wheel_violate);
            }
            
            controlNode.publishRVIZ(mpc_sol.mpc_horizon, track_); // Publish RVIZ

            if(x0.s < cur_s) // Finished a lap, progress resets
            {
                lap_count++;
                cur_s = 0;
            }
            else
                cur_s = x0.s; // Save new progress
        }

        if(comm)
        {
            rate.sleep(); // Wait until Ts
        }
    }
}


