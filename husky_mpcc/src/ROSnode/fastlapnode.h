/**
 * Header file for the FastLapControl ROS node,
 * Subscribes to /mur/slam/Odom topic from SLAM
 * Subscribes to /mur/planner/map topic from planner_exploratory
 * Subscribes to /mur/control/actuation from slow lap control
 * Publishes /mur/control/transition to slow lap control
 * Publishes /mur/control/actuation to actuation
 * 
 * Current Version by Kheng Yu Yeoh, contact @ khengyu_061192@hotmail.com
 * 
 */

#ifndef FASTLAPCONTROLNODE_H
#define FASTLAPCONTROLNODE_H

// General & MPCC includes
#include <cmath>
#include "config.h"
#include "types.h"
#include "Params/track.h"
#include "Params/params.h"
#include "Spline/arc_length_spline.h"
#include "MPC/mpc.h"

// ROS includes
#include <ros/ros.h>
#include <nav_msgs/Odometry.h> // Msg for /mur/slam/Odom topic
#include <tf/tf.h> // For Convertion from Quartenion to Euler
#include "mur_common/map_msg.h" // Msg for /mur/planner/map topic
#include "mur_common/transition_msg.h" // Msg for /mur/control/transition topic
#include <visualization_msgs/Marker.h> // for RVIZ markers

#define ODOM_TOPIC "/control_odom"
#define MAP_TOPIC "/mur/planner/map"
#define CMDVEL_TOPIC "/mur2022internal/cmd_vel"
#define TRANSITION_TOPIC "/mur/control/transition"
#define RVIZ_TOPIC "/mpcc_RVIZ_topic"
#define RVIZ_FRAME "odom" // name of fixed frame in RVIZ

class FastLapControlNode {
    private:
    // Determine if obtained map and whether controller is ready to run
    bool mapready;
    // bool pathready;
    // bool fastlapready;

    // Vehicle States From SLAM
    double x;
    double y;
    double th; // Convert from Quartenion to Euler
    double v;
    double w;

    // Track parameter from Planner TODO: Inner Outer Cone Position in Planner
    std::vector<double> x_outer;
    std::vector<double> y_outer;
    std::vector<double> x_inner;
    std::vector<double> y_inner;
    std::vector<double> x_centre;
    std::vector<double> y_centre;

    // ROS stuff
    ros::NodeHandle nh; // Create its specific node handler
    ros::Subscriber slamSubscriber;
    ros::Subscriber mapSubscriber; 
    ros::Subscriber finalActuationSubscriber; // To obtain final actuation output of slow lap before transition
    ros::Publisher velocityPublisher;
    ros::Publisher transitionPublisher; // Publish so that slow lap can stop publishing actuation
    ros::Publisher RVIZPublisher; // Visualize horizon and track boundary

    public:    
    bool fastlapready; // Skip to fast lap
    
    // Constructor
    FastLapControlNode(const mpcc::PathToJson &path);

    // Callback Function
    void slamCallback(const nav_msgs::Odometry& msg);
    void mapCallback(const mur_common::map_msg& msg);
    void finalActuationCallback(const geometry_msgs::Twist& msg);
    // void pathCallback();
    
    // Publish Function
    void publishVel(double lin_vel, double ang_vel);
    void publishActuation(double accel_D, double steering_angle);

    // Get Functions
    bool getFastLapReady();

    // Function for Mapped Track generation
    mpcc::Track generateTrack();

    // Functions for vehicle state
    mpcc::State initialize();
    mpcc::State update(const mpcc::State& x0, const mpcc::Input& u0, double Ts);

    // RVIZ Functions
    void publishRVIZ(const std::array<mpcc::OptVariables,mpcc::N+1> mpc_horizon, const mpcc::ArcLengthSpline &track);

    mpcc::Param param_;
};

// Returns 1 if positive, 0 if 0, -1 if negative
// For use in Quarternion conversion to Radians
int sign(double x);

// Converts from Quartenion to Euler Yaw
// From https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
double Quart2EulerYaw(double q_x, double q_y, double q_z, double q_w);

#endif