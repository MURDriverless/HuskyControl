/**
 * ROS Node for Husky Control through Terminal
 * 
 * SUBSCRIPTIONS
 * /odometry/filtered which is from Husky's own EKF-SLAM mapping and localisation
 * 
 * PUBLISHERS
 * /husky_velocity_controller/cmd_vel to control the husky
 * 
 * Current Version by Kheng Yu Yeoh, contact @ khengyu_061192@hotmail.com
 * Repo @ https://github.com/MURDriverless/HuskyControl
 */

#ifndef TERMINALCTRL_H
#define TERMINALCTRL_H

#include <ros/ros.h> // Must include for all ROS C++
#include <geometry_msgs/Twist.h> // To control linear and angular velocity of Husky
#include <nav_msgs/Odometry.h> // Msg from /odometry/filtered
#include <tf/tf.h> // For Convertion from Quartenion to Euler
#include <cmath>
#include <sstream>

#define CMDVEL_TOPIC "/husky_velocity_controller/cmd_vel"
#define ODOM_TOPIC "/odometry/filtered"

#define MAXSPD 1.0 // Linear Velocity m/s
#define MAXROT 30*M_PI/180 // Rotation Velociy rad/s
#define HZ 50 // Husky can work at 50Hz max
#define ACCEL_T 1 // Time (s) to reach max vel

// Husky Class
class TerminalControlHusky{
    private:
    // Variables for Current Pose
    double x;
    double y;
    double phi;
    double linvel;
    double angvel;
    double max_v;
    double max_w;
    double KP_dist;
    double KP_angle;
    // ROS
    ros::NodeHandle n; // Create its specific node handler
    ros::Publisher velocityPublisher;
    ros::Subscriber poseSubscriber;

    public:
    // Constructor
    TerminalControlHusky(double max_v, double max_w, double KP_dist, double KP_angle);

    /* Husky Class Functions */
    double getX();
    double getY();
    double getPhi(); // in radians
    double getLinVel();
    double getAngVel();

    // Update Husky's pose when receive from /odometry/filtered
    // topic: ODOM_TOPIC
    // msg type: nav_msgs::Odometry
    void updatePose(const nav_msgs::Odometry& msg);

    // Publish velocity msg
    // topic: CMDVEL_TOPIC
    // msg type: geometry_msgs::Twist
    void publishVel(double linvel, double angvel);
    
    // Move the robot for a certain distance at a certain speed, rotates before moving straight
    void move(double dist, double speed, bool isForward, double angle);
    
    // Move the robot to a specified location, moving while rotating at the same time
    void smoothmove(double dest_x, double dest_y, double error);

    // Converts from Quartenion to Euler Yaw
    // From https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
    // double Quart2EulerYaw(double q_x, double q_y, double q_z, double q_w);
};

#endif //TERMINALCTRL_H