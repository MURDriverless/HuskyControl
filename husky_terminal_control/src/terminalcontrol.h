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
#include <vector>
#include "spline.h"


#define CMDVEL_TOPIC "/husky_velocity_controller/cmd_vel"
#define ODOM_TOPIC "/odometry/filtered"

#define MAXSPD 1.0 // Linear Velocity m/s
#define MAXROT 30*M_PI/180 // Rotation Velociy rad/s
#define HZ 20 // Husky can work at 50Hz max
#define ACCEL_T 1 // Time (s) to reach max vel

// Husky Class
class TerminalControlHusky{
    private:
    // Variables for Current Pose
    float x;
    float y;
    float phi;
    float linvel;
    float angvel;
    float max_v;
    float max_w;
    float KP_dist;
    float KP_angle;
    // ROS
    ros::NodeHandle n; // Create its specific node handler
    ros::Publisher velocityPublisher;
    ros::Subscriber poseSubscriber;

    public:
    // Constructor
    TerminalControlHusky(float max_v, float max_w, float KP_dist, float KP_angle);

    /* Husky Class Functions */
    float getX();
    float getY();
    float getPhi(); // in radians
    float getLinVel();
    float getAngVel();

    // Update Husky's pose when receive from /odometry/filtered
    // topic: ODOM_TOPIC
    // msg type: nav_msgs::Odometry
    void updatePose(const nav_msgs::Odometry& msg);

    // Publish velocity msg
    // topic: CMDVEL_TOPIC
    // msg type: geometry_msgs::Twist
    void publishVel(float linvel, float angvel);
    
    // Move the robot for a certain distance at a certain speed, rotates before moving straight
    void move(float dist, float speed, bool isForward, float angle);
    
    // Move the robot to a specified location, moving while rotating at the same time
    void smoothmove(float dest_x, float dest_y, float error);


    // path follower
    float lin_velocity = 0;
    float ang_velocity = 0;
    std::vector<float> centre_pointsX;       // centre line points of race tack, from path planner
    std::vector<float> centre_pointsY;
    std::vector<float> centre_splinedX;
    std::vector<float> centre_splinedY;
    std::vector<float> T;
    float currentGoalPointX;
    float currentGoalPointY;
    int index = 1;
    bool endOfPath = false;
    float Lfc = 2.5; //look ahead distance
    void pathFollower(std::vector<float>&,std::vector<float>&);
    float getDistFromCar(float&,float&);
    float getAngleFromCar(float&,float&);
    float getSign(float&);
    void getGoalPoint();
    void generateSplines();
    void clearVecs();


};

#endif //TERMINALCTRL_H