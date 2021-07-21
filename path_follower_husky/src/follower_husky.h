#ifndef HUSKY_PATH_FOLLOWER_H
#define HUSKY_PATH_FOLLOWER_H

#include <ros/ros.h> // Must include for all ROS C++
#include <geometry_msgs/Twist.h> // To control linear and angular velocity of Husky
#include <nav_msgs/Odometry.h> // Msg from /odometry/filtered
#include <tf/tf.h> // For Convertion from Quartenion to Euler
#include "mur_common/path_msg.h"
#include <cmath>
#include <sstream>
#include <string>
#include <iostream>
#include <algorithm>
#include <vector>
#include "path_point.h"


#define PI 3.14159265359
#define LENGTH 2.95 //length of vehicle (front to rear wheel)  /////
#define G  9.81 //gravity
#define MAX_ACC 11.772 //1.2*G
#define MAX_DECEL -17.658 //-1.8*Gg
#define MAX_STEER 0.8 //
#define STEPSIZE 0.2 //spline step size   /////
#define DT 0.05
//PID gains:
#define KP 1   /////
#define KI 1   /////
#define KD  1   /////
//pure pursuit gains
#define K 0.1
#define LFV  0.1  ///// look forward gain
#define LFC  3.5    ///// look ahead distance
#define V_CONST 1.0 //constant velocity 1m/s (for now)
#define MAX_V 1.0


#define ODOM_TOPIC "/odometry/filtered" //"/mur/slam/Odom"
#define CMDVEL_TOPIC "/husky_velocity_controller/cmd_vel"
#define PATH_TOPIC "/mur/planner/path"
// #define CONTROL_TOPIC "/mur/control/actuation"
// #define ACCEL_TOPIC "/mur/accel_desired"
// #define STEER_TOPIC "/mur/control_desired"



class HuskyFollower
{
public:
    HuskyFollower(ros::NodeHandle n, double max_v, double max_w, double KP_dist, double KP_angle);
    void spin();

private:

    ros::NodeHandle nh;
    ros::Subscriber sub_odom;
    ros::Subscriber sub_path;
    ros::Publisher pub_control;
    ros::Publisher pub_accel;
    ros::Publisher pub_steer;
    ros::Publisher pub_target;

    float max_v;
    float max_w;
    float KP_dist;
    float KP_angle;

    float car_x;
    float car_y;
    float car_lin_v;
    float car_ang_v;
    float car_yaw;
    bool odom_msg_received = false;
    bool new_centre_points = false;
    float rearX;
    float rearY;

    std::vector<float> path_x;
    std::vector<float> path_y;
    bool path_msg_received = false;

    std::vector<PathPoint> centre_points; //centre line points of race tack
    std::vector<PathPoint> centre_splined;
    int index = -1; //centre splined index
    int oldIndex = -1;
    std::vector<float> xp;
    std::vector<float> yp;
    std::vector<float> T;

    float lin_velocity = 0;
    float ang_velocity = 0;
    PathPoint p;

    void waitForMsgs();
    int launchSubscribers();
    int launchPublishers();
    void odomCallback(const nav_msgs::Odometry &msg);
    void pathCallback(const mur_common::path_msg &msg);
    void publishCtrl();
    void smoothmove();

    //void accelerationControl();
    void steeringControl();
    //void updateRearPos();
    float getDistFromCar(PathPoint);
    float getAngleFromCar(PathPoint);
    void clearVars();
    PathPoint getGoalPoint();

    float getSign(float&);
};



#endif