/**
 * ROS Node for Fast Lap Control
 * 
 * SUBSCRIPTIONS
 * SLAM provides vehicle state
 * Planner provides map on transition, which has inner/outer cone position and centerline reference
 * SlowLapFollower provides final actuation output, so that transition is smooth
 * 
 * PUBLISHERS
 * transitionPublisher, to signify fast lap control is ready for transition and slow lap control can stop
 * actuationPublisher, to control the vehicle
 * 
 * Current Version by Kheng Yu Yeoh, contact @ khengyu_061192@hotmail.com
 * 
 */

#include "fastlapnode.h"
#include <External/Json/include/nlohmann/json.hpp> // To write JSON track
using json = nlohmann::json;

// To write csv file of map
#include <iostream>
#include <fstream>

// Constructor
FastLapControlNode::FastLapControlNode()
{
    this->mapready = false;
    this->fastlapready = false;

    this->slamSubscriber = nh.subscribe(ODOM_TOPIC, 1, &FastLapControlNode::slamCallback, this);
    this->mapSubscriber = nh.subscribe(MAP_TOPIC, 1, &FastLapControlNode::mapCallback, this);
    this->finalActuationSubscriber = nh.subscribe(CMDVEL_TOPIC, 1, &FastLapControlNode::finalActuationCallback, this);
    this->velocityPublisher = nh.advertise<geometry_msgs::Twist>(CMDVEL_TOPIC, 1);
    this->transitionPublisher = nh.advertise<mur_common::transition_msg>(TRANSITION_TOPIC, 1);
}

// Update Husky object's pose when receive /Huskysim/Pose/ msgs
// topic: ODOM_TOPIC
// msg type: geometry_msgs::Pose.msg
// args: [double] x y theta linear_velocity angular_velocity
void FastLapControlNode::slamCallback(const nav_msgs::Odometry& msg)
{
    this->x = msg.pose.pose.position.x;
    this->y = msg.pose.pose.position.y;

    double q_x = msg.pose.pose.orientation.x;
    double q_y = msg.pose.pose.orientation.y;
    double q_z = msg.pose.pose.orientation.z;
    double q_w = msg.pose.pose.orientation.w;
    
    // this->th = Quart2EulerYaw(q_x, q_y, q_z, q_w);
    tf::Quaternion q(q_x, q_y, q_z, q_w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    this->th = yaw;

    this->v = msg.twist.twist.linear.x;
    this->w = msg.twist.twist.angular.z;
}

// Obtain track boundary from planner
// topic: /mur/planner/map
// msg: mur_common/map_msg
void FastLapControlNode::mapCallback(const mur_common::map_msg& msg)
{
    if (msg.mapready && !this->mapready) // Only update once when mapping is done
    {
        std::vector<double> x_o(msg.x_o.begin(), msg.x_o.end());
        std::vector<double> y_o(msg.y_o.begin(), msg.y_o.end());
        std::vector<double> x_i(msg.x_i.begin(), msg.x_i.end());
        std::vector<double> y_i(msg.y_i.begin(), msg.y_i.end());
        std::vector<double> x(msg.x.begin(), msg.x.end());
        std::vector<double> y(msg.y.begin(), msg.y.end());
        this->x_outer = x_o;
        this->y_outer = y_o;
        this->x_inner = x_i;
        this->y_inner = y_i;
        this->x_centre = x;
        this->y_centre = y;

        this->mapready = msg.mapready;

        if (this->mapready)
        {
            ROS_INFO_STREAM("Obtained MAP!, now MAPREADY.");
        }
    }
}

// Obtain final actuation output from slow lap, so it can transition smoothly
// Also publishes fastlapready, so slow lap follower can stop publishing actuation commands
// topic: /mur/control/actuation
// msg: geometry_msgs/Twist
void FastLapControlNode::finalActuationCallback(const geometry_msgs::Twist& msg)
{
    // Only update once when mapping is done
    if (this->mapready && !this->fastlapready) 
    {
        ROS_INFO_STREAM("MAP READY and Receiving final actuation.");
        this->v = msg.linear.x;
        this->w = msg.angular.z;
        this->fastlapready = true;

        if (this->fastlapready)
        {
            ROS_INFO_STREAM("Obtained FINAL ACTUATION!, now FASTLAPREADY.");
            ROS_INFO_STREAM("Final linear velocity command is " << v);
            ROS_INFO_STREAM("Final angular velocity command is " << w);
        }
    }

    // Initialize msg and publish
    mur_common::transition_msg transition_msg;
    transition_msg.fastlapready = this->fastlapready;
    transitionPublisher.publish(transition_msg);
}

// Publisher function to publish desired velocity
void FastLapControlNode::publishVel(double lin_vel, double ang_vel)
{
    // Initialize msg
    geometry_msgs::Twist vel_msg;
    vel_msg.linear.x = lin_vel;
    vel_msg.angular.z = ang_vel;

    ROS_INFO("PUBLISHING VELOCITY: %lf, STEER: %lf\n", lin_vel, ang_vel);

    velocityPublisher.publish(vel_msg);
}

// Get Functions
bool FastLapControlNode::getFastLapReady(){
    return this->fastlapready;
}

// Track generation function, using obtained map inputs
// returns: the mapped track of type Track
mpcc::Track FastLapControlNode::generateTrack() 
{
    mpcc::Track track = mpcc::Track(this->x_outer, this->y_outer, 
                                    this->x_inner, this->y_inner, 
                                    this->x_centre, this->y_centre);

    // // Write current track into csv
    // std::ofstream trackfile;
    // trackfile.open("/workspace/track.csv");
    // trackfile << "x_o,y_o,x_i,y_i,x,y,left as outer and right as inner for now.\n";
    // for (int i = 0; i < c_count; i++)
    // {
    //     if (i > o_count)
    //     {
    //         trackfile << "0,0,";
    //     }
    //     else
    //     {
    //         trackfile << this->x_outer[i] << "," << this->y_outer[i] << ",";
    //     }

    //     if (i > i_count)
    //     {
    //         trackfile << "0,0,";
    //     }
    //     else
    //     {
    //         trackfile << this->x_inner[i] << "," << this->y_inner[i] << ",";
    //     }

    //     trackfile << this->x_centre[i] << "," << this->y_centre[i] << ",\n";
    // }
    
    // trackfile.close();

    // // Write current track as json
    // json json_track;
    // json_track["X_o"] = x_outer;
    // json_track["X_i"] = x_inner;
    // json_track["Y_o"] = y_outer;
    // json_track["Y_i"] = y_inner;
    // json_track["X"] = x_centre;
    // json_track["Y"] = y_centre;

    // std::ofstream file("/workspace/track.json");
    // file << json_track;

    return track;
}


// State initialization function
// returns: initial state of vehicle after transition from slow lap
mpcc::State FastLapControlNode::initialize()
{
    mpcc::State x = {
        this->x,
        this->y,
        this->th,
        this->v,
        this->w,
        0.0,  // s
        this->v // vs
    };

    return x;
}

// Update function
// input: current vehicle state
// returns: state with updated SLAM outputs
mpcc::State FastLapControlNode::update(const mpcc::State& x0, const mpcc::Input& u0, double Ts)
{
    mpcc::State x_new;
    x_new.X = this->x;
    x_new.Y = this->y;
    x_new.th = this->th;
    x_new.v = this->v;
    x_new.w = this->w;
    // Copy over from simTimeStep
    x_new.s = x0.s;
    x_new.vs = x0.vs;

    return x_new;
}

// Returns 1 if positive, 0 if 0, -1 if negative
// For use in Quarternion conversion to Radians
int sign(double x)
{
    if (x>0)
    {
        return 1;
    }
    else if (x==0)
    {
        return 0;
    }
    else
    {
        return -1;
    }
}

// Converts from Quartenion to Euler Yaw
// From https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
double Quart2EulerYaw(double q_x, double q_y, double q_z, double q_w)
{
    double siny_cosp = 2.0 * (q_w * q_z + q_x * q_y);
    double cosy_cosp = 1.0 - (2.0 * (q_y * q_y + q_z * q_z));
    return std::atan2(siny_cosp, cosy_cosp);
}