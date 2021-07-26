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

#include "terminalcontrol.h"

// Husky Class Initiation
TerminalControlHusky::TerminalControlHusky(double max_v, double max_w, double KP_dist, double KP_angle)
{
    this->max_v = max_v < MAXSPD ? max_v : MAXSPD;
    this->max_w = max_w*M_PI/180 < MAXROT ? max_w*M_PI/180 : MAXROT;
    this->KP_dist = KP_dist;
    this->KP_angle = KP_angle;
    this->velocityPublisher = n.advertise<geometry_msgs::Twist>(CMDVEL_TOPIC, 1);
    this->poseSubscriber = n.subscribe(ODOM_TOPIC, 1, &TerminalControlHusky::updatePose, this);
}

/* Husky Class Get Functions */
double TerminalControlHusky::getX()
{
    return this->x;
}
double TerminalControlHusky::getY()
{
    return this->y;
}
double TerminalControlHusky::getPhi() // in radians
{
    return this->phi;
}
double TerminalControlHusky::getLinVel()
{
    return this->linvel;
}
double TerminalControlHusky::getAngVel()
{
    return this->angvel;
}

// Update Husky object's pose when receive /Huskysim/Pose/ msgs
// topic: ODOM_TOPIC
// msg type: geometry_msgs::Pose.msg
// args: [double] x y theta linear_velocity angular_velocity
void TerminalControlHusky::updatePose(const nav_msgs::Odometry& msg)
{
    this->x = msg.pose.pose.position.x;
    this->y = msg.pose.pose.position.y;
    
    double q_x = msg.pose.pose.orientation.x;
    double q_y = msg.pose.pose.orientation.y;
    double q_z = msg.pose.pose.orientation.z;
    double q_w = msg.pose.pose.orientation.w;

    // this->phi = Quart2EulerYaw(q_x, q_y, q_z, q_w); // rads

    this->linvel = msg.twist.twist.linear.x;
    this->angvel = msg.twist.twist.angular.z;

    tf::Quaternion q(q_x, q_y, q_z, q_w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    this->phi = yaw;

    // std::cout << "converted phi:" << this->phi << std::endl;
    // std::cout << "tf phi: " << yaw << std::endl;

    // std::cout << "Updating pose, "<< "\n" << "x: " << msg->x << "\n";
    // std::cout << "y: " << msg->y << "\n";
    // std::cout << "Current angle: " << this->angle << "\n";
    // // std::cout << "linvel: " << msg->linear_velocity << "\n";
    // // std::cout << "angvel: " << msg->angular_velocity << "\n";
}

// Publisher function to publish desired velocity
void TerminalControlHusky::publishVel(double lin_vel, double ang_vel)
{
    // Initialize msg
    geometry_msgs::Twist vel_msg;
    vel_msg.linear.x = lin_vel;
    vel_msg.angular.z = ang_vel;

    this->velocityPublisher.publish(vel_msg);
}

// Move the robot for a certain distance at a certain speed
// topic: CMDVEL_TOPIC
// msg type: geometry_msgs/Twist
// args: linear angular: [x,y,z] [x,y,z]
void TerminalControlHusky::move(double dist, double speed, bool isForward, double angle)
{
    // Update position
    ros::spinOnce();

    ros::Rate rate(HZ);

    // Initialize msg
    geometry_msgs::Twist vel_msg;

    // Time based implementation
    double initialtime, traveltime, finaltime;

    // Check if to turn
    if(angle!= 0)
    {
        vel_msg.angular.z = angle > 0 ? this->max_w : -this->max_w;

        initialtime = ros::Time::now().toSec();
        traveltime = abs(angle)/this->max_w;
        finaltime = initialtime + traveltime;

        ROS_INFO("Begin alining direction!");
    
        // Begin turning until specified angle
        velocityPublisher.publish(vel_msg);
        while(ros::Time::now().toSec() <= finaltime)
        {
            velocityPublisher.publish(vel_msg);
            rate.sleep();
            // ROS_INFO("this->angle: %lf \n", this->angle);
            // ROS_INFO("angle: %lf \n", angle);
            // ROS_INFO("diff: %lf \n", this->angle - angle);
        }

        vel_msg.angular.z = 0;
        velocityPublisher.publish(vel_msg); // Stop turning

        ROS_INFO("Finished aligning!");
    }

    // Set forward or backwards movement
    vel_msg.linear.x = isForward ? abs(speed) : -abs(speed);

    // Stop when reaching the distance specified
    initialtime = ros::Time::now().toSec();
    traveltime = dist/speed;
    finaltime = initialtime + traveltime;
    while (ros::Time::now().toSec() < finaltime)
    {
        velocityPublisher.publish(vel_msg);
        rate.sleep();
        // ros::spinOnce();
    }
    // Reached specified distance
    vel_msg.linear.x = 0;
    
    // Publish again
    velocityPublisher.publish(vel_msg);
    
    ROS_INFO("Husky reached its destination!\n");
}
    
// Move the robot to a specified location, moving while rotating at the same time
void TerminalControlHusky::smoothmove(double dest_x, double dest_y, double error)
{
    ros::Rate rate(HZ);

    // Initialize msg
    geometry_msgs::Twist vel_msg;

    // Initialize Variables
    double move_x, move_y, move_dist, dest_angle, move_angle;
    double count = 0; // To have smooth acceleration at beginning

    // Get initial
    move_x = dest_x - this->x;
    move_y = dest_y - this->y;
    move_dist = sqrt((move_x*move_x) + (move_y*move_y));

    // Have not reached
    while(move_dist > error)
    {
        // Set forward velocity with respect to distance, limit to max_v
        vel_msg.linear.x = (KP_dist * move_dist) > this->max_v ? this->max_v : KP_dist * move_dist;

        // Slowly increase velocity if desired max velocity too early on
        count++;
        if (count < ACCEL_T*HZ && (vel_msg.linear.x == this->max_v))
        {
            vel_msg.linear.x *= count/(ACCEL_T*HZ);
        }
        
        std::cout << "linvel set as:" << vel_msg.linear.x << std::endl;

        // Compute angle to turn
        dest_angle = std::atan2(move_y,move_x);
        move_angle = dest_angle - this->phi;
        
        // Have to fix for facing left, as it fluctuates between PI and -PI
        // This also lets it turn more efficiently (turns the shorter distance to reach dest_angle)
        while(abs(move_angle) > M_PI)
        {
            if(move_angle > 0) // Should turn right instead
            {
                move_angle -= 2*M_PI;
            }
            else // Should turn left instead
            {
                move_angle += 2*M_PI;
            }
        }

        // Scale angle error as velocity to turn (Proportional Controller), limit speed of rotation as this->max_w
        vel_msg.angular.z = (KP_angle * move_angle) >= this->max_w ? this->max_w : KP_angle * move_angle;
        
        std::cout << "angvel set as:" << vel_msg.angular.z << std::endl;

        velocityPublisher.publish(vel_msg);

        // Update position
        ros::spinOnce();

        // Get new distance
        move_x = dest_x - this->x;
        move_y = dest_y - this->y;
        move_dist = sqrt((move_x*move_x) + (move_y*move_y));

        rate.sleep();
    }

    // Should reach destination by now
    vel_msg.linear.x = 0;
    vel_msg.angular.z = 0;
    velocityPublisher.publish(vel_msg);

    ROS_INFO("x: %lf, y:%lf \n", this->x, this->y);    
}

// // Converts from Quartenion to Euler Yaw
// // From https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
// double TerminalControlHusky::Quart2EulerYaw(double q_x, double q_y, double q_z, double q_w)
// {
//     double siny_cosp = 2.0 * (q_w * q_z + q_x * q_y);
//     double cosy_cosp = 1.0 - (2.0 * (q_y * q_y + q_z * q_z));
//     return std::atan2(siny_cosp, cosy_cosp);
// }