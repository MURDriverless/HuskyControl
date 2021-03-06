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

// Fix angle issues when facing left/west as it fluctuates between pi/-pi
void TerminalControlHusky::unwrapAngle(double& angle)
{
    while(abs(angle) > M_PI)
    {
        if(angle > 0) // Should turn right instead
        {
            angle -= 2*M_PI;
        }
        else // Should turn left instead
        {
            angle += 2*M_PI;
        }
    }
}

// Update Husky object's pose when receive /Huskysim/Pose/ msgs
// topic: ODOM_TOPIC
// msg type: geometry_msgs::Pose.msg
// args: [double] x y theta linear_velocity angular_velocity
void TerminalControlHusky::updatePose(const nav_msgs::Odometry& msg)
{
    double q_x = msg.pose.pose.orientation.x;
    double q_y = msg.pose.pose.orientation.y;
    double q_z = msg.pose.pose.orientation.z;
    double q_w = msg.pose.pose.orientation.w;

    if (reinitialise)
    {
        initX = msg.pose.pose.position.x;
        initY = msg.pose.pose.position.y;

        tf::Quaternion q(q_x, q_y, q_z, q_w);
        tf::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);

        initPhi = yaw;
        reinitialise = false;
    }

    this->x = msg.pose.pose.position.x - initX;
    this->y = msg.pose.pose.position.y - initY;

    // this->phi = Quart2EulerYaw(q_x, q_y, q_z, q_w); // rads

    this->linvel = msg.twist.twist.linear.x;
    this->angvel = msg.twist.twist.angular.z;

    tf::Quaternion q(q_x, q_y, q_z, q_w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    this->phi = yaw - initPhi;

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
void TerminalControlHusky::smoothMove(double dest_x, double dest_y, double error)
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
    move_dist = std::sqrt((move_x*move_x) + (move_y*move_y));

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
        
        // std::cout << "linvel set as:" << vel_msg.linear.x << std::endl;

        // Compute angle to turn
        dest_angle = std::atan2(move_y,move_x);
        move_angle = dest_angle - this->phi;
        
        // Have to fix for facing left, as it fluctuates between PI and -PI
        // This also lets it turn more efficiently (turns the shorter distance to reach dest_angle)
        unwrapAngle(move_angle);

        // Scale angle error as velocity to turn (Proportional Controller), limit speed of rotation as this->max_w
        vel_msg.angular.z = abs(KP_angle * move_angle) >= this->max_w ? this->max_w : KP_angle * move_angle;
        
        // std::cout << "angvel set as:" << vel_msg.angular.z << std::endl;

        velocityPublisher.publish(vel_msg);

        // Update position
        ros::spinOnce();
        std::cout << "Husky current position: ("<<getX()<<", "<<getY()<<")"<<std::endl;

        // Get new distance
        move_x = dest_x - this->x;
        move_y = dest_y - this->y;
        move_dist = std::sqrt((move_x*move_x) + (move_y*move_y));

        rate.sleep();
    }

    // Should reach destination by now
    publishVel(0,0);

    ROS_INFO("x: %lf, y:%lf \n", this->x, this->y);    
}

// To test rotation using time based approach at constant velocity
void TerminalControlHusky::testRotTime()
{
    ros::Rate rate(HZ);
    ros::spinOnce();
    ROS_INFO("Current phi: %lf\n", this->phi);

    std::cout << "Rotating in place for 12s at 30deg/s. Should do full rotation." << std::endl;
    double finaltime = ros::Time::now().toSec() + 12;

    while (ros::Time::now().toSec() < finaltime)
    {
        // Update Position
        ros::spinOnce();

        publishVel(0, M_PI/18.0*3.0); // 30 deg per s
        rate.sleep(); // 50Hz max
    }
    publishVel(0, 0);
    ros::spinOnce();
    ROS_INFO("After rotation phi: %lf\n", this->phi);
}

// To test rotation using sensor values and doing feedback control on that
void TerminalControlHusky::testRot()
{
    ros::Rate rate(HZ);

    // Update position and save current angle
    ros::spinOnce();
    ROS_INFO("Current phi: %lf\n", this->phi);
    double cur_angle = this->phi;
    double move_angle = 0.0; // initialize move_angle

    // Initialize msg
    geometry_msgs::Twist vel_msg;

    while(abs(move_angle) <= M_PI/18.0*17.0) // While error lesser than 170deg
    {
        // Set some angular velocity
        vel_msg.angular.z = 3.0/18.0*M_PI; // 20deg/s
        velocityPublisher.publish(vel_msg);

        ros::spinOnce();
        move_angle = this->phi - cur_angle;
        unwrapAngle(move_angle);

        rate.sleep();
    }

    while(abs(move_angle) >= M_PI/180.0*3.0) // While error greater than 5deg
    {
        // Scale angle error as velocity to turn (Proportional Controller), limit speed of rotation as this->max_w
        vel_msg.angular.z = abs(KP_angle * move_angle) >= this->max_w ? this->max_w : KP_angle * abs(move_angle);
        velocityPublisher.publish(vel_msg);

        // Update error term
        ros::spinOnce();
        move_angle = this->phi - cur_angle;
        unwrapAngle(move_angle);

        rate.sleep();
    }

    // Stop rotation
    publishVel(0,0);
    ros::spinOnce();
    ROS_INFO("After rotation phi: %lf\n", this->phi);
}

// To test forward/backwards movement using time based approach at constant velocity
void TerminalControlHusky::testMoveTime()
{
    ros::Rate rate(HZ);
    
    // Update state and set output
    ros::spinOnce();
    ROS_INFO("Current x: %lf, y:%lf, phi:%lf\n", this->x, this->y, this->phi);
    double dest_x = this->x + std::cos(this->phi);
    double dest_y = this->y + std::sin(this->phi);
    double move_x = dest_x - this->x;
    double move_y = dest_y - this->y;
    double move_dist = std::sqrt((move_x*move_x) + (move_y*move_y));
    ROS_INFO("Moving forward by 1m, then backwards by 1m. Expect to reach (%lf, %lf), which is %lf away", dest_x, dest_y, move_dist);

    double finaltime = ros::Time::now().toSec() + 4;

    while (ros::Time::now().toSec() < finaltime)
    {
        // Update Position
        ros::spinOnce();

        publishVel(0.25, 0); // 0.25m/s
        rate.sleep(); // 50Hz max
    }

    publishVel(0, 0);
    ros::spinOnce();
    ROS_INFO("Moved forwards by 1m. Current x: %lf, y:%lf, phi:%lf\n", this->x, this->y, this->phi);

    sleep(3); // sleep 3s

    finaltime = ros::Time::now().toSec() + 4;

    while (ros::Time::now().toSec() < finaltime)
    {
        // Update Position
        ros::spinOnce();

        publishVel(-0.25, 0); // 0.25m/s
        rate.sleep(); // 50Hz max
    }

    publishVel(0, 0);
    ros::spinOnce();
    ROS_INFO("Moved backwards by 1m. Current x: %lf, y:%lf, phi:%lf\n", this->x, this->y, this->phi);
}

// To test forwards/backwards movement using sensor values and doing feedback control
void TerminalControlHusky::testMove()
{
    ros::Rate rate(HZ);

    // Update state and set output
    ros::spinOnce();
    double init_x = this->x;
    double init_y = this->y;
    ROS_INFO("Current x: %lf, y:%lf, phi:%lf\n", init_x, init_y, this->phi);
    double dest_x = init_x + std::cos(this->phi);
    double dest_y = init_y + std::sin(this->phi);
    double move_x = dest_x - init_x;
    double move_y = dest_y - init_y;
    double move_dist = std::sqrt((move_x*move_x) + (move_y*move_y));
    ROS_INFO("Moving forward by 1m, then backwards by 1m. Expect to reach (%lf, %lf), which is %lfm away", dest_x, dest_y, move_dist);

    // Initialize msg
    geometry_msgs::Twist vel_msg;
    std::cout << move_dist << std::endl;

    while(move_dist >= 0.1) // While error greater than 0.1m
    {
        // Set linear velocity to be proportional to error, max of 0.25
        vel_msg.linear.x = (KP_dist * move_dist) > 0.25 ? 0.25 : KP_dist * move_dist;
        velocityPublisher.publish(vel_msg);

        // Update error
        ros::spinOnce();
        move_x = dest_x - this->x;
        move_y = dest_y - this->y;
        move_dist = std::sqrt((move_x*move_x) + (move_y*move_y));

        rate.sleep();
    }
    
    // Stop moving
    publishVel(0, 0);
    ros::spinOnce();
    ROS_INFO("Moved forwards by 1m. Current x: %lf, y:%lf, phi:%lf\n", this->x, this->y, this->phi);

    sleep(3); // sleep 3s

    // Set initial position as new goal
    move_x = this->x - init_x;
    move_y = this->y - init_y;
    move_dist = std::sqrt((move_x*move_x) + (move_y*move_y));

    while(move_dist >= 0.1) // While error greater than 0.1m
    {
        // Set linear velocity to be proportional to error, max of 0.25
        vel_msg.linear.x = (KP_dist * move_dist) > 0.25 ? -0.25 : -(KP_dist * move_dist);
        velocityPublisher.publish(vel_msg);

        // Update error
        ros::spinOnce();
        move_x = init_x - this->x;
        move_y = init_y - this->y;
        move_dist = std::sqrt((move_x*move_x) + (move_y*move_y));

        rate.sleep();
    }

    // Stop moving
    publishVel(0,0);
    ros::spinOnce();
    ROS_INFO("Moved backwards by 1m. Current x: %lf, y:%lf, phi:%lf\n", this->x, this->y, this->phi);
}

// To test square shape movement using time based approach at constant velocity
void TerminalControlHusky::testSqMoveTime(double side)
{
    ros::Rate rate(HZ);

    double dest_x, dest_y, dest_phi, move_x, move_y, move_dist, spd, finaltime, error;

    // loop 4 times
    for(int i = 0; i < 4; i++)
    {
        // Update state and set forward goal
        ros::spinOnce();
        ROS_INFO("Current x: %lf, y:%lf, phi:%lf\n", this->x, this->y, this->phi);

        dest_x = this->x + side*std::cos(this->phi);
        dest_y = this->y + side*std::sin(this->phi);
        move_x = dest_x - this->x;
        move_y = dest_y - this->y;
        move_dist = std::sqrt((move_x*move_x) + (move_y*move_y));
        ROS_INFO("Moving forward by %lfm. Expect to reach (%lf, %lf), which is %lf away", side, dest_x, dest_y, move_dist);

        if (side <= 1)
        {
            spd = 0.25;
        }
        else if (side <= 3)
        {
            spd = 0.5;
        }
        else
        {
            spd = 1.0;
        }

        finaltime = ros::Time::now().toSec() + side/spd;

        while (ros::Time::now().toSec() < finaltime)
        {
            // Update Position
            ros::spinOnce();

            publishVel(spd, 0);
            rate.sleep(); // 50Hz max
        }

        publishVel(0, 0); // Stop
        ros::spinOnce();
        error = std::sqrt((this->x - dest_x)*(this->x - dest_x) + (this->y - dest_y)*(this->y - dest_y));
        ROS_INFO("Current x: %lf, y:%lf, phi:%lf, which is %lf away from goal\n", this->x, this->y, this->phi, error);

        // Set rotation goal
        ros::spinOnce();
        dest_phi = this->phi + M_PI/2;
        unwrapAngle(dest_phi);

        finaltime = ros::Time::now().toSec() + 3;

        while (ros::Time::now().toSec() < finaltime)
        {
            // Update Position
            ros::spinOnce();

            publishVel(0, M_PI/18.0*3.0); // 30 deg per s
            rate.sleep(); // 50Hz max
        }
        publishVel(0, 0);
        ros::spinOnce();
        error = this->phi - dest_phi;
        ROS_INFO("After rotation phi: %lf, which is %lf away from goal\n", this->phi, error);
    }
}

// To test forwards/backwards movement using sensor values and doing feedback control
void TerminalControlHusky::testSqMove(double side)
{
    ros::Rate rate(HZ);

    double init_x, init_y, init_phi, dest_x, dest_y, dest_phi, move_x, move_y, move_dist, move_angle, error;
    double vel, ang;

    // loop 4 times
    for(int i = 0; i < 4; i++)
    {
        // Update state and set forward goal
        ros::spinOnce();
        double init_x = this->x;
        double init_y = this->y;
        ROS_INFO("Current x: %lf, y:%lf, phi:%lf\n", init_x, init_y, this->phi);

        dest_x = init_x + side*std::cos(this->phi);
        dest_y = init_y + side*std::sin(this->phi);
        move_x = dest_x - init_x;
        move_y = dest_y - init_y;
        move_dist = ((this->phi > -45*M_PI/180 && this->phi < 45*M_PI/180) || this->phi > 135*M_PI/180 || this->phi < -135*M_PI/180) ? move_x/std::cos(this->phi) : move_y/std::sin(this->phi); // Relative to Husky direction
        ROS_INFO("Moving forward by %lfm. Expect to reach (%lf, %lf), which is %lf away", side, dest_x, dest_y, move_dist);
        int count = 0;
        while(abs(move_dist) >= 0.01) // While error greater than 0.1m
        {
            // ROS_INFO_STREAM("Dist: " << move_x << "," << move_y << "," << move_dist << "; phi: " << this->phi*180/M_PI);
            // Set forward velocity with respect to distance, limit to max_v
            vel = (KP_dist * move_dist) > this->max_v ? this->max_v : KP_dist * move_dist;

            // Slowly increase velocity if desired max velocity too early on
            count++;
            if (count < ACCEL_T*HZ && (vel == this->max_v))
            {
                vel *= count/(ACCEL_T*HZ);
            }
            publishVel(vel, 0);

            // Update error
            ros::spinOnce();
            move_x = dest_x - this->x;
            move_y = dest_y - this->y;
            move_dist = ((this->phi > -45*M_PI/180 && this->phi < 45*M_PI/180) || this->phi > 135*M_PI/180 || this->phi < -135*M_PI/180) ? move_x/std::cos(this->phi) : move_y/std::sin(this->phi); // Relative to Husky direction

            rate.sleep();
        }
        publishVel(0, 0); // Stop

        ros::spinOnce();
        error = std::sqrt((this->x - dest_x)*(this->x - dest_x) + (this->y - dest_y)*(this->y - dest_y));
        ROS_INFO("Current x: %lf, y:%lf, phi:%lf, which is %lf away from goal\n", this->x, this->y, this->phi, error);

        // Set rotation goal
        ros::spinOnce();
        init_phi = this->phi;
        dest_phi = init_phi + M_PI/2;
        unwrapAngle(dest_phi);
        move_angle = dest_phi - init_phi;

        while (abs(move_angle) >= 0.02) // While error greater than 0.1 rad
        {
            // Scale angle error as velocity to turn (Proportional Controller), limit speed of rotation as this->max_w
            ang = abs(KP_angle * move_angle) >= this->max_w ? this->max_w : KP_angle * abs(move_angle);
            publishVel(0, ang);

            // Update error term
            ros::spinOnce();
            move_angle = this->phi - dest_phi;
            unwrapAngle(move_angle);

            rate.sleep();
        }
        publishVel(0, 0);
        ros::spinOnce();
        error = this->phi - dest_phi;
        ROS_INFO("After rotation phi: %lf, which is %lf away from goal\n", this->phi, error);
        ros::Duration(1).sleep(); // sleep for a second
    }
}

void TerminalControlHusky::angleToZero()
{
    ros::spinOnce();
    ros::Rate rate(HZ);
    geometry_msgs::Twist vel_msg;
    double angle = getPhi();
    while (abs(angle) > 0.01)
    {
        ros::spinOnce();
        angle = getPhi();
        vel_msg.angular.z = abs(angle) <= 0.01 ? -1*getSign(angle)*max_w : -1*angle;
        vel_msg.linear.x = 0;
        velocityPublisher.publish(vel_msg);
        rate.sleep();
    }
}

int TerminalControlHusky::getSign(double num)
{
    if (num < 0)
        return -1;
    else 
        return 1;
}
