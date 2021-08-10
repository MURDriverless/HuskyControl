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
TerminalControlHusky::TerminalControlHusky(float max_v, float max_w, float KP_dist, float KP_angle)
{
    this->max_v = max_v < MAXSPD ? max_v : MAXSPD;
    this->max_w = max_w*M_PI/180 < MAXROT ? max_w*M_PI/180 : MAXROT;
    this->KP_dist = KP_dist;
    this->KP_angle = KP_angle;
    this->velocityPublisher = n.advertise<geometry_msgs::Twist>(CMDVEL_TOPIC, 1);
    this->poseSubscriber = n.subscribe(ODOM_TOPIC, 1, &TerminalControlHusky::updatePose, this);

    centre_pointsX.reserve(10);
    centre_pointsY.reserve(10);
    centre_splinedX.reserve(50);
    centre_splinedY.reserve(50);
    T.reserve(100);
    currentGoalPointX = getX();
    currentGoalPointY = getY();
}

/* Husky Class Get Functions */
float TerminalControlHusky::getX()
{
    return this->x;
}
float TerminalControlHusky::getY()
{
    return this->y;
}
float TerminalControlHusky::getPhi() // in radians
{
    return this->phi;
}
float TerminalControlHusky::getLinVel()
{
    return this->linvel;
}
float TerminalControlHusky::getAngVel()
{
    return this->angvel;
}

// Update Husky object's pose when receive /Huskysim/Pose/ msgs
// topic: ODOM_TOPIC
// msg type: geometry_msgs::Pose.msg
// args: [float] x y theta linear_velocity angular_velocity
void TerminalControlHusky::updatePose(const nav_msgs::Odometry& msg)
{
    this->x = msg.pose.pose.position.x;
    this->y = msg.pose.pose.position.y;
    
    float q_x = msg.pose.pose.orientation.x;
    float q_y = msg.pose.pose.orientation.y;
    float q_z = msg.pose.pose.orientation.z;
    float q_w = msg.pose.pose.orientation.w;

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
void TerminalControlHusky::publishVel(float lin_vel, float ang_vel)
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
void TerminalControlHusky::move(float dist, float speed, bool isForward, float angle)
{
    // Update position
    ros::spinOnce();

    ros::Rate rate(HZ);

    // Initialize msg
    geometry_msgs::Twist vel_msg;

    // Time based implementation
    float initialtime, traveltime, finaltime;

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
void TerminalControlHusky::smoothmove(float dest_x, float dest_y, float error)
{
    ros::Rate rate(HZ);

    // Initialize msg
    geometry_msgs::Twist vel_msg;

    // Initialize Variables
    float move_x, move_y, move_dist, dest_angle, move_angle;
    float count = 0; // To have smooth acceleration at beginning

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
        
        //std::cout << "linvel set as:" << vel_msg.linear.x << std::endl;

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
        
        //std::cout << "angvel set as:" << vel_msg.angular.z << std::endl;

        velocityPublisher.publish(vel_msg);

        // Update position
        ros::spinOnce();
        std::cout << "Husky current position: ("<<getX()<<", "<<getY()<<")"<<std::endl;

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
// float TerminalControlHusky::Quart2EulerYaw(float q_x, float q_y, float q_z, float q_w)
// {
//     float siny_cosp = 2.0 * (q_w * q_z + q_x * q_y);
//     float cosy_cosp = 1.0 - (2.0 * (q_y * q_y + q_z * q_z));
//     return std::atan2(siny_cosp, cosy_cosp);
// }

void TerminalControlHusky::pathFollower(std::vector<float> &ptX,std::vector<float> &ptY)
{
    ros::spinOnce();
    ros::Rate rate(HZ);
    geometry_msgs::Twist vel_msg;
    clearVecs();
    

    centre_pointsX.push_back(getX());
    centre_pointsY.push_back(getY());
    for (int i=0; i<ptX.size(); i++)
    {
        centre_pointsX.push_back(ptX[i]);
        centre_pointsY.push_back(ptY[i]);
    }

    float dist;
    generateSplines();
    while (true)
    {
        ros::spinOnce();
        dist = getDistFromCar(currentGoalPointX,currentGoalPointY);
        if (Lfc > dist)
            getGoalPoint();
        std::cout<<"[Control] Current goal point is: ("<<currentGoalPointX<<", "<<currentGoalPointY<<")"<<std::endl;
        std::cout<<"[Control] ditance to current goal is: "<<dist<<std::endl;


        if (endOfPath)
        {
            std::cout<<"[steeringControl] end of path triggered!"<<std::endl;
            lin_velocity = 0.75 * max_v;  //slow down
            if (dist < 0.25)
            {  
                lin_velocity = 0;
                ang_velocity = 0;
                ROS_INFO_STREAM("GOAL REACHED!");

                break;
            }      
        }
        else
            lin_velocity = max_v; //constant velocity for now
        
        float angle = getAngleFromCar(currentGoalPointX,currentGoalPointY);
        ang_velocity = abs(angle) >= max_w ? getSign(angle)*max_w : angle;

    
        vel_msg.linear.x = lin_velocity;
        vel_msg.angular.z = ang_velocity;
        velocityPublisher.publish(vel_msg);
        rate.sleep();
    }
}

void TerminalControlHusky::clearVecs()
{
    centre_pointsX.clear();
    centre_pointsY.clear();
    centre_splinedX.clear();
    centre_splinedY.clear();
    T.clear();
    index = 1;
    endOfPath = false;
    
}
float TerminalControlHusky::getSign(float &num)
{
    if (num < 0)
        return -1.0;
    else 
        return 1.0;
}
float TerminalControlHusky::getDistFromCar(float &pntX,float &pntY) 
{
    float dX = getX() - pntX;
	float dY = getY() - pntY;
    return sqrt((dX*dX) + (dY*dY));
}

float TerminalControlHusky::getAngleFromCar(float &pntX,float &pntY)
{
    float dX = pntX - getX();
	float dY = pntY - getY();
    float ang  = atan2(dY,dX) - getPhi();
    if (ang > M_PI)
        ang -= 2*M_PI;
    else if (ang < -M_PI)
        ang += 2*M_PI;
    
    return ang;
}
void TerminalControlHusky::getGoalPoint()
{
    
    float dist;

    while (true)
    {
        if (index+1 >= centre_splinedX.size())
            break;
        dist = getDistFromCar(centre_splinedX[index],centre_splinedY[index]);
        if (dist >= Lfc)
            break;
        else
            index++;
    }

    if (index == centre_splinedX.size()-1) //if at last index of centre_splined path
    {
        endOfPath = true;
        currentGoalPointX = centre_splinedX.back();
        currentGoalPointY = centre_splinedY.back();
        std::cout<<"[getGoalPoint] Husky near end of path" <<std::endl;
    }
    else
    {
        endOfPath = false;
        currentGoalPointX = centre_splinedX[index]; //return value
        currentGoalPointY = centre_splinedY[index];
    }

}

void TerminalControlHusky::generateSplines()
{
    // temp vectors for splining
    
    float stepSize = 0.2;

    //there must be at least 3 points for cubic spline to work
    if (centre_pointsX.size() == 2) //if only 2, make a line
    {
        float tempX, tempY;

        float slopeY = (centre_pointsY.back() - centre_pointsY.front()) / stepSize;
        float slopeX = (centre_pointsX.back() - centre_pointsX.front()) / stepSize;
        for (float i = 0; i<=centre_pointsX.size(); i+= stepSize)
        {
            tempY = slopeY * i;
            tempX = slopeX * i;
            centre_splinedX.push_back(tempX);
            centre_splinedY.push_back(tempY);
        }
    }
    else //for 2 < centre points size < 10 
    {
        for (int t = 0; t< centre_pointsX.size();t++)
        {
            T.push_back(t);
        }

        // Generate Spline Objects
        // spline and x and y separately
        // (see how tk::spline works)
        tk::spline sx, sy;
        sx.set_points(T, centre_pointsX);
        sy.set_points(T, centre_pointsY);

        centre_splinedX.clear(); //erase centre_splined and replace with new points
        centre_splinedY.clear();
        for (float i = 0; i < T.size(); i += stepSize)
        {
            centre_splinedX.push_back(sx(i));
            centre_splinedY.push_back(sy(i));

        }
        std::cout<<"[Control] Splined path size: "<<centre_splinedX.size()<<std::endl;
    }
}