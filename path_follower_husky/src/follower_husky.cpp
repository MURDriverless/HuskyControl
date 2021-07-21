#include "follower_husky.h"
#include <iostream>

HuskyFollower::HuskyFollower(ros::NodeHandle n, double max_v, double max_w, double KP_dist, double KP_angle)
                :nh(n), max_v(max_v),max_w(max_w), KP_dist(KP_dist), KP_angle(KP_angle)
{
    path_x.reserve(500);
    path_y.reserve(500);
    centre_points.reserve(500);
    if (ros::ok())
    {
        launchSubscribers();
        launchPublishers();
    }
    
    waitForMsgs();

    ROS_INFO_STREAM("FOLLOWER: follower initialized, pub and sub launched!");
    
}
void HuskyFollower::waitForMsgs()
{
    while (!odom_msg_received ||!path_msg_received && ros::ok()) 
    {
	ros::spinOnce();
    }
}

void HuskyFollower::spin()
{
    waitForMsgs();
    accelerationControl();
    steeringControl();
    publishCtrl();
    clearVars();
    
}
void HuskyFollower::clearVars()
{
    path_x.clear();
    path_y.clear();
    odom_msg_received = false;
    path_msg_received = false;
    new_centre_points = false;

}

int HuskyFollower::launchSubscribers()
{
    sub_odom = nh.subscribe(ODOM_TOPIC, 1, &HuskyFollower::odomCallback, this);
	sub_path = nh.subscribe(PATH_TOPIC, 1, &HuskyFollower::pathCallback, this);
    
}

int HuskyFollower::launchPublishers()
{
    pub_control = nh.advertise<geometry_msgs::Twist>(CMDVEL_TOPIC, 1);
}

//same as updatePose
void HuskyFollower::odomCallback(const nav_msgs::Odometry &msg)
{
    car_x = msg.pose.pose.position.x;
    car_y = msg.pose.pose.position.y;
    car_lin_v = msg.twist.twist.linear.x;
    car_ang_v = msg.twist.twist.angular.z;

    double q_x = msg.pose.pose.orientation.x;
    double q_y = msg.pose.pose.orientation.y;
    double q_z = msg.pose.pose.orientation.z;
    double q_w = msg.pose.pose.orientation.w;
    tf::Quaternion q(q_x, q_y, q_z, q_w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    car_yaw = yaw;

    odom_msg_received = true;

}

void HuskyFollower::pathCallback(const mur_common::path_msg &msg)
{
    for (int i=0; i < msg.x.size(); i++)
    {
        path_x.push_back(msg.x[i]);
        path_y.push_back(msg.y[i]);
    }
    path_msg_received = true;
    if (centre_points.size() != path_x.size())
    {
        new_centre_points = true;
        for(int i=centre_points.size();i<path_x.size();i++)
        {
            centre_points.emplace_back(path_x[i],path_y[i]);
        }
        
    }
    
}
void HuskyFollower::publishCtrl()
{
    // Initialize msg
    geometry_msgs::Twist vel_msg;
    vel_msg.linear.x = lin_velocity;
    vel_msg.angular.z = ang_velocity;

    pub_control.publish(vel_msg);
}


void HuskyFollower::steeringControl()
{
	PathPoint goalPt = getGoalPoint(); //get nearest point from path points
    std::cout<<"goal point: "<<goalPt.x<<", "<<goalPt.y<<std::endl;
    
    float dist = getDistFromCar(goalPt);
    lin_velocity = (KP * dist) > MAX_V ? MAX_V : KP_dist * dist;
    std::cout << "linvel set as:" << lin_velocity << std::endl;

    float angle = getAngleFromCar(centre_points[1]);
    ang_velocity =  (KP * angle) >= this->max_w ? this->max_w : KP_angle * angle; 
    std::cout << "angular velocity set as:" << ang_velocity << std::endl;
 
}


float HuskyFollower::getDistFromCar(PathPoint pnt)
{
    //updateRearPos();
    float dX = rearX - pnt.x;
	float dY = rearY - pnt.y;
    return sqrt(dX * dX + dY * dY);
}

float HuskyFollower::getAngleFromCar(PathPoint pnt)
{
    float dX = rearX - pnt.x;
	float dY = rearY - pnt.y;
    return atan2(dY,dX) - car_yaw;
}



//this is search target index from Dennis' code
//search for the pathpoint that is closest to the car
PathPoint HuskyFollower::getGoalPoint() 
{
    if (index == -1 || oldIndex == -1) //first time
    {
        float dist = 99999.1; //random large number
        float temp;
        int skip = 0;
        for (int i = centre_points.size()-1; i>0; i--)
        {
            temp = getDistFromCar(centre_points[i]);
            if(dist > temp)
            {
               dist = temp;
               skip = 0;
               index = i;
            }
        }
        oldIndex = index;
        std::cout<<"index: "<<index<<std::endl;
    }
    else
    {
        index = oldIndex;
        float dis1 = getDistFromCar(centre_points[index]);
        for(int j = index; j < centre_points.size(); j++)
        {
            if (dis1 > getDistFromCar(centre_points[j]))
            {
                index = j;
                break;
            }
        }
        oldIndex = index;
        std::cout<<"index: "<<index<<std::endl;
    }
    return PathPoint(centre_points[index]);
    

}

float HuskyFollower::getSign(float &num)
{
    if (num < 0)
        return -1;
    else 
        return 1;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "HuskyFollower"); 
    ros::NodeHandle n;
       
        HuskyFollower follower(n);
        //ros::Rate freq(20);
	
	    while (ros::ok())
	    {
	        follower.spin();
          //  freq.sleep();
	    }
    
}