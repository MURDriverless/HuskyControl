#include "follower_husky.h"
#include <iostream>

HuskyFollower::HuskyFollower(ros::NodeHandle n, double max_v, double max_w, double KP_dist, double KP_angle)
                :nh(n), max_v(max_v),max_w(max_w), KP_dist(KP_dist), KP_angle(KP_angle)
{
    path_x.reserve(500);
    path_y.reserve(500);
    centre_points.reserve(500);
    index = 0;
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
    steeringControl();
    publishCtrl();
    pushPathViz();
    clearVars();
    std::cout<<"points size: "<<centre_points.size()<<" index: "<<index<<std::endl;
    
}
void HuskyFollower::clearVars()
{
    path_x.clear();
    path_y.clear();
    odom_msg_received = false;
    path_msg_received = false;
    new_centre_points = false;
    newGP = false;

}

int HuskyFollower::launchSubscribers()
{
    sub_odom = nh.subscribe(ODOM_TOPIC, 1, &HuskyFollower::odomCallback, this);
	sub_path = nh.subscribe(PATH_TOPIC, 1, &HuskyFollower::pathCallback, this);
    
}

int HuskyFollower::launchPublishers()
{
    pub_control = nh.advertise<geometry_msgs::Twist>(CMDVEL_TOPIC, 1);
    pub_path_viz = nh.advertise<nav_msgs::Path>(PATH_VIZ_TOPIC, 1);
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
        PathPoint offset(path_x.front(),path_y.front());
        new_centre_points = true;
        float tempX, tempY;
        for(int i=centre_points.size();i<path_x.size();i++)
        {
            tempX = path_x[i] - offset.x;
            tempY = path_y[i] - offset.y;
            centre_points.emplace_back(tempX,tempY);
            std::cout<<"new points received: "<< centre_points[i].x<<", "<<centre_points[i].y<<std::endl;
        }
        
    }
    
    
}
void HuskyFollower::pushPathViz()
{
    nav_msgs::Path path_viz_msg;
    path_viz_msg.header.frame_id = "map";

    std::vector<geometry_msgs::PoseStamped> poses;
    poses.reserve(path_x.size());

    for (int p = 0; p < path_x.size(); p++)
    {
        geometry_msgs::PoseStamped item; 
        item.header.frame_id = "map";
        item.header.seq = p;
        item.pose.position.x = path_x[p];
        item.pose.position.y = path_y[p];
        item.pose.position.z = 0.0;

        poses.emplace_back(item);
    }

    path_viz_msg.poses = poses;
    pub_path_viz.publish(path_viz_msg);
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
	if (centre_points.size() == 0)
        return;
    PathPoint goalPoint = getGoalPoint(); //get nearest point from path points
    std::cout<<"goal point: "<<goalPoint.x<<", "<<goalPoint.y<<std::endl;
    std::cout<<"car pose: "<<car_x<<", "<<car_y<<std::endl;

    float dist = getDistFromCar(goalPoint);//pos error
    if (dist > ERRL)
        // lin_velocity = (KP * dist) > MAX_V ? MAX_V : KP * dist;
        lin_velocity = 0.5;
    else
        lin_velocity = 0.002;
        
    std::cout << "linear vel set as:" << lin_velocity << std::endl;

    
    float angle = getAngleFromCar(goalPoint); //angle error
    
    if(abs(angle) > ERRA)
        ang_velocity =  getSign(angle)*max_w;
    else
        ang_velocity = 0.0;
    std::cout << "angular velocity set as:" << ang_velocity << std::endl;
 
}


float HuskyFollower::getDistFromCar(PathPoint pnt)
{
    //updateRearPos();
    float dX = car_x - pnt.x;
	float dY = car_y - pnt.y;
    return sqrt(dX * dX + dY * dY);
}

float HuskyFollower::getAngleFromCar(PathPoint pnt)
{
    float dX = car_x - pnt.x;
	float dY = car_y - pnt.y;
    return atan2(pnt.y,pnt.x) - car_yaw;
}



//this is search target index from Dennis' code
//search for the pathpoint that is closest to the car
PathPoint HuskyFollower::getGoalPoint() 
{
    // if (index == -1 || oldIndex == -1) //first time
    // {
        float dist = 99999.1; //random large number
    //     float temp;
    //     int skip = 0;
    //     for (int i = centre_points.size()-1; i>0; i--)
    //     {
    //         temp = getDistFromCar(centre_points[i]);
    //         if(dist > temp)
    //         {
    //            dist = temp;
    //            skip = 0;
    //            index = i;
    //         }
    //     }
    //     oldIndex = index;
    //     std::cout<<"index: "<<index<<std::endl;
    // }
    // else
    // {
    //     index = oldIndex;
    //     float dis1 = getDistFromCar(centre_points[index]);
    //     for(int j = index; j < centre_points.size(); j++)
    //     {
    //         if (dis1 > getDistFromCar(centre_points[j]))
    //         {
    //             index = j;
    //             break;
    //         }
    //     }
    //     oldIndex = index;
    //     std::cout<<"index: "<<index<<std::endl;
    // }
    // return PathPoint(centre_points[index]);
    float temp = getDistFromCar(centre_points[index]);
    std::cout<< "hello world!!!!!!!   current dis: "<<temp<<std::endl;
    if (temp < ERRL)
    {
        index++;
        newGP = true;
        
        return centre_points[index];
    }
    else
        return centre_points[index];



}

float HuskyFollower::getSign(float &num)
{
    if (num < 0)
        return -1.0;
    else 
        return 1.0;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "HuskyFollower"); 
    ros::NodeHandle n;
       
         // Get parameters from CLI
    float max_v = atof(argv[1]);
    float max_w = atof(argv[2]);
    float lingain = atof(argv[3]);
    float anggain = atof(argv[4]);
    
    //Initialize Husky Object
    
    HuskyFollower follower(n,0.7, 0.09, 0.5, 1.0); //change this
        //ros::Rate freq(20);
	
	    while (ros::ok())
	    {
	        follower.spin();
          //  freq.sleep();
	    }
    
}
