#include "follower_husky.h"
#include <iostream>

HuskyFollower::HuskyFollower(ros::NodeHandle n, double max_v, double max_w, double KP_dist, double KP_angle)
                :nh(n), max_v(max_v),max_w(max_w), KP_dist(KP_dist), KP_angle(KP_angle)
{
    //specify minimum size of vectors
    path_x.reserve(500);
    path_y.reserve(500);
    centre_points.reserve(500);
    centre_splined.reserve(2000);
    centre_endOfLap.reserve(10);
    xp.reserve(200);
    yp.reserve(200);
    T.reserve(200);

    if (ros::ok())
    {
        launchSubscribers();
        launchPublishers();
    }
    
    waitForMsgs();

    ROS_INFO_STREAM("FOLLOWER: follower initialized, publisher and subscriber launched!");
}

void HuskyFollower::waitForMsgs()
{
    if (!path_msg_received || !odom_msg_received && ros::ok()) 
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

    if (DEBUG) std::cout<<"Splined points size: "<<centre_splined.size()<<" Current index: "<<index<<"\n"<<std::endl;
    
    ros::Rate(HZ).sleep();
}

void HuskyFollower::clearVars()
{
    path_x.clear();
    path_y.clear();
    odom_msg_received = false;
    path_msg_received = false;
    new_centre_points = false;
    newGP = false;
    xp.clear();
    yp.clear();
    T.clear();
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
    //convert quaternion to Euler
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
        PathPoint offset(path_x.front(),path_y.front()); //use this if path doesnt start at (0,0)
        float tempX, tempY;
        for(int i=centre_points.size();i<path_x.size();i++)
        {
            tempX = path_x[i] - offset.x;
            tempY = path_y[i] - offset.y;
            centre_points.emplace_back(tempX,tempY);
            if (DEBUG) std::cout<<"[pathCallback] new points received: "<< centre_points[i].x<<", "<<centre_points[i].y<<std::endl;
        }
        generateSplines();
    }
    
    
}

void HuskyFollower::pushPathViz() //for rviz
{
    nav_msgs::Path path_viz_msg;
    path_viz_msg.header.frame_id = "odom";

    std::vector<geometry_msgs::PoseStamped> poses;
    poses.reserve(centre_points.size());

    for (int p = 0; p < centre_points.size(); p++)
    {
        geometry_msgs::PoseStamped item; 
        item.header.frame_id = "map";
        item.header.seq = p;
        item.pose.position.x = centre_points[p].x;
        item.pose.position.y = centre_points[p].y;
        item.pose.position.z = 0.0;

        poses.emplace_back(item);
    }

    path_viz_msg.poses = poses;
    pub_path_viz.publish(path_viz_msg);
}

void HuskyFollower::publishCtrl()
{
    geometry_msgs::Twist vel_msg;
    vel_msg.linear.x = lin_velocity;
    vel_msg.angular.z = ang_velocity;

    pub_control.publish(vel_msg);
}


void HuskyFollower::steeringControl()
{
	if (centre_points.size() < 2) //go straight but slowly until more points are discovered
    {
        lin_velocity = 0.02;
        ang_velocity = 0.0;
        return;
    }
    float dist = getDistFromCar(currentGoalPoint);

    if (endOfLap)
    {
        if (dist < 1)
        {  
            lin_velocity = 0;
            ang_velocity = 0;
            ROS_INFO_STREAM("SLOW LAP FINISHED!");
            slowLapFinish = true;
        }
           

    }

    if (DEBUG) std::cout << "[steeringControl] Current car pose: ("<<car_x<<", "<<car_y<<")"<<std::endl;

    if (Lf > dist)
        getGoalPoint();
    else
        if (DEBUG) std::cout<<"[steeringControl] ditance to current goal is: "<<dist<<std::endl;

    if (DEBUG) std::cout<<"[steeringControl] Current goal point is: ("<<currentGoalPoint.x<<", "<<currentGoalPoint.y<<")"<<std::endl;
    
    if (endOfPath) //slow down
    {
        lin_velocity = 0.25 * MAX_V;
        if (getDistFromCar(centre_splined.front())< Lf)//if 1 look ahead distance away from finish line
        {
            endOfLap = true;
        }
    }
    else
        lin_velocity = MAX_V; //constant velocity for now
        
    float angle = getAngleFromCar(currentGoalPoint);

    if (DEBUG) std::cout <<"[steeringControl] angle error is "<<(angle*180/M_PI)<<" degrees" << std::endl;

    ang_velocity = abs(KP_angle * angle) >= max_w ? getSign(angle)*max_w : KP_angle * angle;
 
}


float HuskyFollower::getDistFromCar(PathPoint pnt) 
{
    float dX = car_x - pnt.x;
	float dY = car_y - pnt.y;
    return sqrt((dX*dX) + (dY*dY));
}

float HuskyFollower::getAngleFromCar(PathPoint pnt)
{
    float dX = pnt.x - car_x;
	float dY = pnt.y - car_y;
    return atan2(dY,dX) - car_yaw;
}

/**********
* This Function uses tk::spline library (see spline.h)
* Path points from path planner have metres of interval, they are splined to have a smoother path
* Splining is computationally expensive, so we will not spline all the path points
* variables:
* centre_points: path points from path planner
* centre_splined: splined path points
* xp, yp, T: temporary variables for generatting splines using tk::spline
***********/
void HuskyFollower::generateSplines()
{
    if (endOfLap)
    {
        xp.push_back(car_x);
        yp.push_back(car_y);
        T.push_back(0);
        for (int i=0; i<STOP_INDEX; i++)
        {
            xp.push_back(centre_points[i].x);
            yp.push_back(centre_points[i].y);
            T.push_back(i+1);
        }
        tk::spline sx, sy;
        sx.set_points(T, xp);
        sy.set_points(T, yp);
        for (float i = 0; i < T.size(); i += STEPSIZE)
        {
            centre_endOfLap.emplace_back(sx(i),sy(i));
        }
        if (DEBUG) std::cout<<"[splines] end of lap splined: "<<std::endl;
    }
    
    //there must be at least 3 points for cubic spline to work
    else if (centre_points.size() == 2) //if only 2, make a line
    {
        float tempX, tempY;
        for (auto p:centre_points)
        {
            xp.push_back(p.x);
            yp.push_back(p.y);
        }

        float slopeY = (yp.back() - yp.front()) / STEPSIZE;
        float slopeX = (xp.back() - xp.front()) / STEPSIZE;
        for (float i = 0; i<=xp.size(); i+= STEPSIZE)
        {
            tempY = slopeY * i;
            tempX = slopeX * i;
            centre_splined.emplace_back(tempX,tempY);
        }
        if (DEBUG) std::cout<<"[splines] new centre_splined size is: "<<centre_splined.size()<<std::endl;
        
    }

    else if (centre_points.size()>SPLINE_N) //we will only spline the last N points as it is computationally expensive
    {
        //separate x and y values
        int t = 0;
        for (int i = centre_points.size()-SPLINE_N; i < centre_points.size(); i++)
            {
                xp.push_back(centre_points[i].x);
                yp.push_back(centre_points[i].y);
                T.push_back(t);
                t++;
            }

        // Generate Spline Objects
        // spline and x and y separately
        // (see how tk::spline works)
        tk::spline sx, sy;
        sx.set_points(T, xp);
        sy.set_points(T, yp);
        
        int temp = (centre_points.size() - SPLINE_N )/ STEPSIZE;
        centre_splined.assign(centre_splined.begin(),centre_splined.begin()+ temp);  //erase the last N points, then replace with new points
        for (float i = 0; i < T.size(); i += STEPSIZE)
        {
            centre_splined.emplace_back(sx(i),sy(i));
        }
        if (DEBUG) std::cout<<"[splines] new centre_splined size is: "<<centre_splined.size()<<std::endl;
    }

    else //for 2 < centre points size < 10 
    {
        //separate x and y values
        int t=0;
        for (auto p:centre_points)
        {
            xp.push_back(p.x);
            yp.push_back(p.y);
            T.push_back(t);
            t++;
        }

        // Generate Spline Objects
        // spline and x and y separately
        // (see how tk::spline works)
        tk::spline sx, sy;
        sx.set_points(T, xp);
        sy.set_points(T, yp);

        centre_splined.clear(); //erase centre_splined and replace with new points
        for (float i = 0; i < T.size(); i += STEPSIZE)
        {
            centre_splined.emplace_back(sx(i),sy(i));
        }
        if (DEBUG) std::cout<<"[splines] new centre_splined size is: "<<centre_splined.size()<<std::endl;
    }
       
}


/*************
* This function searches for the goal point from the splined path points 
*  (searches for the index of the goal point from centre_splined vector)
* The concept of look ahead distance of the pure puruit controller is used here
*
**/
void HuskyFollower::getGoalPoint()
{
    if (endOfLap) //if end of lap, change reference path to centre_endOfLap
    {
        currentGoalPoint.updatePoint(centre_endOfLap[index_endOfLap]);//return value
        index_endOfLap ++; 
        return; //so it wont run the rest of this function
    }

    float temp; //temporary var
    float dist = 99999.1; //random large number

    //step 1: look for the point nearest to the car
    if (index == -1 || oldIndex == -1) //first iteration
    {
        for (int i = 0; i < centre_splined.size(); i++)
        {
            temp = getDistFromCar(centre_splined[i]);
            if(dist < temp)
            {
                break;
            }
            else
            {
                dist = temp;
                index = i;
            }   
        }
        oldIndex = index;
    }

    else //
    {
        index = oldIndex;
        dist = getDistFromCar(centre_splined[index]); //get dist of old index
        //search for new index with least dist to car
        for(int j = index+1; j < centre_splined.size(); j++)
        {
            temp = getDistFromCar(centre_splined[j]);
            if (dist < temp)
            {
                index = j;
                break;
            }
            dist = temp;
        }
        oldIndex = index;
    }

    //look ahead distance
    Lf = LFC;
    //if velocity is not constant, we can adjust lookahead dist using the formula:
    // Lf = LFV * car_lin_v + LFC;

    //search for index with distance to car that is closest to look ahead distance
    while (true)
    {
        if (index+1 >= centre_splined.size())
            break;
        dist = getDistFromCar(centre_splined[index]);
        if (dist >= Lf)
            break;
        else
            index++;
    }
    if (DEBUG) std::cout<<"[getGoalPoint] new goal point set" <<std::endl;
    
    if (index == centre_splined.size()-1) //if last index of path
    {
        endOfPath = true;
        currentGoalPoint.updatePoint(centre_splined.back());
        if (DEBUG) std::cout<<"[getGoalPoint] car near end of path" <<std::endl;
    }
    else
    {
        endOfPath = false;
        currentGoalPoint.updatePoint(centre_splined[index]); //return value
    }
        
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
    
    HuskyFollower follower(n,0.7, 0.55, 0.5, 1);
        //ros::Rate freq(20);
	
	while (ros::ok())
    {
	    follower.spin();
        if (follower.slowLapFinish)
            break;
	}
    return 0;
    
}