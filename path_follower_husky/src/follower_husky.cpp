/**
 * This is the Path follower for the husky
 * it receives path information from path planner
 * then passes actuation commands to the Husky
 * 
 * uses pure pursuit controller, velocity is constant for now
 * 
 * see header file for descriptions of member variables
 * author: Aldrei (MURauto21)
*/

#include "follower_husky.h"
#include <iostream>

// constructor
HuskyFollower::HuskyFollower(ros::NodeHandle n, double max_v, double max_w)
                :nh(n), max_v(max_v),max_w(max_w)
{
    //set capacity of vectors
    path_x.reserve(500);
    path_y.reserve(500);
    centre_points.reserve(500);
    centre_splined.reserve(2000);
    centre_endOfLap.reserve(100);
    xp.reserve(200);
    yp.reserve(200);
    T.reserve(200);

    if (ros::ok())
    {
        launchSubscribers();
        launchPublishers();
    }
    
    waitForMsgs();

    ROS_INFO_STREAM("[FOLLOWER] follower initialized, publisher and subscriber launched!");
}

// spinonce when msgs are received
void HuskyFollower::waitForMsgs()
{
    if (!path_msg_received || !odom_msg_received && ros::ok()) 
    {
	ros::spinOnce();
    }
}

// void loop()
void HuskyFollower::spin()
{
    waitForMsgs();
    steeringControl();
    publishCtrl();
    pushPathViz();
    clearVars();
    ros::Rate(HZ).sleep();
}

// clear temporary vectors and flags
void HuskyFollower::clearVars()
{
    path_x.clear();
    path_y.clear();
    odom_msg_received = false;
    path_msg_received = false;
    new_centre_points = false;
    cenPoints_updated = 0;
    newGP = false;
    xp.clear();
    yp.clear();
    T.clear();
}

//standard ROS func
int HuskyFollower::launchSubscribers()
{
    sub_odom = nh.subscribe(ODOM_TOPIC, 1, &HuskyFollower::odomCallback, this);
	sub_path = nh.subscribe(PATH_TOPIC, 1, &HuskyFollower::pathCallback, this);
    sub_transition = nh.subscribe(FASTLAP_READY_TOPIC, 1, &HuskyFollower::transitionCallback, this);
}

//standard ROS func
int HuskyFollower::launchPublishers()
{
    pub_control = nh.advertise<geometry_msgs::Twist>(CMDVEL_TOPIC, 1);
    pub_path_viz = nh.advertise<nav_msgs::Path>(PATH_VIZ_TOPIC2, 1);
}

//standard ROS func. gets transition msg from fast lap
void HuskyFollower::transitionCallback(const mur_common::transition_msg &msg)
{
    fastLapReady = msg.fastlapready;
}

// get odometry messages
void HuskyFollower::odomCallback(const nav_msgs::Odometry &msg)
{
    
    if (!initialised)
    {
       initX = msg.pose.pose.position.x;
       initY = msg.pose.pose.position.y;
       initYaw = car_yaw;
       initialised = true;
       currentGoalPoint.updatePoint(centre_points.back());
    }
    car_x = msg.pose.pose.position.x;
    car_y = msg.pose.pose.position.y;
    
    double q_x = msg.pose.pose.orientation.x;
    double q_y = msg.pose.pose.orientation.y;
    double q_z = msg.pose.pose.orientation.z;
    double q_w = msg.pose.pose.orientation.w;

    car_lin_v = msg.twist.twist.linear.x;
    car_ang_v = msg.twist.twist.angular.z;

    // convert quaternions to euler
    tf::Quaternion q(q_x, q_y, q_z, q_w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    car_yaw = yaw;
    odom_msg_received = true;
}

//get path msgs from path planner
void HuskyFollower::pathCallback(const mur_common::path_msg &msg)
{   
    // if the last 5 path points have changed, copy new path points
    int j =0;
    for (int i=centre_points.size()-1; i>=0 ;i--)
    {
        if (calcDist(centre_points[i],PathPoint(msg.x.back(),msg.y.back()))>0.01)
        {
            new_centre_points = true;
            break;
        }
        if (j>5) break;
        j++;
    }
    
    //copy path points msg
    if (centre_points.empty() || new_centre_points)
    {
        centre_points.clear();
        for (int i=0; i < msg.x.size(); i++)
        {
            centre_points.emplace_back(msg.x[i],msg.y[i]);
        }
        generateSplines();
    }

    //check if lap is complete
    if (calcDist(PathPoint(initX,initY),centre_splined.back())<0.02)
        plannerComplete = true;

    path_msg_received = true;
}



// publish splined path to RVIZ
void HuskyFollower::pushPathViz()
{
    nav_msgs::Path path_viz_msg;
    path_viz_msg.header.frame_id = "odom";

    std::vector<geometry_msgs::PoseStamped> poses;
    poses.reserve(centre_splined.size());

    for (int p = 0; p < centre_splined.size(); p++)
    {
        geometry_msgs::PoseStamped item; 
        item.header.frame_id = "map";
        item.header.seq = p;
        item.pose.position.x = centre_splined[p].x;
        item.pose.position.y = centre_splined[p].y;
        item.pose.position.z = 0.0;

        poses.emplace_back(item);
    }

    path_viz_msg.poses = poses;
    pub_path_viz.publish(path_viz_msg);
}

//publish actuation control commands
void HuskyFollower::publishCtrl()
{
    geometry_msgs::Twist vel_msg;
    vel_msg.linear.x = lin_velocity;
    vel_msg.angular.z = ang_velocity;

    pub_control.publish(vel_msg);
}

// compute linear and angular velocity commands
// can be confusing, dont mind end of lap codes at first
void HuskyFollower::steeringControl()
{
	if (centre_points.size() <= 1) //no path points yet
    {
        lin_velocity = 0.0;
        ang_velocity = 0.0;
        return; //to ignore rest of function
    }
    
    //go straight but slowly until more points are discovered
    // this could cause errors. might need to delete later
    // else if (centre_points.size() < 3) 
    // {
    //     lin_velocity = 0.2;
    //     ang_velocity = 0.0;
    //     return; //to ignore rest of function
    // }

    double dist = getDistFromCar(currentGoalPoint);
    if (endOfLap)
    {
        // lin_velocity = 0;
        // ang_velocity = 0;
        ROS_INFO_STREAM("[FOLLOWER] SLOW LAP FINISHED! waiting for fast lap ready...");
        slowLapFinish = true;
          
    }

    // check if need to change goal pt 
    if (Lf > dist) 
        getGoalPoint();

    if (endOfPath)
    {
        if (DEBUG) std::cout<<"[FOLLOWER] end of path triggered!"<<std::endl;
        lin_velocity = 0.75 * max_v;  //slow down //Max says should not slow down
        
        if (plannerComplete)//
        {
            endOfLap = true;
            index = -1;
            getGoalPoint();
        }
    }
    else
        lin_velocity = max_v; //constant velocity for now
    if (endOfLap)
    {
        if (DEBUG) std::cout<<"[FOLLOWER] Distance to finish line: "<<getDistFromCar(centre_points.front())<<std::endl;
        
    }
        
    double angle = getAngleFromCar(currentGoalPoint);
    ang_velocity = abs(KP * angle) >= max_w ? getSign(angle)*max_w : KP * angle;
}

// calculate distance between 2 points
double HuskyFollower::calcDist(const PathPoint &p1, const PathPoint &p2)
{
    float x_dist = pow(p2.x - p1.x, 2);
    float y_dist = pow(p2.y - p1.y, 2);

    return sqrt(x_dist + y_dist);
}

//calculate distance of a point to the car
double HuskyFollower::getDistFromCar(PathPoint& pnt) 
{
    double dX = car_x - pnt.x;
	double dY = car_y - pnt.y;
    return sqrt((dX*dX) + (dY*dY));
}

// calculate the angle of a point wrt car
double HuskyFollower::getAngleFromCar(PathPoint& pnt)
{
    double dX = pnt.x - car_x;
	double dY = pnt.y - car_y;
    double ang  = atan2(dY,dX) - car_yaw;
    if (ang > M_PI)
        ang -= 2*M_PI;
    else if (ang < -M_PI)
        ang += 2*M_PI;
    
    return ang;
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
    return;
  
    
    //there must be at least 3 points for cubic spline to work
    if (centre_points.size() <= 2) //if less than = 2, make a line
    {
        
        double tempX, tempY, slopeY,slopeX;
        if (centre_points.size() == 1)
        {
            slopeY = (yp.back() - initY) / STEPSIZE;
            slopeX = (xp.back() - initX) / STEPSIZE;
        }
        else
        {
            slopeY = (yp.back() - yp.front()) / STEPSIZE;
            slopeX = (xp.back() - xp.front()) / STEPSIZE;
        }

        for (double i = 0; i<=2; i+= STEPSIZE)
        {
            tempY = slopeY * i;
            tempX = slopeX * i;
            centre_splined.emplace_back(tempX,tempY);
        }
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
        for (double i = 0; i < T.size(); i += STEPSIZE)
        {
            centre_splined.emplace_back(sx(i),sy(i));
        }
        if (endOfPath && plannerComplete) std::cout<<"[FOLLOWER] Splined last sections of the track!"<< std::endl;
    }

    else //for 2 < centre points size < N 
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
        for (double i = 0; i < T.size(); i += STEPSIZE)
        {
            centre_splined.emplace_back(sx(i),sy(i));
        }
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
    double temp; //temporary var
    double dist = 99999.1; //random large number

    //step 1: look for the point nearest to the car
    if (index == -1 || oldIndex == -1)
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
    if (DEBUG) std::cout<<"[FOLLOWER] new goal point set" <<std::endl;
    
    if (index == centre_splined.size()-1) //if at last index of centre_splined path
    {
        endOfPath = true;
        currentGoalPoint.updatePoint(centre_splined.back());
        if (DEBUG) std::cout<<"[FOLLOWER] car near end of path" <<std::endl;
    }
    else
    {
        endOfPath = false;
        currentGoalPoint.updatePoint(centre_splined[index]); //return value
    }
        
}

// 
double HuskyFollower::getSign(double &num)
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
    double max_v = atof(argv[1]);
    double max_w = atof(argv[2]);
    
    //Initialize Husky Object
    
    HuskyFollower follower(n,max_v, max_w);
        //ros::Rate freq(20);
	
	while (ros::ok())
    {
	    follower.spin();
        if (follower.fastLapReady)
            break;
	}
    return 0;
    
}