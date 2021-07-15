#include <ros/ros.h> // Must include for all ROS C++
#include <geometry_msgs/Twist.h> // To control linear and angular velocity of Husky
#include <geometry_msgs/Pose.h> // To obtain coodinates and orientation of Husky
#include <geometry_msgs/Point.h> // To obtain xyz position of Husky
#include <nav_msgs/Odometry.h> // Msg for /mur/slam/Odom topic
#include <cmath>
#include <sstream>

#define CMDVEL_TOPIC "/husky_velocity_controller/cmd_vel"
#define ODOM_TOPIC "/odometry/filtered"

#define MAXSPD 1.0 // Linear Velocity m/s
#define MAXROT 45.0 // Rotation Velociy degrees

// Husky Class
class Husky{
    private:
    // Variables for Current Pose
    double x;
    double y;
    double phi;
    double linvel;
    double angvel;
    double max_v;
    double max_w;
    const double KP_DIST = 5;
    const double KP_ANGLE = 5;
    // ROS
    ros::NodeHandle n; // Create its specific node handler
    ros::Publisher velocityPublisher;
    ros::Subscriber poseSubscriber;

    public:
    // Constructor
    Husky(double max_v, double max_w)
    {
        this->max_v = max_v;
        this->max_w = max_w;
        this->velocityPublisher = n.advertise<geometry_msgs::Twist>(CMDVEL_TOPIC, 10);
        this->poseSubscriber = n.subscribe(ODOM_TOPIC, 10, &Husky::updatePose, this);
    }

    /* Husky Class Functions */
    double getX()
    {
        return x;
    }
    double getY()
    {
        return y;
    }
    double getPhi() // in radians
    {
        return phi;
    }
    double getLinVel()
    {
        return linvel;
    }
    double getAngVel()
    {
        return angvel;
    }

    // Move the robot for a certain distance at a certain speed
    // topic: CMDVEL_TOPIC
    // msg type: geometry_msgs/Twist
    // args: linear angular: [x,y,z] [x,y,z]
    void move(double dist, double speed, bool isForward, double angle)
    {
        // Update position
        ros::spinOnce();

        // Initialize msg
        geometry_msgs::Twist vel_msg;

        // Time based implementation
        double initialtime, traveltime, finaltime;

        // Define Angular Velocity constant
        const double ANG_VEL = this->max_w*M_PI/180;

        // Check if to turn
        if(angle!= 0)
        {
            if(angle  > 0)
            {
                vel_msg.angular.z = ANG_VEL; 
            }
            else
            {
                vel_msg.angular.z = -ANG_VEL; 
            }

            initialtime = ros::Time::now().toSec();
            traveltime = abs(angle)/ANG_VEL;
            finaltime = initialtime + traveltime;

            ROS_INFO("Begin alining direction!");
        
            // Begin turning until specified angle
            velocityPublisher.publish(vel_msg);
            while(ros::Time::now().toSec() <= finaltime)
            {
                velocityPublisher.publish(vel_msg);
                ros::spinOnce();
                // ROS_INFO("this->angle: %lf \n", this->angle);
                // ROS_INFO("angle: %lf \n", angle);
                // ROS_INFO("diff: %lf \n", this->angle - angle);
            }

            vel_msg.angular.z = 0;
            velocityPublisher.publish(vel_msg); // Stop turning

            ROS_INFO("Finished aligning!");
        }

        // Set forward or backwards movement
        if(isForward)
        {
            vel_msg.linear.x = abs(speed);
        }
        else
        {
            vel_msg.linear.x = -abs(speed);
        }

        //Publish Move
        velocityPublisher.publish(vel_msg);

        ROS_INFO("Husky is moving!\n");

        // Stop when reaching the distance specified
        initialtime = ros::Time::now().toSec();
        traveltime = dist/speed;
        finaltime = initialtime + traveltime;
        while (ros::Time::now().toSec() <= finaltime)
        {
            velocityPublisher.publish(vel_msg);
            ros::spinOnce();
        }
        // Reached specified distance
        vel_msg.linear.x = 0;
        
        // Publish again
        velocityPublisher.publish(vel_msg);
        
        ROS_INFO("Husky reached its destination!\n");
    }
    
    // Move the robot to a specified location, moving while rotating at the same time
    void smoothmove(double dest_x, double dest_y, double error)
    {
        // Initialize msg
        geometry_msgs::Twist vel_msg;

        // Initialize Variables
        double move_x, move_y, move_dist, dest_angle, move_angle;
        
        // Initialize frequenccy
        ros::Rate loop_rate(60);

        // Get initial
        move_x = dest_x - this->x;
        move_y = dest_y - this->y;
        move_dist = sqrt((move_x*move_x) + (move_y*move_y));
        
        // Have not reached
        while(move_dist > error)
        {
            // Set forward velocity
            vel_msg.linear.x = KP_DIST * move_dist;
            if(vel_msg.linear.x > this->max_v)
            {
                vel_msg.linear.x = this->max_v;
            }

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

            // Scale angle error as velocity to turn (Proportional Controller)
            vel_msg.angular.z = KP_ANGLE * move_angle;
            
            velocityPublisher.publish(vel_msg);

            // Update position
            ros::spinOnce();

            // Get new distance
            move_x = dest_x - this->x;
            move_y = dest_y - this->y;
            move_dist = sqrt((move_x*move_x) + (move_y*move_y));

            loop_rate.sleep();
        }

        // Should reach destination by now
        vel_msg.linear.x = 0;
        vel_msg.angular.z = 0;
        velocityPublisher.publish(vel_msg);

        ROS_INFO("x: %lf, y:%lf \n", this->x, this->y);    
    }

    // Update Husky object's pose when receive /Huskysim/Pose/ msgs
    // topic: ODOM_TOPIC
    // msg type: geometry_msgs::Pose.msg
    // args: [double] x y theta linear_velocity angular_velocity
    void updatePose(const nav_msgs::Odometry& msg)
    {
        this->x = msg.pose.pose.position.x;
        this->y = msg.pose.pose.position.y;
        
        double q_x = msg.pose.pose.orientation.x;
        double q_y = msg.pose.pose.orientation.y;
        double q_z = msg.pose.pose.orientation.z;
        double q_w = msg.pose.pose.orientation.w;

        this->phi = Quart2EulerYaw(q_x, q_y, q_z, q_w); // rads

        this->linvel = msg.twist.twist.linear.x;
        this->angvel = msg.twist.twist.angular.z;

        // std::cout << "Updating pose, "<< "\n" << "x: " << msg->x << "\n";
        // std::cout << "y: " << msg->y << "\n";
        // std::cout << "Current angle: " << this->angle << "\n";
        // // std::cout << "linvel: " << msg->linear_velocity << "\n";
        // // std::cout << "angvel: " << msg->angular_velocity << "\n";
    }

    // Converts from Quartenion to Euler Yaw
    // From https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
    double Quart2EulerYaw(double q_x, double q_y, double q_z, double q_w)
    {
        double siny_cosp = 2.0 * (q_w * q_z + q_x * q_y);
        double cosy_cosp = 1.0 - (2.0 * (q_y * q_y + q_z * q_z));
        return std::atan2(siny_cosp, cosy_cosp);
    }
};

/* Function List */

bool instruction(Husky& Husky);

int main(int argc, char **argv)
{
    // Initialize ROS node
    ros::init(argc, argv, "husky_terminal_controller");

    //Initialize Husky Object
    Husky husky = Husky(MAXSPD, MAXROT);

    while(ros::ok()) // while ros is running, repeat this 
    {
        if(instruction(husky))
        {
            // exit is run, exit program
            return 0;
        }

        ros::spinOnce();
    }

    return 0;
}

// Instructions for main()
// Prompts for user input to determine what the passed in the Husky object should do
bool instruction(Husky& Husky)
{
    // Initialize Variables
    int command;
    double dist, speed, angle;
    bool isForward;

    // Ask for command
    std::cout << "Hello, this program has two functions. Please enter 1 or 2.\n";
    std::cout << "1) Specify movement and direction to Husky.\n";
    std::cout << "2) Specify location for Husky to travel to.\n";
    std::cout << "Any other number => Exit\n";
    while(!(std::cin >> command))
    {
        std::cout << "You have entered a wrong input, please only specify numbers. ";
        std::cin.clear();
        std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
    }
    
    if (command == 1)
    {
        std::cout << "What is the distance? ";
        while(!(std::cin >> dist))
        {
            std::cout << "You have entered a wrong input, please specify distance with numbers only.\n";
            std::cin.clear();
            std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
            std::cout << "What is the distance? ";
        }
        std::cout << "What is the speed to travel at? ";
        while(!(std::cin >> speed))
        {
            std::cout << "You have entered a wrong input, please specify speed with numbers only.\n";
            std::cin.clear();
            std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
            std::cout << "What is the speed to travel at? ";
        }
        std::cout << "Are you moving forwards? Answer 1 for true or 0 for false. ";
        while(!(std::cin >> isForward))
        {
            std::cout << "You have entered a wrong input, please specify 1 or 0.\n";
            std::cin.clear();
            std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
            std::cout << "Are you moving forwards? Answer 1 for true or 0 for false. ";
        }
        std::cout << "How many degrees to the left would you like to turn? ";
        while(!(std::cin >> angle))
        {
            std::cout << "You have entered a wrong input, please specify a number\n";
            std::cin.clear();
            std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
            std::cout << "How many degrees to the left would you like to turn? ";
        }

        Husky.move(dist, speed, isForward, angle*M_PI/180);
    }
    
    else if(command == 2)
    {
        // Update Position
        ros::spinOnce();
        
        // Initialize Variables
        double goto_x, goto_y, error;

        std::cout << "Current pose of Husky is\n";
        std::cout << "x: " << Husky.getX() << "\n";
        std::cout << "y: " << Husky.getY() << "\n";
        std::cout << "angle: " << Husky.getPhi()*180/M_PI << "\n";
        std::cout << "Where would you like the Husky to go to?\n";
        std::cout << "x = ";
        while(!(std::cin >> goto_x))
        {
            std::cout << "You have entered a wrong input, please specify with numbers only.\n";
            std::cin.clear();
            std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
            std::cout << "x = ";
        }
        std::cout << "y = ";
        while(!(std::cin >> goto_y))
        {
            std::cout << "You have entered a wrong input, please specify with numbers only.\n";
            std::cin.clear();
            std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
            std::cout << "y = ";
        }
        std::cout << "error threshold = ";
        while(!(std::cin >> error))
        {
            std::cout << "You have entered a wrong input, please specify with numbers only.\n";
            std::cin.clear();
            std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
            std::cout << "error = ";
        }

        Husky.smoothmove(goto_x, goto_y, error);
    }

    else // entered random number, exit
    {
        std::cout << "Exiting program...";
        return 1;
    }
}