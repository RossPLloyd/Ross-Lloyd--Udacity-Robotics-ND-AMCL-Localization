#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include "std_msgs/String.h"

class Waypoints {
	public: 
		// msgs to store robot current waypoint data and current position
		geometry_msgs::Pose current_position;
        geometry_msgs::Pose current_goal;
        // list of waypoints for the robot to go to. Not all 7 parameters are used in each case
        // TODO: Read wp in from file
        float wp[6][7] = {
            {0.0, 0.0, 0.0, 0.0, 0.0, -0.959609576409, 0.281335139762},
            {-1.90124212399, -1.22099063492, 0.0, 0.0, 0.0, -0.958133706398, 0.286321149523},
            {-1.91177653191, -1.22028911003, 0.0, 0.0, 0.0, 1.0, 0.0},
            {-7.79428451344, -1.1984112, 0.0, 0.0, 0.0, 1.00, 0.00},
            {-7.89190328585, -1.092385, 0.0, 0.0, 0.0,  -0.703918, 0.710281},
            {-7.65617835157, -4.168655815795, 0.0, 0.0, 0.0, -0.70452631593, 0.709677863655}
};


        // declarations
        ros::NodeHandle nh;
        ros::Publisher vel_pub;
        geometry_msgs::Twist vel_msg;
        ros::Subscriber amcl_sub;
        // counters for keeping track of waypoint status
        int count = 0;
        int buffer = -1;
        int wp_count =0;
        int tracker=0;
        static constexpr double forward_speed = 0.17;
        static constexpr double r_angular_speed = -0.2;
        static constexpr double l_angular_speed = 0.2;
        static constexpr double stopped_speed = 0.0;
        static constexpr double celebrate_speed = -100.0;

// constructor
Waypoints(ros::NodeHandle &nh)

{
    // define subscribers and publishers
    vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    amcl_sub = nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/amcl_pose",10, &Waypoints::poseAMCLCallback, this);
}

void poseAMCLCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
    current_position = msg->pose.pose;  
}

// function to drive forward
void forward(){
    vel_msg.linear.x = forward_speed;
    vel_pub.publish(vel_msg);    
}

// Some of the waypoints need a right or left turn. TODO: combine into one function with "forward"
void rotate_right(){
     vel_msg.angular.z = r_angular_speed;
     vel_pub.publish(vel_msg);
     return;
}

void rotate_left(){
     vel_msg.angular.z = l_angular_speed;
     vel_pub.publish(vel_msg);
     return;
}

void stop(){
     vel_msg.angular.z = stopped_speed;
     vel_msg.linear.x = stopped_speed;
     vel_pub.publish(vel_msg);
     return;
}

// Make robot dance (indicates to user that path is finished along with message).
void celebrate(){
     vel_msg.angular.z = celebrate_speed;
     vel_pub.publish(vel_msg);
     return;
}

//populate the current goal from wp_points array using short for loop.
void pop_goal_from_array(int x){
    int i = x;
    current_goal.position.x = wp[i][0];
    current_goal.position.y = wp[i][1];
    current_goal.position.z = wp[i][2];
    current_goal.orientation.x = wp[i][3];
    current_goal.orientation.y = wp[i][4];
    current_goal.orientation.z = wp[i][5];
    current_goal.orientation.w = wp[i][6];
}

void print_WPinfo(int x){
    // some logic to make sure the right message is printed depending on circumstance
    if(buffer == x-1 && tracker ==1){
        if(x%2 ==0){
            ROS_INFO("Turning to waypoint %d", x);
        }
        else{
            ROS_INFO("Driving forward to waypoint %d", x);
        }
        buffer = x;
    }
    // special case for waypoint 0
    else if(buffer == -1 && tracker ==0){
        ROS_INFO("Turning to waypoint 0");
        ++tracker;
    }
    
}

// Define true / false test, monitors how close current goal and
// current position are and applies the threshold low and high for a little wiggle room
bool runtest(float rati, float lo, float hi){
    bool test = !(rati < lo) && (rati < hi);
    return test;
}


//this function mdecides which way to turn based on waypoint
void decide_rotate(int count){
    // define the ratio between the goal and current position: 
    float z_acc = current_position.orientation.z / current_goal.orientation.z;
    // apply a threshold for decision making
    float low = 0.995; 
    float high = 1.005;
    if(!((runtest(z_acc, low, high)))){
  
        if(wp_count == 0 || wp_count == 2){
            rotate_right();
        }
        else{
            rotate_left();
        }
    }
    else{

        stop();

        wp_count++;
        ros::Duration(1.5).sleep();
        }
}


void decide_move_forward(int count)
{
    float x_acc = current_position.position.x / current_goal.position.x;
    float y_acc = current_position.position.y / current_goal.position.y;
        // apply a threshold for decision making
    float low = 0.94;
    float high = 1.06;
    if(!((runtest(x_acc, low, high) && runtest(y_acc, low, high))))
    {   
        forward();
    }
    else{
        stop();
        wp_count++;
        ros::Duration(1.5).sleep();
        }

}

};
