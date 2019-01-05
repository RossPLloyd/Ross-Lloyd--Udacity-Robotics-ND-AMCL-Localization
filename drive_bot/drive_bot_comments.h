#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include "std_msgs/String.h"

std_msgs::String str_goal; // global variable to store goal data
std_msgs::String str_amcl; // global variable to store amcl pose data

class Waypoints {
	public: 
		// arrays to store robot current waypoint data and current position
        float current_goal[7];
		float current_position[7];
        // list of waypoints for the robot to go to. Not all 7 parameters are used in each case
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

// constructor. Arrays are initialised to avoid problems with dividing by zero when script starts up.
Waypoints(ros::NodeHandle &nh)
    : current_goal{0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 1.0},
      current_position{0.0, 0.0, 0.0, 0.0, 0.0, 5.0, 5.0}
{
    // define subscribers and publishers
    vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    amcl_sub = nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/amcl_pose",10, &Waypoints::poseAMCLCallback, this);
}

void poseAMCLCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
    // Optional conversion to rpy, removed for now 
    //tf2::Quaternion q(
    //msg->pose.pose.orientation.x,
    //msg->pose.pose.orientation.y,
    //msg->pose.pose.orientation.z,
    //msg->pose.pose.orientation.w);
    //tf2::Matrix3x3 m(q);
    //double roll, pitch, yaw;
    //m.getRPY(roll, pitch, yaw);
    
    //set current position array equal to parameters from amcl_pose subscriber
    current_position[0] = msg->pose.pose.position.x;
    current_position[1] = msg->pose.pose.position.y;
    current_position[2] = msg->pose.pose.position.z;
    current_position[3] = msg->pose.pose.orientation.x;
    current_position[4] = msg->pose.pose.orientation.y;
    current_position[5] = msg->pose.pose.orientation.z;
    current_position[6] = msg->pose.pose.orientation.w;
    //current_position[3] = roll;
    //current_position[4] = pitch;
    //current_position[5] = yaw;    
    //ROS_INFO("amcl pose = : %f , %f , %f , %f , %f , %f, %f", current_position[0], current_position[1], current_position[2], current_position[3], current_position[4], current_position[5], current_position[6]);    
}

// function to drive forward
void forward(){
    vel_msg.linear.x = 0.17;
    vel_pub.publish(vel_msg);    
}

// Some of the waypoints need a right or left turn. This could be combined into one function but this was
// neater
int rotate_right(){
     vel_msg.angular.z =-0.2;
     vel_pub.publish(vel_msg);
     //ROS_INFO("Rotating right");
     return 0;
}

int rotate_left(){
     vel_msg.angular.z =0.2;
     vel_pub.publish(vel_msg);
     //ROS_INFO("Rotating right");
     return 0;
}

int stop(){
     vel_msg.angular.z =0.0;
     vel_msg.linear.x = 0.0;
     vel_pub.publish(vel_msg);
     return 0;
}

// Make robot dance (indicates to user that path is finished along with message).
int celebrate(){
     vel_msg.angular.z =-100;
     vel_pub.publish(vel_msg);
     //ROS_INFO("Rotating right");
     return 0;
}

//populate the current goal from wp_points array using short for loop.
int pop_goal_from_array(int x){
    int i = x;
    for(int j=0; j<7; j++){
         current_goal[j] = wp[i][j];
            }
    return 0;
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
    else if(buffer == -1 && tracker ==0){
        ROS_INFO("Turning to waypoint 0");
        ++tracker;
    }
    else{
        }
    
}

/**void timer_callback(const ros::TimerEvent& event)
{
	amcl_sub = nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/amcl_pose",10, &Waypoints::poseAMCLCallback, this);
}**/

void decide_rotate(int count){
    //this function monitors how close the current rotational position is to the current desired pose given in the wp_points array
    // use lamda expression to define the ratio between the goal and current position:
    auto ratio = [=](float cur_pos, float cur_goal){return (cur_pos / cur_goal);};
    //ROS_INFO("*****Current GoalZ [%f]******", current_goal[5]);
    float z_acc = ratio(this->current_position[5], this->current_goal[5]); //PROBLEM: Starting position is zero. Try reversing the order so 0 is on top.
    //float w_acc = ratio(current_goal[6], current_position[6]);
    //ROS_INFO("Ratio5: [%f]", z_acc);
    float low = 0.995; //0.998;
    float high = 1.005;//1.002;
    // use lambda expression to define true / false test:
    auto test = [=](float rati, float lo, float hi){return !(rati < lo) && (rati < hi);};
    //auto stop_test = [](float value, float low, float high){return !(value > low) && !(value > high);};
    //ROS_INFO("Int value passed to rotate function: %d", count); //Continue with passing wp_count by reference.
    //ROS_INFO("Test evaluates to: %d ",((test(z_acc, low*z_acc, high*z_acc))));// && (test(w_acc, low*w_acc, high*w_acc))));
    //if(count == 0 || count == 2 || count == 4 || count == 6){
        //func = rotate_left();
    if(!((test(z_acc, low, high)))){//} && (test(w_acc, low, high)))){
    //if((current_goal[5] != current_position[5]) && (current_goal[6] != current_position[6])){    
        if(wp_count == 0 || wp_count == 2){
            rotate_right();
            //ROS_INFO("Rotating right");
            }
        else{
            rotate_left();
            //ROS_INFO("Rotating left");
        }
    }
    else{
    //else if(((current_goal[5]/current_position[5] > 0.98 && current_goal[5]/current_position[5]) < 1.02)){
    //else if((current_goal[5] == current_position[5]) && (current_goal[6] == current_position[6])){ 
        stop();
        //ROS_INFO("Stopped");
        //ROS_INFO("Accuracy of Orientation 5: %f percent", (current_position[5]/current_goal[5]));
        //ROS_INFO("Accuracy of Orientation 6: %f percent", (current_goal[6]/current_position[6]));
        wp_count++;
        //ROS_INFO("Value of wp count inside of stopping if statement: %d", wp_count);
        ros::Duration(1.5).sleep();
        }
}


void decide_move_forward(int count)
{
    //ROS_INFO("Int value passed to forward function: %d", count);
    //this function monitors how close the current rotational position is to the current desired pose given in the wp_points array
    // use lamda expression to define the ratio between the goal and current position:
    //ROS_INFO("*****Current GoalX [%f]****** Current PositionX [%f}", this->current_goal[0], this->current_position[0]);
    //ROS_INFO("*****Current GoalY [%f]****** Current PositionY [%f}", this->current_goal[1], this->current_position[1]);
    auto ratio1 = [=](float cur_pos, float cur_goal){return (cur_pos / cur_goal);};

    float x_acc = ratio1(this->current_position[0], this->current_goal[0]); //PROBLEM: Starting position is zero. Try reversing the order so 0 is on top.
    float y_acc = ratio1(this->current_position[1], this->current_goal[1]);
    //float w_acc = ratio(current_goal[6], current_position[6]);
    //ROS_INFO("RatioX: [%f], RatioY: [%f]", x_acc, y_acc);
    float low = 0.94;
    float high = 1.06;
    // use lambda expression to define true / false test:
    auto test = [=](float rati, float lo, float hi){return !(rati < lo) && (rati < hi);};
    //auto stop_test = [](float value, float low, float high){return !(value > low) && !(value > high);};
    //ROS_INFO("Test evaluates to: %d ",(test(x_acc, low*x_acc, high*x_acc) && (test(y_acc, low*y_acc, high*y_acc))));
    //if(count == 0 || count == 2 || count == 4 || count == 6){
        //func = rotate_left();
    if(!((test(x_acc, low, high) && test(y_acc, low, high))))
    {//} && (test(w_acc, low, high)))){
    //if((current_goal[5] != current_position[5]) && (current_goal[6] != current_position[6])){    
        forward();
    }
    else{
    //else if(((current_goal[5]/current_position[5] > 0.98 && current_goal[5]/current_position[5]) < 1.02)){
    //else if((current_goal[5] == current_position[5]) && (current_goal[6] == current_position[6])){ 
        stop();
       // ROS_INFO("Stopped");
        //ROS_INFO("Accuracy of Orientation x: %f percent -- Accuracy of Orientation x: %f percent", (current_position[0]/current_goal[0]), (current_position[1]/current_goal[1]));
        //ROS_INFO("Accuracy of Orientation 6: %f percent", (current_goal[6]/current_position[6]));
        wp_count++;
        //ROS_INFO("Value of wp count inside of stopping if statement: %d", wp_count);
        ros::Duration(1.5).sleep();
    }

}

};
